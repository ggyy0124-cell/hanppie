#include "hanppie/ctrl.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hanppie {
namespace ctrl {

namespace {

const double PI = 3.14159265358979323846;
// Speed caps of the App control frames (robot.cpp payload builders).
const double CHASSIS_SPEED_MAX = 1.0;
const double CHASSIS_ROTATE_MAX = 150.0;
const double GIMBAL_SPEED_MAX = 120.0;
// Motion thread period; well inside the 0.25 s command lease.
const double RENEW_SECONDS = 0.05;
const double LEASE_SECONDS = 0.25;
// Closed-loop gain and limits follow the App's verified gimbal centering
// (shared ui/robot/remote/GimbalCentering.kt): 3 deg/s per degree, 1 degree
// tolerance, telemetry older than 0.5 s aborts.
const double APPROACH_GAIN = 3.0;
const double APPROACH_MIN_RATE = 3.0;
const double TELEMETRY_STALE_SECONDS = 0.5;

void checkRange(double value, double low, double high, const char* name) {
    if (!(value >= low && value <= high)) {
        throw std::invalid_argument(std::string(name) + " out of range");
    }
}

double clampValue(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
}

int rotationSign(int direction) {
    if (direction == rm_define::clockwise) return 1;
    if (direction == rm_define::anticlockwise) return -1;
    throw std::invalid_argument("direction must be rm_define::clockwise or anticlockwise");
}

GimbalTelemetry freshGimbal(Robot& robot, double after) {
    GimbalTelemetry sample;
    if (!robot.waitForGimbal(sample, after, TELEMETRY_STALE_SECONDS)) {
        throw std::runtime_error("gimbal telemetry unavailable");
    }
    return sample;
}

ChassisAttitude freshAttitude(Robot& robot, double after) {
    ChassisAttitude sample;
    if (!robot.waitForChassisAttitude(sample, after, TELEMETRY_STALE_SECONDS)) {
        throw std::runtime_error("chassis attitude telemetry unavailable");
    }
    return sample;
}

// Drive the gimbal to absolute chassis-relative angles. Axes passed as NAN are
// left alone. Fails rather than guessing when telemetry stops.
void gimbalTo(RobotCtrl& rm, double yaw, double pitch, double maxRate) {
    double deadline = monotonicSeconds() + 10.0;
    double after = 0.0;
    try {
        for (;;) {
            GimbalTelemetry sample = freshGimbal(rm.robot, after);
            after = sample.receivedAt;
            double yawRate = std::isnan(yaw) ? 0.0 : approachRate(yaw - sample.yawDegrees(), maxRate);
            double pitchRate =
                std::isnan(pitch) ? 0.0 : approachRate(pitch - sample.pitchDegrees(), maxRate);
            if (yawRate == 0.0 && pitchRate == 0.0) break;
            if (monotonicSeconds() >= deadline) throw std::runtime_error("gimbal move timed out");
            rm.setGimbal(pitchRate, yawRate);
        }
    } catch (...) {
        rm.setGimbal(0, 0);
        throw;
    }
    rm.setGimbal(0, 0);
}

}  // namespace

std::pair<double, double> directionVelocity(double directionAngle, double speed) {
    double radians = directionAngle * PI / 180.0;
    return std::make_pair(std::cos(radians) * speed, std::sin(radians) * speed);
}

double approachRate(double error, double maxRate, double tolerance) {
    if (std::fabs(error) <= tolerance) return 0.0;
    double rate = clampValue(std::fabs(error) * APPROACH_GAIN, APPROACH_MIN_RATE, maxRate);
    return error > 0 ? rate : -rate;
}

double angleDifference(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d <= -180.0) d += 360.0;
    if (d > 180.0) d -= 360.0;
    return d;
}

// ---- RobotCtrl -------------------------------------------------------------

RobotCtrl::RobotCtrl(Robot& robot_)
    : robot(robot_),
      chassis_ctrl(*this),
      gimbal_ctrl(*this),
      led_ctrl(*this),
      gun_ctrl(*this),
      ir_blaster_ctrl(*this),
      media_ctrl(*this),
      chassisHeldUntil_(0.0),
      stopping_(false),
      gimbalMoving_(false) {
    MotionSetpoint zero = {0, 0, 0, 0, 0};
    setpoint_ = zero;
    robot.enterControlMode();
    robot.arm();
    robot.subscribeChassisAttitude();
    thread_ = std::thread(&RobotCtrl::run, this);
}

RobotCtrl::~RobotCtrl() {
    {
        std::lock_guard<std::mutex> lock(lock_);
        stopping_ = true;
    }
    wake_.notify_all();
    thread_.join();
    try {
        if (robot.connected()) {
            robot.chassis.stop();
            robot.gimbal.stop();
            robot.exitControlMode();
        } else {
            robot.disarm();
        }
    } catch (...) {
        // Destructors must not throw; the SDK lease already zeroes motion.
    }
}

void RobotCtrl::sleep(double seconds) {
    if (seconds > 0) std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

void RobotCtrl::setChassis(double x, double y, double z) {
    {
        std::lock_guard<std::mutex> lock(lock_);
        setpoint_.x = x;
        setpoint_.y = y;
        setpoint_.z = z;
    }
    wake_.notify_all();
}

void RobotCtrl::setGimbal(double pitch, double yaw) {
    {
        std::lock_guard<std::mutex> lock(lock_);
        setpoint_.pitch = pitch;
        setpoint_.yaw = yaw;
    }
    wake_.notify_all();
}

MotionSetpoint RobotCtrl::setpoint() const {
    std::lock_guard<std::mutex> lock(lock_);
    return setpoint_;
}

void RobotCtrl::holdChassisFor(double seconds) {
    std::lock_guard<std::mutex> lock(lock_);
    chassisHeldUntil_ = monotonicSeconds() + seconds;
}

void RobotCtrl::run() {
    std::unique_lock<std::mutex> lock(lock_);
    while (!stopping_) {
        MotionSetpoint target = setpoint_;
        bool held = monotonicSeconds() < chassisHeldUntil_;
        lock.unlock();
        try {
            apply(target, held);
        } catch (...) {
            // Disconnected or disarmed; the lease stops the robot. Keep looping
            // so motion resumes if the caller re-arms.
        }
        lock.lock();
        wake_.wait_for(lock, std::chrono::duration<double>(RENEW_SECONDS));
    }
}

void RobotCtrl::apply(const MotionSetpoint& target, bool chassisHeld) {
    if (!robot.connected() || !robot.armed()) return;
    // Infrared firing borrows the control frame; leave it alone meanwhile.
    if (!chassisHeld) {
        if (target.x != 0 || target.y != 0 || target.z != 0) {
            robot.chassis.driveSpeed(target.x, target.y, target.z, LEASE_SECONDS);
        } else {
            robot.chassis.stop();
        }
    }
    if (target.pitch != 0 || target.yaw != 0) {
        robot.gimbal.driveSpeed(target.pitch, target.yaw, LEASE_SECONDS);
        gimbalMoving_ = true;
    } else if (gimbalMoving_) {
        robot.gimbal.stop();
        gimbalMoving_ = false;
    }
}

// ---- ChassisCtrl -----------------------------------------------------------

void ChassisCtrl::set_trans_speed(double speed) {
    checkRange(speed, 0, 3.5, "speed");
    speed_ = clampValue(speed, 0, CHASSIS_SPEED_MAX);
}

void ChassisCtrl::set_rotate_speed(double speed) {
    checkRange(speed, 0, 600, "speed");
    rotateSpeed_ = clampValue(speed, 0, CHASSIS_ROTATE_MAX);
}

void ChassisCtrl::move(double direction_angle) {
    checkRange(direction_angle, -180, 180, "direction_angle");
    std::pair<double, double> v = directionVelocity(direction_angle, speed_);
    rm_.setChassis(v.first, v.second, 0);
}

void ChassisCtrl::move_with_time(double direction_angle, double seconds) {
    checkRange(seconds, 0, 20, "time");
    move(direction_angle);
    RobotCtrl::sleep(seconds);
    stop();
}

void ChassisCtrl::move_with_distance(double direction_angle, double distance) {
    checkRange(direction_angle, -180, 180, "direction_angle");
    checkRange(distance, 0, 5, "distance");
    if (speed_ == 0) return;
    move(direction_angle);
    RobotCtrl::sleep(distance / speed_);
    stop();
}

void ChassisCtrl::move_with_speed(double speed_x, double speed_y, double speed_z) {
    checkRange(speed_x, -3.5, 3.5, "speed_x");
    checkRange(speed_y, -3.5, 3.5, "speed_y");
    checkRange(speed_z, -600, 600, "speed_z");
    rm_.setChassis(clampValue(speed_x, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX),
                   clampValue(speed_y, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX),
                   clampValue(speed_z, -CHASSIS_ROTATE_MAX, CHASSIS_ROTATE_MAX));
}

double ChassisCtrl::rotateRate(int direction, double speed) const {
    return rotationSign(direction) * clampValue(speed, 0, CHASSIS_ROTATE_MAX);
}

void ChassisCtrl::rotate(int direction) { rm_.setChassis(0, 0, rotateRate(direction, rotateSpeed_)); }

void ChassisCtrl::rotate_with_speed(int direction, double speed) {
    checkRange(speed, 0, 600, "speed");
    rm_.setChassis(0, 0, rotateRate(direction, speed));
}

void ChassisCtrl::rotate_with_time(int direction, double seconds) {
    checkRange(seconds, 0, 20, "time");
    rotate(direction);
    RobotCtrl::sleep(seconds);
    stop();
}

void ChassisCtrl::rotate_with_degree(int direction, double degree) {
    int sign = rotationSign(direction);
    checkRange(degree, 0, 1800, "degree");
    if (rotateSpeed_ == 0 || degree == 0) return;
    // Accumulate unwrapped yaw change; the magnitude is used so the result
    // does not depend on the attitude topic's yaw sign convention.
    double deadline = monotonicSeconds() + degree / APPROACH_MIN_RATE + 5.0;
    try {
        ChassisAttitude last = freshAttitude(rm_.robot, 0.0);
        double turned = 0.0;
        for (;;) {
            double rate = approachRate(degree - turned, rotateSpeed_);
            if (rate <= 0.0) break;
            if (monotonicSeconds() >= deadline) throw std::runtime_error("chassis rotate timed out");
            rm_.setChassis(0, 0, sign * rate);
            ChassisAttitude sample = freshAttitude(rm_.robot, last.receivedAt);
            turned += std::fabs(angleDifference(sample.yawDegrees, last.yawDegrees));
            last = sample;
        }
    } catch (...) {
        stop();
        throw;
    }
    stop();
}

void ChassisCtrl::move_and_rotate(double direction_angle, int direction) {
    checkRange(direction_angle, -180, 180, "direction_angle");
    std::pair<double, double> v = directionVelocity(direction_angle, speed_);
    rm_.setChassis(v.first, v.second, rotateRate(direction, rotateSpeed_));
}

void ChassisCtrl::stop() { rm_.setChassis(0, 0, 0); }

double ChassisCtrl::get_attitude(int attitude) {
    ChassisAttitude sample;
    if (!rm_.robot.chassisAttitude(sample)) sample = freshAttitude(rm_.robot, 0.0);
    if (attitude == rm_define::chassis_pitch) return sample.pitchDegrees;
    if (attitude == rm_define::chassis_roll) return sample.rollDegrees;
    if (attitude == rm_define::chassis_yaw) return sample.yawDegrees;
    throw std::invalid_argument("attitude must be chassis_pitch, chassis_roll or chassis_yaw");
}

// ---- GimbalCtrl ------------------------------------------------------------

void GimbalCtrl::set_rotate_speed(double speed) {
    checkRange(speed, 0, 540, "speed");
    speed_ = clampValue(speed, 0, GIMBAL_SPEED_MAX);
}

void GimbalCtrl::rotate(int direction) {
    if (direction == rm_define::gimbal_up) {
        rm_.setGimbal(speed_, 0);
    } else if (direction == rm_define::gimbal_down) {
        rm_.setGimbal(-speed_, 0);
    } else if (direction == rm_define::gimbal_right) {
        rm_.setGimbal(0, speed_);
    } else if (direction == rm_define::gimbal_left) {
        rm_.setGimbal(0, -speed_);
    } else {
        throw std::invalid_argument("direction must be gimbal_up, gimbal_down, gimbal_right or gimbal_left");
    }
}

void GimbalCtrl::rotate_with_speed(double yaw_speed, double pitch_speed) {
    checkRange(yaw_speed, -540, 540, "yaw_speed");
    checkRange(pitch_speed, -540, 540, "pitch_speed");
    rm_.setGimbal(clampValue(pitch_speed, -GIMBAL_SPEED_MAX, GIMBAL_SPEED_MAX),
                  clampValue(yaw_speed, -GIMBAL_SPEED_MAX, GIMBAL_SPEED_MAX));
}

void GimbalCtrl::rotate_with_degree(int direction, double degree) {
    GimbalTelemetry now = freshGimbal(rm_.robot, 0.0);
    if (direction == rm_define::gimbal_up) {
        gimbalTo(rm_, NAN, now.pitchDegrees() + degree, speed_);
    } else if (direction == rm_define::gimbal_down) {
        gimbalTo(rm_, NAN, now.pitchDegrees() - degree, speed_);
    } else if (direction == rm_define::gimbal_right) {
        gimbalTo(rm_, now.yawDegrees() + degree, NAN, speed_);
    } else if (direction == rm_define::gimbal_left) {
        gimbalTo(rm_, now.yawDegrees() - degree, NAN, speed_);
    } else {
        throw std::invalid_argument("direction must be gimbal_up, gimbal_down, gimbal_right or gimbal_left");
    }
}

void GimbalCtrl::yaw_ctrl(double degree) {
    checkRange(degree, -250, 250, "degree");
    gimbalTo(rm_, degree, NAN, speed_);
}

void GimbalCtrl::pitch_ctrl(double degree) {
    checkRange(degree, -20, 35, "degree");
    gimbalTo(rm_, NAN, degree, speed_);
}

void GimbalCtrl::angle_ctrl(double yaw, double pitch) {
    checkRange(yaw, -250, 500, "yaw");
    checkRange(pitch, -20, 55, "pitch");
    gimbalTo(rm_, yaw, pitch, speed_);
}

void GimbalCtrl::recenter() { gimbalTo(rm_, 0, 0, speed_); }

void GimbalCtrl::stop() { rm_.setGimbal(0, 0); }

double GimbalCtrl::get_axis_angle(int axis) {
    GimbalTelemetry sample;
    if (!rm_.robot.gimbalTelemetry(sample)) sample = freshGimbal(rm_.robot, 0.0);
    if (axis == rm_define::gimbal_axis_pitch) return sample.pitchDegrees();
    if (axis == rm_define::gimbal_axis_yaw) return sample.yawDegrees();
    throw std::invalid_argument("axis must be gimbal_axis_pitch or gimbal_axis_yaw");
}

// ---- LedCtrl / blasters / media --------------------------------------------

void LedCtrl::set(int comp, int r, int g, int b, int effect) {
    const char* name;
    if (effect == rm_define::effect_always_on) {
        name = "on";
    } else if (effect == rm_define::effect_always_off) {
        name = "off";
    } else if (effect == rm_define::effect_breath) {
        name = "breath";
    } else if (effect == rm_define::effect_flash) {
        name = "flash";
    } else {
        throw std::invalid_argument("effect must be effect_always_on, effect_always_off, effect_breath or effect_flash");
    }
    rm_.robot.request(
        protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV, protocol::CMDSET_RM,
        protocol::CMD_RM_LED_COLOR_SET,
        buildLedPayload(static_cast<std::uint32_t>(comp) & rm_define::armor_all, r, g, b, name));
}

void LedCtrl::set_top_led(int comp, int r, int g, int b, int effect) {
    set(comp & rm_define::armor_top_all, r, g, b, effect);
}

void LedCtrl::set_bottom_led(int comp, int r, int g, int b, int effect) {
    set(comp & rm_define::armor_bottom_all, r, g, b, effect);
}

void LedCtrl::turn_off(int comp) { set(comp, 0, 0, 0, rm_define::effect_always_off); }

void LedCtrl::gun_led_on() { rm_.robot.setMuzzleLed(false, true); }

void LedCtrl::gun_led_off() { rm_.robot.setMuzzleLed(false, false); }

void GunCtrl::fire_once() { rm_.robot.fireGel(); }

void IrBlasterCtrl::fire_once() {
    const double trigger = 0.12;
    rm_.holdChassisFor(trigger + RENEW_SECONDS);
    rm_.robot.fireInfrared(trigger);
}

void MediaCtrl::play_sound(std::uint32_t sound_id) { rm_.robot.playSound(sound_id); }

void MediaCtrl::capture() { rm_.robot.capture(); }

}  // namespace ctrl
}  // namespace hanppie
