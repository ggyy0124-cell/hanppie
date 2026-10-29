// Lab-style control API: S1 Lab script names and semantics, executed on the
// host over the App protocol session instead of inside the robot.
//
// Names, argument order, units and ranges follow the S1 onboard Lab runtime
// (assets/s1-system/data/dji_scratch/src/robomaster/rm_ctrl.py and rm_define.py)
// so Lab scripts port almost line by line. They deliberately keep Lab's
// snake_case instead of the SDK's camelCase.
//
// Differences from onboard Lab, all caused by running remotely:
// - Angle targets (chassis rotate_with_degree, gimbal *_ctrl / rotate_with_degree)
//   close the loop on 10 Hz telemetry over Wi-Fi; expect about a degree of error.
// - move_with_distance has no verified position feedback over the App session
//   and is open loop (distance / speed). Lab uses an onboard position command.
// - App control frames cap chassis speed at 1 m/s and 150 deg/s, and gimbal
//   speed at 120 deg/s; larger Lab values are clamped.
#ifndef HANPPIE_CTRL_HPP
#define HANPPIE_CTRL_HPP

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <utility>

#include "hanppie/robot.hpp"

namespace hanppie {
namespace ctrl {

// Subset of the onboard rm_define constants used by this API; values match Lab.
namespace rm_define {
const int clockwise = 6;
const int anticlockwise = 5;

const int gimbal_up = 1;
const int gimbal_down = 2;
const int gimbal_right = 3;
const int gimbal_left = 4;
const int gimbal_axis_pitch = 0;
const int gimbal_axis_yaw = 1;

const int chassis_pitch = 1;
const int chassis_roll = 2;
const int chassis_yaw = 3;

const int armor_bottom_back = 0x1;
const int armor_bottom_front = 0x2;
const int armor_bottom_left = 0x4;
const int armor_bottom_right = 0x8;
const int armor_top_left = 0x10;
const int armor_top_right = 0x20;
const int armor_top_all = 0x30;
const int armor_bottom_all = 0xF;
const int armor_all = 0x3F;

const int effect_always_on = 0;
const int effect_always_off = 1;
const int effect_breath = 2;
const int effect_flash = 3;

const std::uint32_t media_sound_attacked = 0x101;
const std::uint32_t media_sound_shoot = 0x102;
const std::uint32_t media_sound_scanning = 0x103;
const std::uint32_t media_sound_recognize_success = 0x104;
const std::uint32_t media_sound_gimbal_rotate = 0x105;
const std::uint32_t media_sound_count_down = 0x106;
// Musical notes 1C..3B are 0x107..0x12A, as in Lab.
const std::uint32_t media_sound_solmization_1C = 0x107;
const std::uint32_t media_sound_solmization_3B = 0x12A;
}  // namespace rm_define

class RobotCtrl;

// Velocity setpoints renewed by RobotCtrl's motion thread so continuous Lab
// actions (move, rotate) keep running until stopped, while the SDK's lease
// still zeroes motion if the program stalls or disconnects.
struct MotionSetpoint {
    double x, y, z;          // chassis m/s, m/s, deg/s
    double pitch, yaw;       // gimbal deg/s
};

class ChassisCtrl {
public:
    explicit ChassisCtrl(RobotCtrl& rm) : rm_(rm), speed_(0.5), rotateSpeed_(30.0) {}

    void set_trans_speed(double speed);   // m/s, 0..1 (Lab allows 3.5)
    void set_rotate_speed(double speed);  // deg/s, 0..150 (Lab allows 600)

    // direction_angle in degrees, -180..180; 0 is forward, 90 is right.
    void move(double direction_angle);
    void move_with_time(double direction_angle, double seconds);
    // Open loop: runs for distance / trans speed. Distance in metres, 0..5.
    void move_with_distance(double direction_angle, double distance);
    // x forward, y right (m/s), z clockwise (deg/s); continuous until stop().
    void move_with_speed(double speed_x = 0, double speed_y = 0, double speed_z = 0);

    void rotate(int direction);
    void rotate_with_speed(int direction, double speed);
    void rotate_with_time(int direction, double seconds);
    // Closed loop on chassis attitude yaw. Degrees 0..1800.
    void rotate_with_degree(int direction, double degree);
    void move_and_rotate(double direction_angle, int direction);

    void stop();
    // Degrees; attitude is chassis_pitch, chassis_roll or chassis_yaw.
    double get_attitude(int attitude);

private:
    double rotateRate(int direction, double speed) const;
    RobotCtrl& rm_;
    double speed_;
    double rotateSpeed_;
};

class GimbalCtrl {
public:
    explicit GimbalCtrl(RobotCtrl& rm) : rm_(rm), speed_(30.0) {}

    void set_rotate_speed(double speed);  // deg/s, 0..120 (Lab allows 540)
    void rotate(int direction);           // gimbal_up/down/left/right, continuous
    // yaw positive is right, pitch positive is up (deg/s); continuous.
    void rotate_with_speed(double yaw_speed, double pitch_speed);
    // Relative move on one axis. Degrees.
    void rotate_with_degree(int direction, double degree);
    // Absolute angles relative to the chassis, closed loop on gimbal telemetry.
    void yaw_ctrl(double degree);    // -250..250
    void pitch_ctrl(double degree);  // -20..35
    void angle_ctrl(double yaw, double pitch);
    void recenter();
    void stop();
    double get_axis_angle(int axis);  // gimbal_axis_pitch or gimbal_axis_yaw

private:
    RobotCtrl& rm_;
    double speed_;
};

class LedCtrl {
public:
    explicit LedCtrl(RobotCtrl& rm) : rm_(rm) {}

    // comp is an armor_* mask; effect is effect_always_on/off, breath or flash.
    void set_top_led(int comp, int r, int g, int b, int effect);
    void set_bottom_led(int comp, int r, int g, int b, int effect);
    void turn_off(int comp);
    void gun_led_on();
    void gun_led_off();

private:
    void set(int comp, int r, int g, int b, int effect);
    RobotCtrl& rm_;
};

class GunCtrl {
public:
    explicit GunCtrl(RobotCtrl& rm) : rm_(rm) {}
    void fire_once();  // gel bead; blocks about 400 ms

private:
    RobotCtrl& rm_;
};

class IrBlasterCtrl {
public:
    explicit IrBlasterCtrl(RobotCtrl& rm) : rm_(rm) {}
    void fire_once();

private:
    RobotCtrl& rm_;
};

class MediaCtrl {
public:
    explicit MediaCtrl(RobotCtrl& rm) : rm_(rm) {}
    void play_sound(std::uint32_t sound_id);
    void capture();

private:
    RobotCtrl& rm_;
};

// Entry point, the equivalent of a Lab script's global controllers:
//
//   hanppie::Robot robot("192.168.2.1", "0123abcd");
//   robot.initialize();
//   hanppie::ctrl::RobotCtrl rm(robot);   // enters control mode and arms
//   rm.chassis_ctrl.set_trans_speed(0.5);
//   rm.chassis_ctrl.move_with_time(0, 2);
//   rm.gimbal_ctrl.angle_ctrl(30, 10);
//   rm.gun_ctrl.fire_once();
//
// The destructor stops all motion, disarms and leaves control mode.
class RobotCtrl {
public:
    explicit RobotCtrl(Robot& robot);
    ~RobotCtrl();

    Robot& robot;
    ChassisCtrl chassis_ctrl;
    GimbalCtrl gimbal_ctrl;
    LedCtrl led_ctrl;
    GunCtrl gun_ctrl;
    IrBlasterCtrl ir_blaster_ctrl;
    MediaCtrl media_ctrl;

    // Lab time.sleep equivalent.
    static void sleep(double seconds);

    // Internal: shared by the controllers.
    void setChassis(double x, double y, double z);
    void setGimbal(double pitch, double yaw);
    MotionSetpoint setpoint() const;
    void holdChassisFor(double seconds);

private:
    RobotCtrl(const RobotCtrl&);
    RobotCtrl& operator=(const RobotCtrl&);
    void run();
    void apply(const MotionSetpoint& target, bool chassisHeld);

    mutable std::mutex lock_;
    std::condition_variable wake_;
    MotionSetpoint setpoint_;
    double chassisHeldUntil_;
    bool stopping_;
    bool gimbalMoving_;  // motion thread only
    std::thread thread_;
};

// Pure helpers, exposed for tests.
// Chassis velocity (x forward, y right) for a Lab direction angle and speed.
std::pair<double, double> directionVelocity(double directionAngle, double speed);
// Proportional rate toward target with Lab-like gain, tolerance and clamp;
// 0 once within tolerance.
double approachRate(double error, double maxRate, double tolerance = 1.0);
// Shortest signed difference a - b in degrees, in (-180, 180].
double angleDifference(double a, double b);

}  // namespace ctrl
}  // namespace hanppie

#endif  // HANPPIE_CTRL_HPP
