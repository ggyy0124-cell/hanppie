// RoboMaster robot control over the App UDP protocol session.
// C++11 port of src/hanppie/robot.py.
#ifndef HANPPIE_ROBOT_HPP
#define HANPPIE_ROBOT_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "hanppie/connection.hpp"
#include "hanppie/media.hpp"
#include "hanppie/protocol.hpp"

namespace hanppie {

const std::uint16_t LED_ARMOR_ALL_MASK = 0xFF;
const std::uint32_t LED_SUBCOMPONENT_ALL = 0x3F;
const std::uint32_t LED_SUBCOMPONENT_TOP = 0x30;
const std::uint32_t LED_SUBCOMPONENT_BOTTOM = 0x0F;

struct RobotAck {
    std::uint16_t sequence;
    std::uint8_t sender;
    std::uint8_t receiver;
    std::uint8_t cmdset;
    std::uint8_t cmdid;
    Bytes payload;

    bool hasReturnCode() const { return !payload.empty(); }
    int returnCode() const { return payload.empty() ? -1 : payload[0]; }
    bool accepted() const { return payload.empty() || payload[0] == 0; }
};

// Raised when the robot acknowledges a request with a non-zero return code.
class RequestRejected : public std::runtime_error {
public:
    RequestRejected(const std::string& what, int code) : std::runtime_error(what), returnCode(code) {}
    int returnCode;
};

struct Odometry {
    double receivedAt;
    std::uint16_t sequence;
    int batteryPercent;
    float headingLike;
    std::vector<float> values;
    std::string payloadHex;

    bool hasPosition() const { return values.size() >= 2; }
    float x() const { return values.at(0); }
    float y() const { return values.at(1); }
    bool rawMotionValues(float& a, float& b, float& c) const;
};

struct GimbalTelemetry {
    double receivedAt;
    std::uint16_t sequence;
    std::int16_t values[4];
    std::uint8_t flag;
    std::string payloadHex;

    // SDK GimbalPosSubject wire angles, not a calibrated world orientation.
    double groundYawDegrees() const { return values[0] / 10.0; }
    double groundPitchDegrees() const { return values[1] / 10.0; }
    double yawDegrees() const { return values[2] / 10.0; }
    double pitchDegrees() const { return values[3] / 10.0; }
};

// Chassis attitude from App DDS topic 13, packet-tested in robot-core
// (telemetry/Telemetry.kt ChassisSubscriptions). Angles are degrees in -180..180.
struct ChassisAttitude {
    double receivedAt;
    float yawDegrees;
    float pitchDegrees;
    float rollDegrees;
};

bool decodeOdometry(const protocol::DussFrame& frame, Odometry& out);
bool decodeGimbal(const protocol::DussFrame& frame, GimbalTelemetry& out);
bool decodeChassisAttitude(const protocol::DussFrame& frame, ChassisAttitude& out);
// DDS add-subscription payload: subscriber node 2, one UID, periodic, no timestamp.
Bytes buildDdsSubscriptionPayload(std::uint8_t messageId, std::uint64_t uid, std::uint16_t hz);

// Encode App control-channel chassis velocity in m/s and degrees/s.
Bytes buildChassisControlPayload(double x, double y, double z);
// Encode App direct-channel gimbal velocity in degrees/s.
Bytes buildGimbalSpeedPayload(double pitch, double yaw);
// effect is off, on, solid, breath, or flash.
Bytes buildLedPayload(std::uint32_t component, int red, int green, int blue,
                      const std::string& effect);

class Robot;

class Chassis {
public:
    explicit Chassis(Robot& robot) : robot_(robot) {}

    void driveSpeed(double x = 0.0, double y = 0.0, double z = 0.0, double leaseSeconds = 0.25);
    void stop();
    RobotAck driveWheels(int w1 = 0, int w2 = 0, int w3 = 0, int w4 = 0, double timeout = 1.0);

private:
    Robot& robot_;
};

class Gimbal {
public:
    explicit Gimbal(Robot& robot) : robot_(robot) {}

    void driveSpeed(double pitchSpeed = 0.0, double yawSpeed = 0.0, double leaseSeconds = 0.25);
    void stop();

private:
    Robot& robot_;
};

// High-level robot control over the RoboMaster App UDP protocol session.
class Robot {
public:
    Robot(const std::string& robotIp, const std::string& appid,
          const std::string& localIp = "0.0.0.0", bool debug = false);
    // Closes the session (exits control mode) if still connected.
    ~Robot();

    AppConnection connection;
    media::Camera camera;
    media::Audio audio;
    Chassis chassis;
    Gimbal gimbal;

    bool connected() const { return connection.connected(); }
    bool controlMode() const { return controlMode_; }
    bool armed() const { return armed_; }
    std::string robotIp() const { return connection.robotIp(); }
    AppConnectionInfo info() const { return connection.info(); }
    bool odometry(Odometry& out) const;
    bool gimbalTelemetry(GimbalTelemetry& out) const;
    bool chassisAttitude(ChassisAttitude& out) const;

    bool initialize(const std::string& connType = "sta", const std::string& protoType = "udp",
                    double timeoutSeconds = 20.0);
    std::vector<std::uint16_t> enterControlMode();
    void exitControlMode();
    void arm();
    void disarm();
    void stop();
    void close();

    RobotAck request(std::uint8_t sender, std::uint8_t receiver, std::uint8_t cmdset,
                     std::uint8_t cmdid, const Bytes& payload = Bytes(), double timeout = 1.0);

    // component is all, top, gimbal, or bottom.
    RobotAck setLed(const std::string& component = "all", int red = 255, int green = 255,
                    int blue = 255, const std::string& effect = "on", double timeout = 1.0);
    RobotAck setMuzzleLed(bool fire, bool enabled, double timeout = 1.0);
    RobotAck playSound(std::uint32_t soundId, double timeout = 1.0);
    RobotAck setSpeakerVolume(int volume, double timeout = 1.0);
    RobotAck capture(double timeout = 1.0);
    void fireInfrared(double leaseSeconds = 0.12);
    // Fire one gel bead through the S1-verified 3f:51 route (robot-core
    // AppSession.fireGelOnce), lighting the muzzle and blaster LEDs for 400 ms.
    // Blocks for that window.
    void fireGel();
    // Subscribe to the chassis attitude topic (10 Hz). Idempotent.
    void subscribeChassisAttitude();

    bool waitForOdometry(Odometry& out, double after = 0.0, double timeoutSeconds = 2.0);
    bool waitForGimbal(GimbalTelemetry& out, double after = 0.0, double timeoutSeconds = 2.0);
    bool waitForChassisAttitude(ChassisAttitude& out, double after = 0.0,
                                double timeoutSeconds = 2.0);

    void requireArmed() const;

private:
    Robot(const Robot&);
    Robot& operator=(const Robot&);

    std::vector<std::uint16_t> sendSetup(const std::vector<protocol::SetupEntry>& entries,
                                         bool includeFlags);
    void handleDuss(const protocol::DussFrame& frame);

    std::atomic<bool> controlMode_;
    std::atomic<bool> armed_;
    mutable std::mutex telemetryLock_;
    std::condition_variable telemetryCondition_;
    bool hasOdometry_;
    Odometry odometry_;
    bool hasGimbal_;
    GimbalTelemetry gimbal_;
    bool hasAttitude_;
    ChassisAttitude attitude_;
};

}  // namespace hanppie

#endif  // HANPPIE_ROBOT_HPP
