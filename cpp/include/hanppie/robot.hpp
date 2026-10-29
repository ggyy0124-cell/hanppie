// RoboMaster robot control over the App UDP protocol session.
// C++11 port of src/hanppie/robot.py.
#ifndef HANPPIE_ROBOT_HPP
#define HANPPIE_ROBOT_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "hanppie/connection.hpp"
#include "hanppie/media.hpp"
#include "hanppie/protocol.hpp"
#include "hanppie/vision.hpp"

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

// Camera simple status push 02:e4, relayed by the HDVT node on S1; field layout
// from robot-core media/CameraStorageStatus.kt (S1 live-verified 2026-10-25).
struct CameraStatus {
    double receivedAt;
    int workMode;  // 0 photo, 1 video
    bool cardInserted;
    int cardState;  // 5 formatting, 8 full, 2/3/4/6/10/11 error
    bool recording;
    bool shooting;
    int recordingSeconds;

    bool formatting() const { return cardState == 5; }
    bool full() const { return cardState == 8; }
    bool cardError() const;
    bool canCapture() const {
        return cardInserted && !formatting() && !recording && !shooting && !full() && !cardError();
    }
};

bool decodeCameraStatus(const protocol::DussFrame& frame, CameraStatus& out);

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
    Vision vision;

    bool connected() const { return connection.connected(); }
    bool controlMode() const { return controlMode_; }
    bool armed() const { return armed_; }
    std::string robotIp() const { return connection.robotIp(); }
    AppConnectionInfo info() const { return connection.info(); }
    bool odometry(Odometry& out) const;
    bool gimbalTelemetry(GimbalTelemetry& out) const;
    bool chassisAttitude(ChassisAttitude& out) const;
    bool cameraStatus(CameraStatus& out) const;

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
    // Take a photo to the SD card, switching the camera to photo mode first
    // when the latest camera status reports video mode.
    RobotAck capture(double timeout = 1.0);
    // Start or stop SD card video recording (robot-core CameraCommands.setRecording).
    // Starting needs a fresh camera status with a usable card, switches to video
    // mode, and waits for the status to report recording; stopping waits for it
    // to clear. Throws std::runtime_error when the card or status does not allow it.
    void setRecording(bool active);
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
    bool waitForCameraStatus(CameraStatus& out, double after = 0.0, double timeoutSeconds = 1.5);

    void requireArmed() const;

private:
    Robot(const Robot&);
    Robot& operator=(const Robot&);

    std::vector<std::uint16_t> sendSetup(const std::vector<protocol::SetupEntry>& entries,
                                         bool includeFlags);
    void handleDuss(const protocol::DussFrame& frame);
    CameraStatus freshCameraStatus() const;
    void setCameraMode(int mode);
    bool awaitCameraStatus(double after, const std::function<bool(const CameraStatus&)>& matches);

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
    bool hasCameraStatus_;
    CameraStatus cameraStatus_;
};

}  // namespace hanppie

#endif  // HANPPIE_ROBOT_HPP
