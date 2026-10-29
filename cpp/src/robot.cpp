#include "hanppie/robot.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

namespace hanppie {

namespace {

using namespace protocol;

const char* const PAIR_HASH_1 = "ba7dc15a96c84f408e436c7bca716ae67b2188f68100b217";
const char* const PAIR_HASH_2 = "ca01dd0a449f4c8f844008cc9aa9140e56b47e09372be5b2";

// Fixed App subscription, corroborated by the macOS SubscribeManager and SDK.
// See docs/architecture/robomaster.md, macOS client analysis; preserve the captured bytes.
const std::uint8_t GIMBAL_SUBSCRIPTION_ID = 0x0A;
const std::uint64_t GIMBAL_SUBSCRIPTION_UID = 0x00020009F79B3C97ULL;
const std::uint16_t GIMBAL_SUBSCRIPTION_HZ = 10;

const char* const IR_GUN_CONFIG =
    "0d000000e903000000000000ea03000001000000eb03000020bf0200ec030000b0040000"
    "ed03000064000000ef0300000a000000f003000000000000f1030000b80b0000f2030000"
    "dc0500000604000001000000070400000000000008040000010000000904000001000000"
    "05000000dd050000dc050000de050000c4090000df050000b80b0000e0050000b80b0000"
    "4006000000879303050000004d0400003075000001000000de0500004e04000010270000"
    "01000000dd0500004f0400003075000001000000df050000500400001027000001000000"
    "e0050000b0040000000000000100000040060000";

const char* const TRIGGER_CONTROL = "0000042000010840000230";

// Read-only chassis attitude topic verified on the native App session; see
// robot-core telemetry/Telemetry.kt ChassisSubscriptions.
const std::uint8_t CHASSIS_ATTITUDE_SUBSCRIPTION_ID = 13;
const std::uint64_t CHASSIS_ATTITUDE_SUBSCRIPTION_UID = 0x000200096B986306ULL;
const std::uint16_t CHASSIS_ATTITUDE_SUBSCRIPTION_HZ = 10;

// S1-verified gel firing LED payloads; see robot-core remote/RemoteControl.kt.
const char* const MUZZLE_FIRE_LED_ON = "40000000ff0001ffffff6401000100";
const char* const MUZZLE_FIRE_LED_OFF = "40000000ff0000ffffff6401000100";
const char* const BLASTER_LED_ON = "71ffffff0164006400";
const char* const BLASTER_LED_OFF = "70ffffff0164006400";

// Camera commands (cmdset 0x02); see robot-core media/RobotCameraService.kt.
const std::uint8_t CMD_CAMERA_RECORD = 0x02;
const std::uint8_t CMD_CAMERA_SET_MODE = 0x10;
const std::uint8_t CMD_CAMERA_STATUS_PUSH = 0xE4;
const double CAMERA_STATUS_STALE_SECONDS = 1.5;

void sleepSeconds(double seconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(seconds * 1e6)));
}

template <typename T>
T clamp(T value, T low, T high) {
    return value < low ? low : (value > high ? high : value);
}

// Python round(): ties to even under the default floating-point environment.
long roundHalfEven(double value) { return static_cast<long>(std::nearbyint(value)); }

void appendLe(Bytes& out, std::uint64_t value, int bytes) {
    for (int i = 0; i < bytes; ++i) out.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

float readFloatLe(const Bytes& data, std::size_t offset) {
    std::uint32_t bits = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) |
                         (static_cast<std::uint32_t>(data[offset + 3]) << 24);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

Bytes concat(std::uint8_t head, const Bytes& tail) {
    Bytes out(1, head);
    out.insert(out.end(), tail.begin(), tail.end());
    return out;
}

SetupEntry direct(std::uint8_t receiver, std::uint8_t attr, std::uint8_t cmdset,
                  std::uint8_t cmdid, const Bytes& payload, const Bytes& flags = Bytes()) {
    SetupEntry e;
    e.kind = "direct";
    e.sender = HOST_MOBILE;
    e.receiver = receiver;
    e.attr = attr;
    e.cmdset = cmdset;
    e.cmdid = cmdid;
    e.payload = payload;
    e.flags = flags;
    return e;
}

SetupEntry control() {
    SetupEntry e;
    e.kind = "control";
    e.sender = e.receiver = e.attr = e.cmdset = e.cmdid = 0;
    e.payload = neutralControl();
    return e;
}

Bytes gimbalSubscriptionRemove() {
    Bytes out;
    out.push_back(0);
    out.push_back(2);
    out.push_back(GIMBAL_SUBSCRIPTION_ID);
    return out;
}

Bytes gimbalSubscriptionAdd() {
    Bytes out;
    out.push_back(2);
    out.push_back(GIMBAL_SUBSCRIPTION_ID);
    out.push_back(0);
    out.push_back(0);
    out.push_back(1);
    appendLe(out, GIMBAL_SUBSCRIPTION_UID, 8);
    appendLe(out, GIMBAL_SUBSCRIPTION_HZ, 2);
    return out;
}

// Protocol facts recovered from Windows RoboMaster App traffic. These entries
// continue after APP_CONNECTION_SETUP and deliberately regenerate the active
// session, tick, DUSS sequence and CRC rather than replaying captured packets.
std::vector<SetupEntry> buildDirectModeSetup() {
    std::vector<SetupEntry> s;
    const Bytes noFlags = fromHex("0000");
    Bytes zeros34(34, 0);
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_RM, CMD_RM_SPECIAL_CONTROL,
                       fromHex(MODE_REMOTE), noFlags));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF,
                       fromHex("010500"), noFlags));
    s.push_back(direct(HOST_CHASSIS_CAN, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_WORK_MODE_SET,
                       fromHex("00"), noFlags));
    s.push_back(direct(HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON, CMD_GET_DEVICE_VERSION, Bytes(),
                       fromHex("4000")));
    s.push_back(direct(HOST_CHASSIS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_FC_RMC, fromHex("0200"),
                       fromHex("6040")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_FC_RMC, fromHex("0200"),
                       fromHex("4000")));
    s.push_back(direct(HOST_SCRATCH_SYS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SCRIPT_CTRL,
                       concat(0xA9, zeros34), fromHex("4000")));
    s.push_back(direct(HOST_SCRATCH_SYS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SCRIPT_CTRL,
                       concat(0x09, zeros34), noFlags));
    Bytes hash1(PAIR_HASH_1, PAIR_HASH_1 + std::strlen(PAIR_HASH_1));
    Bytes hash2(PAIR_HASH_2, PAIR_HASH_2 + std::strlen(PAIR_HASH_2));
    s.push_back(direct(HOST_SCRATCH_SYS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SCRIPT_CTRL,
                       concat(0x51, hash1), noFlags));
    s.push_back(direct(HOST_SCRATCH_SYS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SCRIPT_CTRL,
                       concat(0x91, hash2), noFlags));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS, CMD_VBUS_DEL_MSG,
                       gimbalSubscriptionRemove(), noFlags));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS, CMD_VBUS_ADD_MSG,
                       gimbalSubscriptionAdd(), noFlags));
    s.push_back(control());
    s.push_back(direct(HOST_SCRATCH_SYS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SCRIPT_CTRL,
                       concat(0x08, hash1), fromHex("6000")));
    s.push_back(direct(HOST_SCRATCH_SYS, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SCRIPT_CTRL,
                       concat(0xA8, hash2), noFlags));
    s.push_back(control());
    s.push_back(control());
    s.push_back(direct(HOST_WIFI, ATTR_NEED_ACK, CMDSET_WIFI, CMD_WIFI_GET_WORK_MODE, Bytes(),
                       noFlags));
    for (int i = 0; i < 4; ++i) s.push_back(control());
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_RM, CMD_RM_SPECIAL_CONTROL,
                       fromHex(MODE_REMOTE), noFlags));
    return s;
}

std::vector<SetupEntry> buildDirectModeEffect() {
    std::vector<SetupEntry> s;
    const Bytes irGun = fromHex(IR_GUN_CONFIG);
    s.push_back(control());
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF, fromHex("010301")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF, fromHex("010401")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF, fromHex("010201")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_PLAY_SOUND_TASK,
                       fromHex("05049012516a00000000")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF, fromHex("010401")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF, fromHex("010201")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SYSTEM_STATUS_CONFIG,
                       fromHex("01")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_GAME_STATE_SYNC, irGun));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_RM, CMD_RM_SPECIAL_CONTROL,
                       fromHex("010301")));
    s.push_back(direct(HOST_WIFI, ATTR_NEED_ACK, CMDSET_WIFI, CMD_WIFI_AP_KEEPALIVE, Bytes()));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SYSTEM_FUNCTION_CONFIG,
                       fromHex("02")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_GAME_STATE_SYNC, irGun));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_RM, CMD_RM_SPECIAL_CONTROL,
                       fromHex("010301")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_GAMECTRL_CMD,
                       fromHex("0100")));
    s.push_back(direct(HOST_CAMERA, ATTR_NEED_ACK, CMDSET_CAMERA, CMD_SET_ZOOM_PARAM,
                       fromHex("0900006400")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SYSTEM_FUNCTION_CONFIG,
                       fromHex("02")));
    s.push_back(direct(HOST_VISION, ATTR_NEED_ACK, CMDSET_VISION, CMD_VISION_CUSTOM, fromHex("0000")));
    s.push_back(direct(HOST_VISION, ATTR_NEED_ACK, CMDSET_VISION, CMD_VISION_CUSTOM, fromHex("0000")));
    return s;
}

std::vector<SetupEntry> buildDirectModeExit() {
    std::vector<SetupEntry> s;
    s.push_back(control());
    s.push_back(direct(HOST_CAMERA, ATTR_NEED_ACK, CMDSET_CAMERA, CMD_SET_ZOOM_PARAM,
                       fromHex("0900006400")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SYSTEM_FUNCTION_CONFIG,
                       fromHex("00")));
    s.push_back(direct(HOST_VISION, ATTR_NEED_ACK, CMDSET_VISION, CMD_VISION_CUSTOM, fromHex("0000")));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_PLAY_SOUND_TASK,
                       fromHex("06040000000000000000")));
    s.push_back(control());
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_RM, CMD_RM_SPECIAL_CONTROL,
                       fromHex(MODE_NORMAL)));
    s.push_back(direct(HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SAVE_PREF,
                       fromHex(MODE_EXIT_PREF)));
    s.push_back(direct(HOST_VISION, ATTR_NEED_ACK, CMDSET_VISION, CMD_VISION_CUSTOM, fromHex("0000")));
    return s;
}

Bytes ledColorPayload(std::uint32_t component, std::uint8_t mode, int red, int green, int blue,
                      std::uint8_t reserved, std::uint16_t onInterval, std::uint16_t offInterval) {
    Bytes out;
    appendLe(out, component, 4);
    appendLe(out, LED_ARMOR_ALL_MASK, 2);
    out.push_back(mode);
    out.push_back(static_cast<std::uint8_t>(clamp(red, 0, 255)));
    out.push_back(static_cast<std::uint8_t>(clamp(green, 0, 255)));
    out.push_back(static_cast<std::uint8_t>(clamp(blue, 0, 255)));
    out.push_back(reserved);
    appendLe(out, onInterval, 2);
    appendLe(out, offInterval, 2);
    return out;
}

std::string lower(const std::string& value) {
    std::string out(value);
    for (std::size_t i = 0; i < out.size(); ++i) {
        if (out[i] >= 'A' && out[i] <= 'Z') out[i] = static_cast<char>(out[i] - 'A' + 'a');
    }
    return out;
}


const std::vector<SetupEntry>& directModeSetup() {
    static const std::vector<SetupEntry> value = buildDirectModeSetup();
    return value;
}

const std::vector<SetupEntry>& directModeEffect() {
    static const std::vector<SetupEntry> value = buildDirectModeEffect();
    return value;
}

const std::vector<SetupEntry>& directModeExit() {
    static const std::vector<SetupEntry> value = buildDirectModeExit();
    return value;
}
}  // namespace

bool Odometry::rawMotionValues(float& a, float& b, float& c) const {
    if (values.size() < 5) return false;
    a = values[2];
    b = values[3];
    c = values[4];
    return true;
}

bool decodeOdometry(const DussFrame& frame, Odometry& out) {
    if (frame.cmdset != 0x48 || frame.cmdid != 0x08 || frame.payload.size() != 62) return false;
    out.receivedAt = monotonicSeconds();
    out.sequence = frame.sequence;
    out.batteryPercent = frame.payload[10];
    out.headingLike = readFloatLe(frame.payload, 12);
    out.values.clear();
    for (std::size_t offset = 26; offset < 62; offset += 4) {
        out.values.push_back(readFloatLe(frame.payload, offset));
    }
    out.payloadHex = toHex(frame.payload);
    return true;
}

bool decodeGimbal(const DussFrame& frame, GimbalTelemetry& out) {
    if (!frame.valid || frame.cmdset != CMDSET_VIRTUAL_BUS ||
        frame.cmdid != CMD_VBUS_DATA_ANALYSIS || frame.payload.size() != 11 ||
        frame.payload[0] != 0 || frame.payload[1] != GIMBAL_SUBSCRIPTION_ID) {
        return false;
    }
    out.receivedAt = monotonicSeconds();
    out.sequence = frame.sequence;
    for (int i = 0; i < 4; ++i) {
        out.values[i] = static_cast<std::int16_t>(frame.payload[2 + i * 2] |
                                                  (frame.payload[3 + i * 2] << 8));
    }
    out.flag = frame.payload[10];
    out.payloadHex = toHex(frame.payload);
    return true;
}

bool decodeChassisAttitude(const DussFrame& frame, ChassisAttitude& out) {
    if (!frame.valid || frame.cmdset != CMDSET_VIRTUAL_BUS ||
        frame.cmdid != CMD_VBUS_DATA_ANALYSIS || frame.payload.size() != 14 ||
        frame.payload[0] != 0 || frame.payload[1] != CHASSIS_ATTITUDE_SUBSCRIPTION_ID) {
        return false;
    }
    float angles[3];
    for (int i = 0; i < 3; ++i) {
        angles[i] = readFloatLe(frame.payload, 2 + i * 4);
        if (!(angles[i] >= -180.0f && angles[i] <= 180.0f)) return false;
    }
    out.receivedAt = monotonicSeconds();
    out.yawDegrees = angles[0];
    out.pitchDegrees = angles[1];
    out.rollDegrees = angles[2];
    return true;
}

Bytes buildDdsSubscriptionPayload(std::uint8_t messageId, std::uint64_t uid, std::uint16_t hz) {
    Bytes out;
    out.push_back(2);
    out.push_back(messageId);
    out.push_back(0);
    out.push_back(0);
    out.push_back(1);
    appendLe(out, uid, 8);
    appendLe(out, hz, 2);
    return out;
}

Bytes buildChassisControlPayload(double x, double y, double z) {
    x = clamp(x, -1.0, 1.0);
    y = clamp(y, -1.0, 1.0);
    z = clamp(z, -150.0, 150.0) / 150.0;
    long linearX = clamp(roundHalfEven(1024 + 256 * x), 0L, 2047L);
    long linearY = clamp(roundHalfEven(1024 + 256 * y), 0L, 2047L);
    long angular = clamp(roundHalfEven(256 * z), -1024L, 1023L) & 0x0FFF;
    Bytes payload = fromHex("0004200001084000000000000000");
    payload[0] = static_cast<std::uint8_t>(linearY & 0xFF);
    payload[1] = static_cast<std::uint8_t>(((linearX << 3) & 0xF8) | ((linearY >> 8) & 0x07));
    payload[2] = static_cast<std::uint8_t>((payload[2] & 0xC0) | ((linearX >> 5) & 0x3F));
    payload[5] = static_cast<std::uint8_t>(((angular << 4) & 0xF0) | 0x08);
    payload[6] = static_cast<std::uint8_t>((angular >> 4) & 0xFF);
    payload[8] = static_cast<std::uint8_t>(0x02 | ((angular << 2) & 0xFF));
    payload[9] = static_cast<std::uint8_t>((angular >> 6) & 0xFF);
    payload[10] = 0x04;
    payload[11] = 0x0C;
    payload[12] = 0x00;
    payload[13] = 0x04;
    return payload;
}

Bytes buildGimbalSpeedPayload(double pitch, double yaw) {
    pitch = clamp(pitch, -120.0, 120.0) / 120.0;
    yaw = clamp(yaw, -120.0, 120.0) / 120.0;
    long pitchRaw = clamp(roundHalfEven(-1024 * pitch), -1024L, 1023L);
    long yawRaw = clamp(roundHalfEven(-1024 * yaw), -1024L, 1023L);
    Bytes payload = fromHex("0805");
    appendLe(payload, static_cast<std::uint16_t>(static_cast<std::int16_t>(pitchRaw)), 2);
    appendLe(payload, static_cast<std::uint16_t>(static_cast<std::int16_t>(yawRaw)), 2);
    return payload;
}

Bytes buildLedPayload(std::uint32_t component, int red, int green, int blue,
                      const std::string& effect) {
    std::string key = lower(effect);
    std::uint8_t mode;
    if (key == "off") {
        mode = 0;
    } else if (key == "on" || key == "solid") {
        mode = 1;
    } else if (key == "breath") {
        mode = 2;
    } else if (key == "flash") {
        mode = 3;
    } else {
        throw std::invalid_argument("effect must be off, on, solid, breath, or flash");
    }
    std::uint16_t interval = mode == 3 ? 250 : 1000;
    return ledColorPayload(component, static_cast<std::uint8_t>(0x70 | mode), red, green, blue, 0,
                           interval, interval);
}

void Chassis::driveSpeed(double x, double y, double z, double leaseSeconds) {
    robot_.requireArmed();
    robot_.connection.setControlPayload(buildChassisControlPayload(x, y, z), leaseSeconds);
}

void Chassis::stop() { robot_.connection.setControlPayload(neutralControl()); }

RobotAck Chassis::driveWheels(int w1, int w2, int w3, int w4, double timeout) {
    robot_.requireArmed();
    int wheels[4] = {w1, w2, w3, w4};
    Bytes payload;
    for (int i = 0; i < 4; ++i) {
        appendLe(payload, static_cast<std::uint16_t>(static_cast<std::int16_t>(
                              clamp(wheels[i], -1000, 1000))),
                 2);
    }
    return robot_.request(HOST_MOBILE, HOST_CHASSIS_CAN, CMDSET_RM, CMD_RM_WHEEL_SPEED_SET,
                          payload, timeout);
}

void Gimbal::driveSpeed(double pitchSpeed, double yawSpeed, double leaseSeconds) {
    robot_.requireArmed();
    PeriodicDuss command;
    command.sender = HOST_MOBILE;
    command.receiver = HOST_GIMBAL;
    command.attr = ATTR_NO_ACK;
    command.cmdset = CMDSET_GIMBAL;
    command.cmdid = CMD_GIMBAL_ROTATE_SPEED;
    command.payload = buildGimbalSpeedPayload(pitchSpeed, yawSpeed);
    robot_.connection.setPeriodicDuss("gimbal-speed", command, leaseSeconds);
}

void Gimbal::stop() {
    robot_.connection.clearPeriodicDuss("gimbal-speed");
    if (robot_.connection.connected()) {
        robot_.connection.sendDuss(HOST_MOBILE, HOST_GIMBAL, ATTR_NO_ACK, CMDSET_GIMBAL,
                                   CMD_GIMBAL_ROTATE_SPEED, buildGimbalSpeedPayload(0, 0));
    }
}

bool CameraStatus::cardError() const {
    switch (cardState) {
        case 2: case 3: case 4: case 6: case 10: case 11: return true;
        default: return false;
    }
}

bool decodeCameraStatus(const DussFrame& frame, CameraStatus& out) {
    // Camera notifications arrive directly or relayed by the HDVT node.
    if (!frame.valid || (frame.sender != HOST_CAMERA && frame.sender != HOST_HDVT_UAV) ||
        frame.receiver != HOST_MOBILE || frame.attr != ATTR_NO_ACK || frame.cmdset != CMDSET_CAMERA ||
        frame.cmdid != CMD_CAMERA_STATUS_PUSH || frame.payload.size() < 26) {
        return false;
    }
    const Bytes& p = frame.payload;
    unsigned flags = p[5] | (p[6] << 8);
    out.receivedAt = monotonicSeconds();
    out.workMode = flags & 3;
    out.cardInserted = (p[2] & 0x10) != 0;
    out.cardState = p[2] & 0x0F;
    out.recording = (flags & 0x3000) != 0;
    out.shooting = (p[7] & 0x30) != 0;
    out.recordingSeconds = p[3] | (p[4] << 8);
    return true;
}

Robot::Robot(const std::string& robotIp, const std::string& appid, const std::string& localIp,
             bool debug)
    : connection(robotIp, appid, localIp, protocol::LOCAL_CONTROL_PORT, debug),
      camera(connection),
      audio(connection),
      chassis(*this),
      gimbal(*this),
      vision(*this),
      controlMode_(false),
      armed_(false),
      hasOdometry_(false),
      hasGimbal_(false),
      hasAttitude_(false),
      hasCameraStatus_(false) {
    connection.onDuss([this](const DussFrame& frame) { handleDuss(frame); });
}

Robot::~Robot() {
    try {
        close();
    } catch (...) {
    }
}

bool Robot::odometry(Odometry& out) const {
    std::lock_guard<std::mutex> lock(telemetryLock_);
    if (!hasOdometry_) return false;
    out = odometry_;
    return true;
}

bool Robot::gimbalTelemetry(GimbalTelemetry& out) const {
    std::lock_guard<std::mutex> lock(telemetryLock_);
    if (!hasGimbal_) return false;
    out = gimbal_;
    return true;
}

bool Robot::chassisAttitude(ChassisAttitude& out) const {
    std::lock_guard<std::mutex> lock(telemetryLock_);
    if (!hasAttitude_) return false;
    out = attitude_;
    return true;
}

bool Robot::cameraStatus(CameraStatus& out) const {
    std::lock_guard<std::mutex> lock(telemetryLock_);
    if (!hasCameraStatus_) return false;
    out = cameraStatus_;
    return true;
}

bool Robot::initialize(const std::string& connType, const std::string& protoType,
                       double timeoutSeconds) {
    if (connType != "sta" || protoType != "udp") {
        throw std::invalid_argument(
            "the RoboMaster direct backend requires connType 'sta' and protoType 'udp'");
    }
    return connection.initialize(timeoutSeconds);
}

std::vector<std::uint16_t> Robot::enterControlMode() {
    if (!connected()) throw std::runtime_error("App connection is not initialized");
    if (controlMode_) return std::vector<std::uint16_t>();
    connection.setControlPayload(neutralControl());
    connection.configureModeKeepalive(fromHex(MODE_REMOTE), false);
    std::vector<std::uint16_t> sequences = sendSetup(directModeSetup(), true);
    std::vector<std::uint16_t> effects = sendSetup(directModeEffect(), false);
    sequences.insert(sequences.end(), effects.begin(), effects.end());
    controlMode_ = true;
    armed_ = false;
    return sequences;
}

void Robot::exitControlMode() {
    if (!controlMode_) return;
    disarm();
    sendSetup(directModeExit(), false);
    // The exit sequence disables onboard vision.
    vision.forget();
    connection.configureModeKeepalive(fromHex(MODE_NORMAL), true);
    controlMode_ = false;
}

void Robot::arm() {
    if (!controlMode_) throw std::runtime_error("enterControlMode() must complete before arm()");
    stop();
    armed_ = true;
}

void Robot::disarm() {
    armed_ = false;
    stop();
}

void Robot::stop() {
    chassis.stop();
    gimbal.stop();
}

void Robot::close() {
    struct Closer {
        AppConnection& connection;
        ~Closer() { connection.close(); }
    } closer = {connection};
    audio.close();
    camera.close();
    if (connected()) exitControlMode();
}

RobotAck Robot::request(std::uint8_t sender, std::uint8_t receiver, std::uint8_t cmdset,
                        std::uint8_t cmdid, const Bytes& payload, double timeout) {
    std::uint16_t sequence = connection.sendDuss(sender, receiver, ATTR_NEED_ACK, cmdset, cmdid,
                                                 payload);
    DussFrame frame;
    char name[32];
    std::snprintf(name, sizeof(name), "DUSS 0x%02x/0x%02x", cmdset, cmdid);
    if (!connection.waitForDuss(sequence, frame, cmdset, cmdid, 1, timeout)) {
        throw TimeoutError(std::string(name) + " ACK timed out");
    }
    RobotAck ack;
    ack.sequence = frame.sequence;
    ack.sender = frame.sender;
    ack.receiver = frame.receiver;
    ack.cmdset = frame.cmdset;
    ack.cmdid = frame.cmdid;
    ack.payload = frame.payload;
    if (!ack.accepted()) {
        char code[16];
        std::snprintf(code, sizeof(code), "%d", ack.returnCode());
        throw RequestRejected(std::string(name) + " rejected with " + code, ack.returnCode());
    }
    return ack;
}

RobotAck Robot::setLed(const std::string& component, int red, int green, int blue,
                       const std::string& effect, double timeout) {
    std::string key = lower(component);
    std::uint32_t componentId;
    if (key == "all") {
        componentId = LED_SUBCOMPONENT_ALL;
    } else if (key == "top" || key == "gimbal") {
        componentId = LED_SUBCOMPONENT_TOP;
    } else if (key == "bottom") {
        componentId = LED_SUBCOMPONENT_BOTTOM;
    } else {
        throw std::invalid_argument("component must be all, top, gimbal, or bottom");
    }
    return request(HOST_MOBILE, HOST_HDVT_UAV, CMDSET_RM, CMD_RM_LED_COLOR_SET,
                   buildLedPayload(componentId, red, green, blue, effect), timeout);
}

RobotAck Robot::setMuzzleLed(bool fire, bool enabled, double timeout) {
    std::uint8_t mode = fire ? 0 : 7;
    Bytes payload = ledColorPayload(1u << 6, static_cast<std::uint8_t>((mode << 4) | (enabled ? 1 : 0)),
                                    255, 255, 255, 100, 1, 1);
    return request(HOST_MOBILE, HOST_HDVT_UAV, CMDSET_RM, CMD_RM_LED_COLOR_SET, payload, timeout);
}

RobotAck Robot::playSound(std::uint32_t soundId, double timeout) {
    std::uint8_t control = (soundId >= 0x107 && soundId <= 0x12A) ? 2 : 1;
    Bytes payload;
    appendLe(payload, soundId, 4);
    payload.push_back(control);
    appendLe(payload, 5000, 2);
    payload.push_back(1);
    return request(HOST_MOBILE, HOST_HDVT_UAV, CMDSET_RM, CMD_RM_PLAY_SOUND, payload, timeout);
}

RobotAck Robot::setSpeakerVolume(int volume, double timeout) {
    return request(HOST_MOBILE, HOST_CAMERA, CMDSET_RM, CMD_RM_SET_SPEAKER_VOLUME,
                   Bytes(1, static_cast<std::uint8_t>(clamp(volume, 0, 100))), timeout);
}

RobotAck Robot::capture(double timeout) {
    CameraStatus status;
    if (cameraStatus(status) && monotonicSeconds() - status.receivedAt < CAMERA_STATUS_STALE_SECONDS &&
        status.workMode != 0) {
        setCameraMode(0);
    }
    return request(HOST_MOBILE, HOST_CAMERA, CMDSET_CAMERA, CMD_CAPTURE, Bytes(1, 0x01), timeout);
}

CameraStatus Robot::freshCameraStatus() const {
    CameraStatus status;
    if (!cameraStatus(status) || monotonicSeconds() - status.receivedAt >= CAMERA_STATUS_STALE_SECONDS) {
        throw std::runtime_error("no fresh camera status");
    }
    return status;
}

bool Robot::awaitCameraStatus(double after,
                              const std::function<bool(const CameraStatus&)>& matches) {
    // A new status must confirm the change within the status freshness window.
    std::unique_lock<std::mutex> lock(telemetryLock_);
    return telemetryCondition_.wait_for(
        lock, std::chrono::duration<double>(CAMERA_STATUS_STALE_SECONDS), [&] {
            return hasCameraStatus_ && cameraStatus_.receivedAt > after && matches(cameraStatus_);
        });
}

void Robot::setCameraMode(int mode) {
    double before = monotonicSeconds();
    request(HOST_MOBILE, HOST_CAMERA, CMDSET_CAMERA, CMD_CAMERA_SET_MODE,
            Bytes(1, static_cast<std::uint8_t>(mode)));
    if (!awaitCameraStatus(before, [mode](const CameraStatus& s) { return s.workMode == mode; })) {
        throw std::runtime_error("camera did not switch mode");
    }
}

void Robot::setRecording(bool active) {
    if (active) {
        CameraStatus status = freshCameraStatus();
        if (!status.canCapture()) throw std::runtime_error("SD card cannot record");
        if (status.workMode != 1) {
            setCameraMode(1);
            if (!freshCameraStatus().canCapture()) throw std::runtime_error("SD card cannot record");
        }
    }
    double before = monotonicSeconds();
    request(HOST_MOBILE, HOST_CAMERA, CMDSET_CAMERA, CMD_CAMERA_RECORD,
            Bytes(1, active ? 1 : 0));
    if (!awaitCameraStatus(before, [active](const CameraStatus& s) { return s.recording == active; })) {
        throw std::runtime_error(active ? "camera did not start recording"
                                        : "camera did not stop recording");
    }
}

void Robot::fireInfrared(double leaseSeconds) {
    requireArmed();
    connection.setControlPayload(fromHex(TRIGGER_CONTROL), leaseSeconds);
}

void Robot::fireGel() {
    requireArmed();
    if (!controlMode_) throw std::runtime_error("gel firing requires control mode");
    connection.sendDuss(HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_LED_COLOR_SET,
                        fromHex(MUZZLE_FIRE_LED_ON));
    connection.sendDuss(HOST_MOBILE, HOST_GUN, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_GUN_LED_SET,
                        fromHex(BLASTER_LED_ON));
    connection.sendDuss(HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_SHOOT_CMD,
                        Bytes(1, 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    if (!connected()) return;
    connection.sendDuss(HOST_MOBILE, HOST_GUN, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_GUN_LED_SET,
                        fromHex(BLASTER_LED_OFF));
    connection.sendDuss(HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_RM, CMD_RM_LED_COLOR_SET,
                        fromHex(MUZZLE_FIRE_LED_OFF));
}

void Robot::subscribeChassisAttitude() {
    if (!connected()) throw std::runtime_error("App connection is not initialized");
    Bytes remove;
    remove.push_back(0);
    remove.push_back(2);
    remove.push_back(CHASSIS_ATTITUDE_SUBSCRIPTION_ID);
    connection.sendDuss(HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS,
                        CMD_VBUS_DEL_MSG, remove);
    connection.sendDuss(HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS,
                        CMD_VBUS_ADD_MSG,
                        buildDdsSubscriptionPayload(CHASSIS_ATTITUDE_SUBSCRIPTION_ID,
                                                    CHASSIS_ATTITUDE_SUBSCRIPTION_UID,
                                                    CHASSIS_ATTITUDE_SUBSCRIPTION_HZ));
}

bool Robot::waitForChassisAttitude(ChassisAttitude& out, double after, double timeoutSeconds) {
    std::unique_lock<std::mutex> lock(telemetryLock_);
    bool ready = telemetryCondition_.wait_for(
        lock, std::chrono::duration<double>(timeoutSeconds > 0 ? timeoutSeconds : 0),
        [this, after] { return hasAttitude_ && attitude_.receivedAt > after; });
    if (ready) out = attitude_;
    return ready;
}

bool Robot::waitForCameraStatus(CameraStatus& out, double after, double timeoutSeconds) {
    std::unique_lock<std::mutex> lock(telemetryLock_);
    bool ready = telemetryCondition_.wait_for(
        lock, std::chrono::duration<double>(timeoutSeconds > 0 ? timeoutSeconds : 0),
        [this, after] { return hasCameraStatus_ && cameraStatus_.receivedAt > after; });
    if (ready) out = cameraStatus_;
    return ready;
}

bool Robot::waitForOdometry(Odometry& out, double after, double timeoutSeconds) {
    std::unique_lock<std::mutex> lock(telemetryLock_);
    bool ready = telemetryCondition_.wait_for(
        lock, std::chrono::duration<double>(timeoutSeconds > 0 ? timeoutSeconds : 0),
        [this, after] { return hasOdometry_ && odometry_.receivedAt > after; });
    if (ready) out = odometry_;
    return ready;
}

bool Robot::waitForGimbal(GimbalTelemetry& out, double after, double timeoutSeconds) {
    std::unique_lock<std::mutex> lock(telemetryLock_);
    bool ready = telemetryCondition_.wait_for(
        lock, std::chrono::duration<double>(timeoutSeconds > 0 ? timeoutSeconds : 0),
        [this, after] { return hasGimbal_ && gimbal_.receivedAt > after; });
    if (ready) out = gimbal_;
    return ready;
}

void Robot::requireArmed() const {
    if (!armed_) throw std::runtime_error("direct mechanical control requires arm()");
}

std::vector<std::uint16_t> Robot::sendSetup(const std::vector<SetupEntry>& entries,
                                            bool includeFlags) {
    std::vector<std::uint16_t> sequences;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const SetupEntry& e = entries[i];
        if (e.kind == "control") {
            sequences.push_back(connection.sendControl(e.payload));
            sleepSeconds(0.02);
            continue;
        }
        sequences.push_back(connection.sendDuss(e.sender, e.receiver, e.attr, e.cmdset, e.cmdid,
                                                e.payload, includeFlags ? e.flags : Bytes()));
        sleepSeconds(0.006);
    }
    return sequences;
}

void Robot::handleDuss(const DussFrame& frame) {
    Odometry odometry;
    GimbalTelemetry gimbal;
    ChassisAttitude attitude;
    CameraStatus camera;
    if (decodeOdometry(frame, odometry)) {
        std::lock_guard<std::mutex> lock(telemetryLock_);
        odometry_ = odometry;
        hasOdometry_ = true;
        telemetryCondition_.notify_all();
    } else if (decodeGimbal(frame, gimbal)) {
        std::lock_guard<std::mutex> lock(telemetryLock_);
        gimbal_ = gimbal;
        hasGimbal_ = true;
        telemetryCondition_.notify_all();
    } else if (decodeChassisAttitude(frame, attitude)) {
        std::lock_guard<std::mutex> lock(telemetryLock_);
        attitude_ = attitude;
        hasAttitude_ = true;
        telemetryCondition_.notify_all();
    } else if (decodeCameraStatus(frame, camera)) {
        std::lock_guard<std::mutex> lock(telemetryLock_);
        cameraStatus_ = camera;
        hasCameraStatus_ = true;
        telemetryCondition_.notify_all();
    }
}

}  // namespace hanppie
