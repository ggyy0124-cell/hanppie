#include "hanppie/vision.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include "hanppie/robot.hpp"

namespace hanppie {

namespace {

using namespace protocol;

// Vision command IDs (cmdset 0x0a); see robot-core vision/VisionProtocol.kt.
const std::uint8_t CMD_VISION_ENABLE = 0xA3;
const std::uint8_t CMD_VISION_PUSH = 0xA4;
const std::uint8_t CMD_VISION_GET_ENABLE = 0xA5;
const std::uint8_t CMD_VISION_SET_COLOR = 0xAB;
// The colour attribute is addressed to the HDVT node, not the vision node.
const std::uint8_t VISION_COLOR_RECEIVER = HOST_HDVT_UAV;

const double REQUEST_TIMEOUT = 1.5;
const double STOP_RETRY_TIMEOUT = 0.5;
const int STOP_ATTEMPTS = 3;

std::uint32_t readU32(const Bytes& p, std::size_t at) {
    return p[at] | (p[at + 1] << 8) | (p[at + 2] << 16) | (static_cast<std::uint32_t>(p[at + 3]) << 24);
}

float readFloat(const Bytes& p, std::size_t at) {
    std::uint32_t bits = readU32(p, at);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool unit(float v) { return std::isfinite(v) && v >= 0.0f && v <= 1.0f; }

}  // namespace

std::uint16_t visionModeMask(VisionMode mode) {
    switch (mode) {
        case VisionMode::HeadShoulders: return 0x01;
        case VisionMode::People: return 0x02;
        case VisionMode::Pose: return 0x04;
        case VisionMode::Lines: return 0x10;
        case VisionMode::Markers: return 0x20;
    }
    throw std::invalid_argument("unknown vision mode");
}

int visionReportType(VisionMode mode) {
    switch (mode) {
        case VisionMode::HeadShoulders: return 0;
        case VisionMode::People: return 1;
        case VisionMode::Pose: return 2;
        case VisionMode::Lines: return 4;
        case VisionMode::Markers: return 5;
    }
    throw std::invalid_argument("unknown vision mode");
}

bool decodeVisionPush(const DussFrame& frame, VisionReport& out) {
    if (!frame.valid || frame.sender != HOST_VISION || frame.receiver != HOST_MOBILE ||
        frame.attr != ATTR_NO_ACK || frame.cmdset != CMDSET_VISION || frame.cmdid != CMD_VISION_PUSH) {
        return false;
    }
    const Bytes& p = frame.payload;
    if (p.size() < 9 || p[0] > 6 || p.size() != 9 + p[8] * 20u) return false;
    VisionReport report;
    report.receivedAt = monotonicSeconds();
    report.type = p[0];
    report.status = p[1];
    report.error = p[6] | (p[7] << 8);
    for (std::size_t i = 0; i < p[8]; ++i) {
        std::size_t at = 9 + i * 20;
        float x = readFloat(p, at), y = readFloat(p, at + 4);
        float w = readFloat(p, at + 8), h = readFloat(p, at + 12);
        std::uint32_t info = readU32(p, at + 16);
        if (report.type == 4) {
            if (!unit(x) || !unit(y) || !std::isfinite(w) || !std::isfinite(h) || h < 0.0f ||
                h > 10.0f) {
                return false;
            }
            VisionLinePoint point = {x, y, w, h, info};
            report.linePoints.push_back(point);
            continue;
        }
        if (!unit(x) || !unit(y) || !unit(w) || !unit(h)) return false;
        if (w <= 0.0f || h <= 0.0f) continue;
        VisionRect rect = {x, y, w, h, info};
        report.rectangles.push_back(rect);
    }
    out = report;
    return true;
}

Bytes buildVisionEnablePayload(std::uint16_t mask) {
    Bytes out;
    out.push_back(static_cast<std::uint8_t>(mask & 0xFF));
    out.push_back(static_cast<std::uint8_t>(mask >> 8));
    return out;
}

Bytes buildVisionColorPayload(VisionMode mode, VisionColor color) {
    Bytes out;
    if (mode == VisionMode::Lines) {
        out.push_back(1);
    } else if (mode == VisionMode::Markers) {
        out.push_back(2);
    } else {
        throw std::invalid_argument("only line and marker detection take a colour");
    }
    out.push_back(static_cast<std::uint8_t>(color));
    return out;
}

Vision::Vision(Robot& robot)
    : robot_(robot), active_(false), mode_(VisionMode::Markers), hasReport_(false) {
    listener_ = robot_.connection.onDuss([this](const DussFrame& frame) { handle(frame); });
}

Vision::~Vision() { robot_.connection.removeListener(listener_); }

void Vision::start(VisionMode mode) { begin(mode, false, VisionColor::Red); }

void Vision::start(VisionMode mode, VisionColor color) { begin(mode, true, color); }

void Vision::begin(VisionMode mode, bool hasColor, VisionColor color) {
    if (!robot_.controlMode()) throw std::runtime_error("vision requires control mode");
    if (active()) throw std::runtime_error("stop the current vision mode first");
    Bytes colorPayload = hasColor ? buildVisionColorPayload(mode, color) : Bytes();
    std::uint16_t mask = visionModeMask(mode);
    if (readMask() != 0) throw VisionError("another vision mode is running on the robot");
    robot_.request(HOST_MOBILE, HOST_VISION, CMDSET_VISION, CMD_VISION_ENABLE,
                   buildVisionEnablePayload(0), REQUEST_TIMEOUT);
    if (hasColor) {
        robot_.request(HOST_MOBILE, VISION_COLOR_RECEIVER, CMDSET_VISION, CMD_VISION_SET_COLOR,
                       colorPayload, REQUEST_TIMEOUT);
    }
    {
        // Mark before transmission: a lost reply cannot prove the detector stayed off.
        std::lock_guard<std::mutex> lock(lock_);
        active_ = true;
        mode_ = mode;
        hasReport_ = false;
    }
    try {
        robot_.request(HOST_MOBILE, HOST_VISION, CMDSET_VISION, CMD_VISION_ENABLE,
                       buildVisionEnablePayload(mask), REQUEST_TIMEOUT);
        if (readMask() != mask) throw VisionError("robot did not confirm the vision mode");
    } catch (...) {
        try {
            stop();
        } catch (...) {
        }
        throw;
    }
}

std::uint16_t Vision::readMask() {
    RobotAck ack = robot_.request(HOST_MOBILE, HOST_VISION, CMDSET_VISION, CMD_VISION_GET_ENABLE,
                                  Bytes(), REQUEST_TIMEOUT);
    if (ack.payload.size() != 3) throw VisionError("unsupported vision enable report");
    return static_cast<std::uint16_t>(ack.payload[1] | (ack.payload[2] << 8));
}

void Vision::stop() {
    if (!active()) return;
    for (int attempt = 0; attempt < STOP_ATTEMPTS; ++attempt) {
        try {
            robot_.request(HOST_MOBILE, HOST_VISION, CMDSET_VISION, CMD_VISION_ENABLE,
                           buildVisionEnablePayload(0), STOP_RETRY_TIMEOUT);
            if (readMask() == 0) {
                forget();
                return;
            }
        } catch (const std::exception&) {
            if (!robot_.connected()) break;
        }
    }
    throw VisionError("robot did not confirm that vision stopped");
}

bool Vision::active() const {
    std::lock_guard<std::mutex> lock(lock_);
    return active_;
}

VisionMode Vision::mode() const {
    std::lock_guard<std::mutex> lock(lock_);
    return mode_;
}

bool Vision::latest(VisionReport& out, double maxAgeSeconds) const {
    std::lock_guard<std::mutex> lock(lock_);
    if (!active_ || !hasReport_ || monotonicSeconds() - report_.receivedAt > maxAgeSeconds) {
        return false;
    }
    out = report_;
    return true;
}

bool Vision::waitForReport(VisionReport& out, double after, double timeoutSeconds) {
    std::unique_lock<std::mutex> lock(lock_);
    bool ready = condition_.wait_for(
        lock, std::chrono::duration<double>(timeoutSeconds > 0 ? timeoutSeconds : 0),
        [this, after] { return !active_ || (hasReport_ && report_.receivedAt > after); });
    if (!ready || !active_) return false;
    out = report_;
    return true;
}

void Vision::handle(const DussFrame& frame) {
    VisionReport report;
    if (!decodeVisionPush(frame, report)) return;
    std::lock_guard<std::mutex> lock(lock_);
    if (!active_ || report.type != visionReportType(mode_)) return;
    report_ = report;
    hasReport_ = true;
    condition_.notify_all();
}

void Vision::forget() {
    std::lock_guard<std::mutex> lock(lock_);
    active_ = false;
    hasReport_ = false;
    condition_.notify_all();
}

}  // namespace hanppie
