// Onboard vision detection over the App protocol session.
// C++11 port of robot-core vision/VisionProtocol.kt and VisionControl.kt: the
// robot runs the detector and pushes normalized results to the App node.
//
// Robot aiming (mask 8) is not exposed: on the robot it takes over the gimbal,
// which needs the App's gimbal ownership handling.
#ifndef HANPPIE_VISION_HPP
#define HANPPIE_VISION_HPP

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "hanppie/protocol.hpp"

namespace hanppie {

class Robot;

enum class VisionMode { HeadShoulders, People, Pose, Lines, Markers };
// App/SDK colour values; only line and marker detection take a colour.
enum class VisionColor { Red = 1, Green = 2, Blue = 3 };

// Enable-mask bit and push report type of a detector.
std::uint16_t visionModeMask(VisionMode mode);
int visionReportType(VisionMode mode);

// Center-based normalized video geometry; info depends on the detector.
struct VisionRect {
    float x, y, width, height;
    std::uint32_t info;

    // Marker reports: low word ID, high word distance labelled in centimetres.
    int markerId() const { return static_cast<int>(info & 0xFFFF); }
    int markerDistanceCentimetres() const { return static_cast<int>(info >> 16); }
    int poseId() const { return static_cast<int>(info & 0xFFFF); }
};

// Line reports carry tangent and curvature in the rectangle width/height slots.
struct VisionLinePoint {
    float x, y, tangent, curvature;
    std::uint32_t info;
};

struct VisionReport {
    double receivedAt;
    int type;
    int status;  // 0 exist, 1 initializing, 2 waiting confirm, 3 running
    int error;
    std::vector<VisionRect> rectangles;
    std::vector<VisionLinePoint> linePoints;
};

// Decode a vision push (0a:a4 from the vision node); false if not one or malformed.
bool decodeVisionPush(const protocol::DussFrame& frame, VisionReport& out);
Bytes buildVisionEnablePayload(std::uint16_t mask);
Bytes buildVisionColorPayload(VisionMode mode, VisionColor color);

// Raised when the vision pipeline cannot be started or confirmed stopped.
class VisionError : public std::runtime_error {
public:
    explicit VisionError(const std::string& what) : std::runtime_error(what) {}
};

// One detector at a time, as in the App. start() and stop() verify the enable
// mask read back from the robot; reports older than 0.75 s are treated as gone.
class Vision {
public:
    explicit Vision(Robot& robot);
    ~Vision();

    // Requires control mode. Fails if another detector is already running on
    // the robot, or if the robot does not confirm the requested mask.
    void start(VisionMode mode);
    void start(VisionMode mode, VisionColor color);
    // Disable and confirm the mask is 0, retrying up to three times.
    void stop();

    bool active() const;
    VisionMode mode() const;
    // Latest report of the active detector, if newer than maxAgeSeconds.
    bool latest(VisionReport& out, double maxAgeSeconds = 0.75) const;
    // Wait for a report of the active detector received after `after`.
    bool waitForReport(VisionReport& out, double after = 0.0, double timeoutSeconds = 2.0);

private:
    Vision(const Vision&);
    Vision& operator=(const Vision&);
    friend class Robot;

    void begin(VisionMode mode, bool hasColor, VisionColor color);
    std::uint16_t readMask();
    void handle(const protocol::DussFrame& frame);
    // Control mode exit already disables vision on the robot.
    void forget();

    Robot& robot_;
    int listener_;
    mutable std::mutex lock_;
    std::condition_variable condition_;
    bool active_;
    VisionMode mode_;
    bool hasReport_;
    VisionReport report_;
};

}  // namespace hanppie

#endif  // HANPPIE_VISION_HPP
