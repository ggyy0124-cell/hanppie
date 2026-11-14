// Loopback integration tests: a scripted fake S1 on 127.0.0.1 exercises the
// AppID claim, outer session, setup replay, request/ACK matching, command
// leases, telemetry, media dispatch and Lab FTP upload without hardware.
// Requires the fixed App ports (45678, 56789, 10607, 10609) to be free.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "golden.hpp"
#include "hanppie/hanppie.hpp"
#include "socket_compat.hpp"
#include "test_support.hpp"

#ifndef _WIN32
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

using namespace hanppie;
using namespace hanppie::protocol;
namespace sys = hanppie::net::sys;

namespace {

const char* const APPID = "b6359877";

struct Captured {
    std::uint8_t type;  // outer envelope type: 5 direct, 4 control
    Bytes flags;        // direct envelope flags (bytes 18..20)
    DussFrame frame;
};

sockaddr_in loopback(std::uint16_t port) {
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = sys::networkPort(port);
    sys::parseIpv4("127.0.0.1", address.sin_addr);
    return address;
}

sys::Handle udpSocket(std::uint16_t port) {
    sys::startup();
    sys::Handle fd = sys::openSocket(SOCK_DGRAM);
    int one = 1;
    sys::setOption(fd, SOL_SOCKET, SO_REUSEADDR, one);
    sys::ignoreUdpConnectionReset(fd);
    sockaddr_in address = loopback(port);
    if (sys::bindAddress(fd, address) != 0) {
        sys::closeHandle(fd);
        throw std::runtime_error("fake robot could not bind a required port");
    }
    return fd;
}

void sendTo(sys::Handle fd, const Bytes& data, const sockaddr_in& address) {
    sys::sendDatagram(fd, data.data(), data.size(), address);
}

// Minimal robot side of the App protocol.
class FakeRobot {
public:
    FakeRobot()
        : appFd_(udpSocket(ROBOT_APP_PORT)),
          controlFd_(udpSocket(ROBOT_CONTROL_PORT)),
          stop_(false),
          sequence_(1),
          heartbeat_(true),
          hasPeer_(false) {
        thread_ = std::thread(&FakeRobot::run, this);
    }

    ~FakeRobot() {
        stop_ = true;
        thread_.join();
        sys::closeHandle(appFd_);
        sys::closeHandle(controlFd_);
    }

    // Reply to requests for (cmdset, cmdid) with the given ACK payload.
    void ack(std::uint8_t cmdset, std::uint8_t cmdid, const Bytes& payload) {
        std::lock_guard<std::mutex> lock(lock_);
        acks_[std::make_pair(cmdset, cmdid)] = payload;
    }

    // Stateful ACKs: return true and fill payload to answer a request.
    typedef std::function<bool(const DussFrame&, Bytes&)> Responder;
    void respond(const Responder& responder) {
        std::lock_guard<std::mutex> lock(lock_);
        responder_ = responder;
    }

    // Index of the next captured frame, after letting in-flight packets land.
    std::size_t mark() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::lock_guard<std::mutex> lock(lock_);
        return captured_.size();
    }

    std::vector<Captured> since(std::size_t index) {
        std::lock_guard<std::mutex> lock(lock_);
        return std::vector<Captured>(captured_.begin() + index, captured_.end());
    }

    // Send a DUSS frame inside a 20-byte direct header for the active session.
    void push(std::uint8_t attr, std::uint8_t cmdset, std::uint8_t cmdid, const Bytes& payload) {
        pushFrom(HOST_HDVT_UAV, attr, cmdset, cmdid, payload);
    }

    void pushFrom(std::uint8_t sender, std::uint8_t attr, std::uint8_t cmdset, std::uint8_t cmdid,
                  const Bytes& payload) {
        std::lock_guard<std::mutex> lock(lock_);
        // Simulators may start before discovery completes. Telemetry has no
        // recipient until the direct session handshake has supplied both IDs.
        if (!hasPeer_ || session_.size() != 2) return;
        Bytes duss = buildDuss(sender, HOST_MOBILE, attr, cmdset, cmdid, payload, sequence_++);
        Bytes packet(20, 0);
        packet[2] = session_[0];
        packet[3] = session_[1];
        packet[6] = 0x09;
        packet.insert(packet.end(), duss.begin(), duss.end());
        sendTo(controlFd_, packet, peer_);
    }

    // Like the real robot's telemetry pushes, keep the link fresh with a Wi-Fi RSSI
    // push every 100 ms once a session exists; turn it off to simulate a silent robot.
    void setHeartbeat(bool enabled) { heartbeat_ = enabled; }

    void pushVideo(const Bytes& chunk) {
        Bytes packet(20, 0);
        std::lock_guard<std::mutex> lock(lock_);
        packet[2] = session_[0];
        packet[3] = session_[1];
        packet[6] = 0x02;
        packet.insert(packet.end(), chunk.begin(), chunk.end());
        sendTo(controlFd_, packet, peer_);
    }

private:
    void run() {
        Bytes buffer(65535);
        std::chrono::steady_clock::time_point nextBeat = std::chrono::steady_clock::now();
        while (!stop_) {
            if (heartbeat_ && std::chrono::steady_clock::now() >= nextBeat) {
                nextBeat = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
                bool ready;
                {
                    std::lock_guard<std::mutex> lock(lock_);
                    ready = hasPeer_ && session_.size() == 2;
                }
                if (ready) pushFrom(HOST_WIFI, ATTR_NO_ACK, CMDSET_WIFI, CMD_WIFI_AP_PUSH_RSSI, Bytes(1, 0xC4));
            }
            pollfd items[2];
            items[0].fd = appFd_;
            items[1].fd = controlFd_;
            for (int i = 0; i < 2; ++i) {
                items[i].events = POLLIN;
                items[i].revents = 0;
            }
            if (sys::pollHandles(items, 2, 20) <= 0) continue;
            sockaddr_in from;
            if (items[0].revents & POLLIN) {
                sys::IoSize size = sys::receiveDatagram(appFd_, &buffer[0], buffer.size(), from);
                if (size == 8) answerClaim(Bytes(buffer.begin(), buffer.begin() + size));
            }
            if (items[1].revents & POLLIN) {
                sys::IoSize size = sys::receiveDatagram(controlFd_, &buffer[0], buffer.size(), from);
                if (size > 0) handle(Bytes(buffer.begin(), buffer.begin() + size), from);
            }
        }
    }

    void answerClaim(const Bytes& claim) {
        Bytes decoded(24, 0);
        decoded[0] = 0x5A;
        decoded[1] = 0x5B;
        decoded[6] = 127;
        decoded[9] = 1;
        std::copy(claim.begin(), claim.end(), decoded.begin() + 16);
        sendTo(appFd_, test::encodeBroadcast(decoded), loopback(APP_PORT));
    }

    void handle(const Bytes& packet, const sockaddr_in& from) {
        if (packet.size() < 8) return;
        std::vector<std::pair<std::uint8_t, Bytes> > replies;
        {
            std::lock_guard<std::mutex> lock(lock_);
            peer_ = from;
            hasPeer_ = true;
            session_ = Bytes(packet.begin() + 2, packet.begin() + 4);
            if (packet.size() == 0x30 && packet[0] == 0x30 && packet[6] == 0x00) {
                // Preconnect: acknowledge the session.
                Bytes reply(12, 0);
                reply[2] = session_[0];
                reply[3] = session_[1];
                sendTo(controlFd_, reply, from);
                return;
            }
            std::vector<DussFrame> frames = parseDussFrames(packet);
            for (std::size_t i = 0; i < frames.size(); ++i) {
                if (!frames[i].valid) continue;
                Captured item;
                item.type = packet[6];
                if (packet[6] == 0x05) item.flags = Bytes(packet.begin() + 18, packet.begin() + 20);
                item.frame = frames[i];
                captured_.push_back(item);
                std::map<std::pair<std::uint8_t, std::uint8_t>, Bytes>::const_iterator found =
                    acks_.find(std::make_pair(frames[i].cmdset, frames[i].cmdid));
                Bytes answer;
                bool answered = (frames[i].attr & 0x40) && responder_ && responder_(frames[i], answer);
                if (!answered && (frames[i].attr & 0x40) && found != acks_.end()) {
                    answer = found->second;
                    answered = true;
                }
                if (answered) {
                    Bytes duss = buildDuss(frames[i].receiver, frames[i].sender, ATTR_ACK,
                                           frames[i].cmdset, frames[i].cmdid, answer,
                                           frames[i].sequence);
                    Bytes reply(20, 0);
                    reply[2] = session_[0];
                    reply[3] = session_[1];
                    reply[6] = 0x09;
                    reply.insert(reply.end(), duss.begin(), duss.end());
                    replies.push_back(std::make_pair(0, reply));
                }
            }
        }
        for (std::size_t i = 0; i < replies.size(); ++i) sendTo(controlFd_, replies[i].second, from);
    }

    sys::Handle appFd_;
    sys::Handle controlFd_;
    std::atomic<bool> stop_;
    std::uint16_t sequence_;
    std::atomic<bool> heartbeat_;
    std::thread thread_;
    std::mutex lock_;
    bool hasPeer_;
    sockaddr_in peer_;
    Bytes session_;
    std::vector<Captured> captured_;
    std::map<std::pair<std::uint8_t, std::uint8_t>, Bytes> acks_;
    Responder responder_;
};

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// While in control mode the SDK sends one command to each actuator on every 50 Hz
// tick: 3f:20 (wheel speed) or 3f:21 (body velocity) to the chassis, 04:0c to the
// gimbal. They interleave with every other frame, so sequence checks skip them.
bool isChassisStop(const DussFrame& f) {
    return f.receiver == HOST_CHASSIS && f.cmdset == CMDSET_RM && f.cmdid == CMD_RM_WHEEL_SPEED_SET;
}

bool isChassisSpeed(const DussFrame& f) {
    return f.receiver == HOST_CHASSIS && f.cmdset == CMDSET_RM && f.cmdid == CMD_RM_SPEED_SET;
}

bool isGimbalSpeed(const DussFrame& f) {
    return f.receiver == HOST_GIMBAL && f.cmdset == CMDSET_GIMBAL &&
           f.cmdid == CMD_GIMBAL_EXT_CTRL_ACCEL;
}

bool isMotionCommand(const DussFrame& f) {
    return isChassisStop(f) || isChassisSpeed(f) || isGimbalSpeed(f);
}

// Direct frames other than the periodic motion commands.
std::vector<Captured> modeFrames(const std::vector<Captured>& items) {
    std::vector<Captured> out;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x05 && !isMotionCommand(items[i].frame)) out.push_back(items[i]);
    }
    return out;
}

// Compare the direct frames of `actual` with the direct rows of a golden sequence and
// return the direct frames that follow it. The Python reference leaves the transport
// flags of the effect and exit tables at their default {attr, 0}; robot-core sends
// every such row with 0000, and so does the port: golden rows from `appFlagsFrom` on
// are expected with 0000 instead.
std::vector<Captured> checkDirectSequence(const std::vector<Captured>& actual,
                                          const golden::GoldenEntry* expected, std::size_t count,
                                          std::size_t appFlagsFrom = static_cast<std::size_t>(-1)) {
    std::vector<Captured> direct = modeFrames(actual);
    std::size_t expectedDirect = 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (std::string(expected[i].kind) != "direct") continue;
        CHECK(expectedDirect < direct.size());
        const Captured& got = direct[expectedDirect++];
        CHECK_EQ(got.frame.sender, expected[i].sender);
        CHECK_EQ(got.frame.receiver, expected[i].receiver);
        CHECK_EQ(got.frame.attr, expected[i].attr);
        CHECK_EQ(got.frame.cmdset, expected[i].cmdset);
        CHECK_EQ(got.frame.cmdid, expected[i].cmdid);
        CHECK_EQ(toHex(got.frame.payload), std::string(expected[i].payload));
        CHECK_EQ(toHex(got.flags), i >= appFlagsFrom ? std::string("0000")
                                                      : std::string(expected[i].flags));
    }
    return std::vector<Captured>(direct.begin() + expectedDirect, direct.end());
}

// A chassis work-mode command as robot-core AppSession sends it: to the chassis,
// needing an ACK, with the default flags.
void checkChassisModeFrame(const Captured& got, std::uint8_t cmdid, std::uint8_t value) {
    CHECK_EQ(got.type, 0x05);
    CHECK_EQ(got.frame.sender, HOST_MOBILE);
    CHECK_EQ(got.frame.receiver, HOST_CHASSIS);
    CHECK_EQ(got.frame.attr, ATTR_NEED_ACK);
    CHECK_EQ(got.frame.cmdset, CMDSET_RM);
    CHECK_EQ(got.frame.cmdid, cmdid);
    CHECK_EQ(toHex(got.frame.payload), toHex(Bytes(1, value)));
    CHECK_EQ(toHex(got.flags), std::string("4000"));
}

bool sawControlPayload(const std::vector<Captured>& items, const Bytes& payload) {
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x04 && items[i].frame.payload == payload) return true;
    }
    return false;
}

// Whether every control frame in `items` carries `payload`.
bool onlyControlPayload(const std::vector<Captured>& items, const Bytes& payload) {
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x04 && items[i].frame.payload != payload) return false;
    }
    return true;
}

// Number of captured motion frames of one command whose payload equals `payload`.
int countMotion(const std::vector<Captured>& items, bool (*kind)(const DussFrame&),
                const Bytes& payload) {
    int count = 0;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x05 && kind(items[i].frame) && items[i].frame.payload == payload) {
            ++count;
        }
    }
    return count;
}

int countMotion(const std::vector<Captured>& items, bool (*kind)(const DussFrame&)) {
    int count = 0;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x05 && kind(items[i].frame)) ++count;
    }
    return count;
}

int countMotion(const std::vector<Captured>& items) {
    int count = 0;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x05 && isMotionCommand(items[i].frame)) ++count;
    }
    return count;
}

// What the App holds while idle: zero wheel RPM for the chassis (3f:20), zero
// velocity for the gimbal (04:0c).
const Bytes& idleChassisPayload() {
    static const Bytes payload(8, 0);
    return payload;
}

const Bytes& idleGimbalPayload() {
    static const Bytes payload = fromHex("000000000000dc");
    return payload;
}

// Every motion command in `items` is the idle hold, in the form the App sends it.
bool onlyIdleHold(const std::vector<Captured>& items) {
    for (std::size_t i = 0; i < items.size(); ++i) {
        const DussFrame& f = items[i].frame;
        if (items[i].type != 0x05 || !isMotionCommand(f)) continue;
        if (isChassisStop(f) &&
            (f.attr != ATTR_NEED_ACK || f.payload != idleChassisPayload() ||
             items[i].flags != fromHex("4000"))) {
            return false;
        }
        if (isGimbalSpeed(f) &&
            (f.attr != ATTR_NO_ACK || f.payload != idleGimbalPayload() ||
             items[i].flags != fromHex("0000"))) {
            return false;
        }
        if (isChassisSpeed(f)) return false;
    }
    return true;
}

// Whether `items` hold only idle motion commands from the first mode frame (a direct
// frame that is not a motion command) on; the frames before it may still carry the
// speed that was set before the call under test.
bool idleFromFirstModeFrame(const std::vector<Captured>& items) {
    std::size_t start = 0;
    while (start < items.size() &&
           !(items[start].type == 0x05 && !isMotionCommand(items[start].frame))) {
        ++start;
    }
    return start < items.size() &&
           onlyIdleHold(std::vector<Captured>(items.begin() + start, items.end()));
}

}  // namespace

namespace {

// Rows of the App's remoteSetup table; the rows after it are the remoteEffects table.
const std::size_t ENTER_SETUP_ROWS = 23;

}  // namespace

TEST(loopback_session_matches_the_app_flow) {
    FakeRobot fake;
    fake.ack(CMDSET_RM, CMD_RM_LED_COLOR_SET, Bytes(1, 0));
    fake.ack(CMDSET_RM, CMD_RM_SET_SPEAKER_VOLUME, Bytes(1, 1));

    Robot robot("127.0.0.1", APPID);
    robot.connection.suspendIdleKeepalive = true;
    double started = monotonicSeconds();
    CHECK(robot.initialize("sta", "udp", 5.0));
    // The outer session must be acknowledged rather than waiting out its 5 s budget.
    CHECK(monotonicSeconds() - started < 2.0);
    CHECK(robot.connected());
    CHECK_EQ(robot.info().state, std::string("idle"));
    sleepMs(100);
    std::vector<Captured> setup = fake.since(0);
    CHECK(setup.size() >= GOLDEN_COUNT(golden::CONNECT_SEQUENCE));
    setup.resize(GOLDEN_COUNT(golden::CONNECT_SEQUENCE));
    CHECK(checkDirectSequence(setup, golden::CONNECT_SEQUENCE,
                              GOLDEN_COUNT(golden::CONNECT_SEQUENCE)).empty());

    // Product identification and working devices.
    fake.push(ATTR_RESP_NEED_ACK, CMDSET_RM, CMD_RM_PRODUCT_ATTRIBUTE_GET, fromHex("0001"));
    fake.push(ATTR_NO_ACK, CMDSET_RM, CMD_RM_MODULE_STATUS_PUSH, fromHex("01000300"));
    sleepMs(100);
    RobotProduct product = robot.info().product;
    CHECK(product.model == RobotModel::ROBOMASTER_S1);
    CHECK(product.capabilities.components().count(RobotComponent::CHASSIS) == 1);

    // Mechanical commands are refused before arm(), and outside control mode
    // stopping has nothing to hold.
    CHECK_THROWS(robot.chassis.driveSpeed(0.1), std::runtime_error);
    CHECK_THROWS(robot.arm(), std::runtime_error);
    std::size_t outsideMark = fake.mark();
    robot.stop();
    sleepMs(100);
    CHECK_EQ(countMotion(fake.since(outsideMark)), 0);

    // robot-core AppSession.enterRemote: the setup and effect tables, then the
    // chassis SDK work mode with the stick overlay off.
    std::size_t enterMark = fake.mark();
    std::vector<std::uint16_t> enterSequences = robot.enterControlMode();
    sleepMs(100);
    std::vector<Captured> entered = fake.since(enterMark);
    std::vector<Captured> afterTables =
        checkDirectSequence(entered, golden::ENTER_SEQUENCE, GOLDEN_COUNT(golden::ENTER_SEQUENCE),
                            ENTER_SETUP_ROWS);
    CHECK_EQ(afterTables.size(), 2u);
    checkChassisModeFrame(afterTables[0], CMD_RM_WORK_MODE_SET, 1);
    checkChassisModeFrame(afterTables[1], CMD_RM_SPEED_MODE_SET, 0);
    CHECK_EQ(enterSequences.size(), GOLDEN_COUNT(golden::ENTER_SEQUENCE) + 2);
    // The actuators are held at zero from then on, and not before.
    bool speedModeSent = false;
    for (std::size_t i = 0; i < entered.size(); ++i) {
        const DussFrame& f = entered[i].frame;
        if (entered[i].type != 0x05) continue;
        if (f.receiver == HOST_CHASSIS && f.cmdset == CMDSET_RM && f.cmdid == CMD_RM_SPEED_MODE_SET) {
            speedModeSent = true;
        }
        if (isMotionCommand(f)) CHECK(speedModeSent);
    }
    CHECK(countMotion(entered, isChassisStop, idleChassisPayload()) >= 2);
    CHECK(countMotion(entered, isGimbalSpeed, idleGimbalPayload()) >= 2);
    CHECK(onlyIdleHold(entered));
    CHECK(robot.controlMode());
    CHECK(robot.enterControlMode().empty());

    // Request/ACK matching: accepted, rejected and timed out.
    RobotAck ack = robot.setLed("all", 0, 255, 0);
    CHECK(ack.accepted());
    CHECK_THROWS(robot.setSpeakerVolume(50), RequestRejected);
    CHECK_THROWS(robot.capture(0.2), TimeoutError);

    // Chassis velocity is sent as RM_SPEED_SET (no ACK) on every tick until its lease
    // lapses; the chassis then returns to the zero wheel-speed hold. The control
    // frames stay neutral throughout.
    robot.arm();
    CHECK(robot.armed());
    std::size_t driveMark = fake.mark();
    robot.chassis.driveSpeed(0.5, -0.5, 30, 0.15);
    sleepMs(100);
    std::vector<Captured> driving = fake.since(driveMark);
    CHECK(countMotion(driving, isChassisSpeed, buildChassisSpeedPayload(0.5, -0.5, 30)) >= 2);
    for (std::size_t i = 0; i < driving.size(); ++i) {
        if (driving[i].type != 0x05 || !isChassisSpeed(driving[i].frame)) continue;
        CHECK_EQ(driving[i].frame.attr, ATTR_NO_ACK);
        CHECK_EQ(toHex(driving[i].flags), std::string("0000"));
    }
    CHECK(onlyControlPayload(driving, neutralControl()));
    sleepMs(150);
    std::size_t expiredMark = fake.mark();
    sleepMs(100);
    std::vector<Captured> expired = fake.since(expiredMark);
    CHECK_EQ(countMotion(expired, isChassisSpeed), 0);
    CHECK(countMotion(expired, isChassisStop, idleChassisPayload()) >= 2);
    CHECK(sawControlPayload(expired, neutralControl()));

    // A zero velocity, negative zero included, is the stop command, not a speed.
    std::size_t zeroMark = fake.mark();
    robot.chassis.driveSpeed(-0.0, 0.0, -0.0, 0.5);
    sleepMs(80);
    std::vector<Captured> zeroed = fake.since(zeroMark);
    CHECK_EQ(countMotion(zeroed, isChassisSpeed), 0);
    CHECK(countMotion(zeroed, isChassisStop, idleChassisPayload()) >= 2);

    // Gimbal velocity goes out as GIMBAL_EXT_CTRL_ACCEL on every tick until its
    // lease lapses, then the gimbal returns to the zero hold.
    std::size_t gimbalMark = fake.mark();
    robot.gimbal.driveSpeed(30, -30, 0.1);
    sleepMs(80);
    std::vector<Captured> gimbal = fake.since(gimbalMark);
    CHECK(countMotion(gimbal, isGimbalSpeed, buildGimbalSpeedPayload(30, -30)) >= 2);
    sleepMs(100);
    std::size_t gimbalIdleMark = fake.mark();
    sleepMs(80);
    std::vector<Captured> gimbalIdle = fake.since(gimbalIdleMark);
    CHECK(countMotion(gimbalIdle, isGimbalSpeed, idleGimbalPayload()) >= 2);
    CHECK_EQ(countMotion(gimbalIdle, isGimbalSpeed, buildGimbalSpeedPayload(30, -30)), 0);

    // Infrared firing takes over only the control frame, so chassis motion carries
    // on; halting drops a pending trigger as AppSession.halt does.
    const Bytes trigger = fromHex("0000042000010840000230");
    robot.chassis.driveSpeed(0.2, 0, 0, 1.0);
    std::size_t fireMark = fake.mark();
    robot.fireInfrared(0.12);
    sleepMs(80);
    std::vector<Captured> firing = fake.since(fireMark);
    CHECK(sawControlPayload(firing, trigger));
    CHECK(countMotion(firing, isChassisSpeed, buildChassisSpeedPayload(0.2, 0, 0)) >= 2);
    sleepMs(100);
    std::size_t firedMark = fake.mark();
    sleepMs(60);
    std::vector<Captured> fired = fake.since(firedMark);
    CHECK(!sawControlPayload(fired, trigger));
    CHECK(sawControlPayload(fired, neutralControl()));
    robot.fireInfrared(2.0);
    robot.stop();
    std::size_t haltMark = fake.mark();
    const double haltDeadline = monotonicSeconds() + 0.3;
    std::vector<Captured> halted;
    do {
        sleepMs(10);
        halted = fake.since(haltMark);
    } while (countMotion(halted, isChassisStop, idleChassisPayload()) < 2 &&
             monotonicSeconds() < haltDeadline);
    CHECK(!sawControlPayload(halted, trigger));
    CHECK_EQ(countMotion(halted, isChassisSpeed), 0);
    CHECK(countMotion(halted, isChassisStop, idleChassisPayload()) >= 2);

    // Telemetry and media dispatch.
    double before = monotonicSeconds();
    fake.push(ATTR_NO_ACK, CMDSET_VIRTUAL_BUS, CMD_VBUS_DATA_ANALYSIS,
              fromHex("000a8403f6ff2c01ceff01"));
    GimbalTelemetry telemetry;
    CHECK(robot.waitForGimbal(telemetry, before, 1.0));
    CHECK(telemetry.yawDegrees() == 30.0);
    fake.push(ATTR_NO_ACK, CMDSET_VIRTUAL_BUS, CMD_VBUS_DATA_ANALYSIS,
              fromHex(golden::ODOMETRY_PAYLOAD));
    Odometry odometry;
    CHECK(robot.waitForOdometry(odometry, before, 1.0));
    int battery = 0;
    CHECK(robot.connection.battery(battery) && battery == 87);

    robot.camera.startVideoStream("1080p");
    fake.pushVideo(fromHex("0000000167"));
    Bytes chunk;
    CHECK(robot.camera.readVideoChunk(chunk, 1.0));
    CHECK_EQ(toHex(chunk), std::string("0000000167"));
    robot.camera.stopVideoStream();
    robot.camera.startAudioStream();
    fake.push(ATTR_NO_ACK, CMDSET_RM, CMD_RM_AUDIO_TO_APP, fromHex("48aa"));
    Bytes opus;
    CHECK(robot.camera.readAudioOpus(opus, 1.0));
    CHECK_EQ(toHex(opus), std::string("48aa"));
    robot.camera.stopAudioStream();

    std::size_t exitMark = fake.mark();
    robot.close();
    CHECK(!robot.connected());
    CHECK(!robot.controlMode());
    sleepMs(50);
    // robot-core AppSession.exitRemote: the exit table, then the chassis back to its
    // normal work mode, with the actuators held at zero throughout.
    std::vector<Captured> exited = fake.since(exitMark);
    std::vector<Captured> afterExit =
        checkDirectSequence(exited, golden::EXIT_SEQUENCE, GOLDEN_COUNT(golden::EXIT_SEQUENCE), 0);
    CHECK_EQ(afterExit.size(), 1u);
    checkChassisModeFrame(afterExit[0], CMD_RM_WORK_MODE_SET, 0);
    CHECK(idleFromFirstModeFrame(exited));
}

TEST(leaving_control_mode_stops_motion_and_releases_the_slots) {
    FakeRobot fake;
    Robot robot("127.0.0.1", APPID);
    robot.connection.suspendIdleKeepalive = true;
    CHECK(robot.initialize("sta", "udp", 5.0));
    robot.enterControlMode();
    robot.arm();
    robot.chassis.driveSpeed(0.3, 0, 0, 5.0);
    robot.gimbal.driveSpeed(10, 10, 5.0);
    sleepMs(100);

    // Disarming zeroes both actuators before the first exit-table frame goes out.
    std::size_t exitMark = fake.mark();
    robot.exitControlMode();
    CHECK(!robot.controlMode());
    CHECK(!robot.armed());
    CHECK(idleFromFirstModeFrame(fake.since(exitMark)));

    // Nothing is sent afterwards, and later calls cannot bring a slot back.
    std::size_t releasedMark = fake.mark();
    robot.stop();
    robot.chassis.stop();
    robot.gimbal.stop();
    CHECK_THROWS(robot.chassis.driveSpeed(0.1), std::runtime_error);
    CHECK_THROWS(robot.gimbal.driveSpeed(1, 1), std::runtime_error);
    sleepMs(100);
    CHECK_EQ(countMotion(fake.since(releasedMark)), 0);

    // Entering again resumes the zero hold.
    std::size_t reenterMark = fake.mark();
    robot.enterControlMode();
    sleepMs(100);
    std::vector<Captured> reentered = fake.since(reenterMark);
    CHECK(countMotion(reentered, isChassisStop, idleChassisPayload()) >= 2);
    CHECK(countMotion(reentered, isGimbalSpeed, idleGimbalPayload()) >= 2);
    CHECK(onlyIdleHold(reentered));
    robot.close();
}

TEST(silent_robot_stops_motion_like_the_app) {
    FakeRobot fake;
    Robot robot("127.0.0.1", APPID);
    robot.connection.suspendIdleKeepalive = true;
    CHECK(robot.initialize("sta", "udp", 5.0));
    robot.enterControlMode();
    robot.arm();
    robot.chassis.driveSpeed(0.3, 0, 0, 5.0);
    robot.gimbal.driveSpeed(10, 10, 5.0);

    // After 500 ms without any data, the next drive call stops both and throws.
    fake.setHeartbeat(false);
    sleepMs(600);
    CHECK_THROWS(robot.chassis.driveSpeed(0.3, 0, 0, 5.0), std::runtime_error);
    CHECK_THROWS(robot.fireInfrared(), std::runtime_error);
    std::size_t stoppedMark = fake.mark();
    sleepMs(100);
    std::vector<Captured> stopped = fake.since(stoppedMark);
    CHECK(countMotion(stopped, isChassisStop, idleChassisPayload()) >= 2);
    CHECK(countMotion(stopped, isGimbalSpeed, idleGimbalPayload()) >= 2);
    CHECK(onlyIdleHold(stopped));

    // Motion is accepted again once the robot is heard from.
    fake.setHeartbeat(true);
    sleepMs(200);
    std::size_t resumedMark = fake.mark();
    robot.gimbal.driveSpeed(10, 10, 5.0);
    sleepMs(100);
    CHECK(countMotion(fake.since(resumedMark), isGimbalSpeed, buildGimbalSpeedPayload(10, 10)) >= 1);
    robot.close();
}

namespace {

sys::Handle listenTcp(std::uint16_t& port) {
    sys::startup();
    sys::Handle fd = sys::openSocket(SOCK_STREAM);
    sockaddr_in address = loopback(0);
    sys::bindAddress(fd, address);
    sys::listenOn(fd, 1);
    sys::localAddress(fd, address);
    port = sys::hostPort(address.sin_port);
    return fd;
}

void writeLine(sys::Handle fd, const std::string& line) {
    std::string data = line + "\r\n";
    sys::sendBytes(fd, data.data(), data.size());
}

std::string readLine(sys::Handle fd) {
    std::string line;
    char c;
    while (sys::receiveBytes(fd, &c, 1) == 1 && c != '\n') {
        if (c != '\r') line.push_back(c);
    }
    return line;
}

}  // namespace

TEST(lab_upload_uses_anonymous_binary_passive_ftp) {
    std::uint16_t port = 0;
    sys::Handle server = listenTcp(port);
    std::vector<std::string> commands;
    std::string stored;
    std::thread ftp([&] {
        sys::Handle control = sys::acceptConnection(server);
        writeLine(control, "220-Fake S1");
        writeLine(control, "220 ready");
        sys::Handle data = sys::INVALID_HANDLE;
        for (;;) {
            std::string line = readLine(control);
            if (line.empty()) break;
            commands.push_back(line);
            if (line.compare(0, 4, "USER") == 0) {
                writeLine(control, "331 password");
            } else if (line.compare(0, 4, "PASS") == 0) {
                writeLine(control, "230 ok");
            } else if (line.compare(0, 4, "PASV") == 0) {
                std::uint16_t dataPort = 0;
                data = listenTcp(dataPort);
                char reply[64];
                std::snprintf(reply, sizeof(reply), "227 Entering Passive Mode (127,0,0,1,%u,%u)",
                              dataPort >> 8, dataPort & 0xFF);
                writeLine(control, reply);
            } else if (line.compare(0, 4, "STOR") == 0) {
                sys::Handle stream = sys::acceptConnection(data);
                writeLine(control, "150 sending");
                char buffer[4096];
                sys::IoSize size;
                while ((size = sys::receiveBytes(stream, buffer, sizeof(buffer))) > 0) {
                    stored.append(buffer, static_cast<std::size_t>(size));
                }
                sys::closeHandle(stream);
                sys::closeHandle(data);
                writeLine(control, "226 done");
            } else if (line == "QUIT") {
                writeLine(control, "221 bye");
                break;
            } else {
                writeLine(control, "200 ok");
            }
        }
        sys::closeHandle(control);
    });

    LabProgramIdentity identity;
    std::string dsp = buildLabProgram("def start():\n    pass\n", identity);
    std::string digest = uploadLabProgram("127.0.0.1", dsp, DEFAULT_UPLOAD_NAME, 5.0, port);
    ftp.join();
    sys::closeHandle(server);

    CHECK_EQ(stored, dsp);
    CHECK_EQ(digest, md5Hex(dsp));
    CHECK(commands.size() >= 6);
    CHECK_EQ(commands[0], std::string("USER anonymous"));
    CHECK_EQ(commands[1], std::string("PASS anonymous@"));
    CHECK_EQ(commands[2], std::string("CWD python"));
    CHECK_EQ(commands[3], std::string("TYPE I"));
    CHECK_EQ(commands[4], std::string("PASV"));
    CHECK_EQ(commands[5], std::string("STOR python_raw.dsp"));
}

namespace {

// Kinematic stand-in for the robot: integrates the gimbal speed and chassis
// rotation commands the SDK sends and pushes 10 Hz telemetry back, like S1.
class MotionSimulator {
public:
    MotionSimulator(FakeRobot& fake, double chassisYaw, double attitudeSign = 1,
                    double wobble = 0, double stopCoast = 0)
        : fake_(fake), done_(false), cursor_(fake.mark()), yaw_(0), pitch_(0),
          chassisYaw_(chassisYaw), yawSpeed_(0), pitchSpeed_(0), chassisRate_(0),
          attitudeSign_(attitudeSign), wobble_(wobble), stopCoast_(stopCoast), tick_(0),
          coasted_(false) {
        thread_ = std::thread(&MotionSimulator::run, this);
    }
    ~MotionSimulator() {
        done_ = true;
        thread_.join();
    }
    void angles(double& yaw, double& pitch, double& chassisYaw) {
        std::lock_guard<std::mutex> lock(lock_);
        yaw = yaw_;
        pitch = pitch_;
        chassisYaw = chassisYaw_;
    }
    void driftHeading(double degrees) {
        std::lock_guard<std::mutex> lock(lock_);
        chassisYaw_ += degrees;
    }

private:
    static double int16At(const Bytes& data, std::size_t offset) {
        return static_cast<std::int16_t>(data[offset] | (data[offset + 1] << 8));
    }

    static double floatAt(const Bytes& data, std::size_t offset) {
        std::uint32_t bits = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) |
                             (static_cast<std::uint32_t>(data[offset + 3]) << 24);
        float value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    // The latest command of each actuator wins; the SDK sends a zero hold once a
    // lease lapses, so a stopped actuator shows up as an explicit zero.
    void observe(const std::vector<Captured>& items) {
        for (std::size_t i = 0; i < items.size(); ++i) {
            const DussFrame& f = items[i].frame;
            if (items[i].type != 0x05) continue;
            if (isGimbalSpeed(f) && f.payload.size() == 7) {
                // int16 yaw, roll, pitch in 0.1 deg/s, then the accel-control flag.
                yawSpeed_ = int16At(f.payload, 0) / 10.0;
                pitchSpeed_ = int16At(f.payload, 4) / 10.0;
            } else if (isChassisSpeed(f) && f.payload.size() == 12) {
                chassisRate_ = floatAt(f.payload, 8);  // clockwise deg/s
            } else if (isChassisStop(f)) {
                chassisRate_ = 0;
            }
        }
    }

    void run() {
        while (!done_) {
            sleepMs(100);
            std::vector<Captured> items = fake_.since(cursor_);
            cursor_ += items.size();
            Bytes gimbal(2, 0);
            Bytes attitude(2, 0);
            {
                std::lock_guard<std::mutex> lock(lock_);
                double previousRate = chassisRate_;
                observe(items);
                yaw_ += yawSpeed_ * 0.1;
                pitch_ += pitchSpeed_ * 0.1;
                chassisYaw_ += chassisRate_ * 0.1;
                // One deterministic coast after the first stop, plus alternating
                // measurement wobble while moving. Physical yaw stays observable.
                if (!coasted_ && previousRate != 0 && chassisRate_ == 0 && stopCoast_ != 0) {
                    chassisYaw_ += previousRate > 0 ? stopCoast_ : -stopCoast_;
                    coasted_ = true;
                }
                if (chassisYaw_ > 180) chassisYaw_ -= 360;
                if (chassisYaw_ <= -180) chassisYaw_ += 360;
                gimbal[1] = 0x0A;
                std::int16_t values[4] = {0, 0, static_cast<std::int16_t>(yaw_ * 10),
                                          static_cast<std::int16_t>(pitch_ * 10)};
                for (int v = 0; v < 4; ++v) {
                    gimbal.push_back(static_cast<std::uint8_t>(values[v] & 0xFF));
                    gimbal.push_back(static_cast<std::uint8_t>((values[v] >> 8) & 0xFF));
                }
                gimbal.push_back(0);
                attitude[1] = 13;
                double noise = chassisRate_ == 0 ? 0 : (++tick_ % 2 ? wobble_ : -wobble_);
                float angles[3] = {static_cast<float>(attitudeSign_ * chassisYaw_ + noise),
                                   0.0f, 0.0f};
                for (int a = 0; a < 3; ++a) {
                    std::uint32_t bits;
                    std::memcpy(&bits, &angles[a], sizeof(bits));
                    for (int b = 0; b < 4; ++b) attitude.push_back((bits >> (8 * b)) & 0xFF);
                }
            }
            fake_.push(ATTR_NO_ACK, CMDSET_VIRTUAL_BUS, CMD_VBUS_DATA_ANALYSIS, gimbal);
            fake_.push(ATTR_NO_ACK, CMDSET_VIRTUAL_BUS, CMD_VBUS_DATA_ANALYSIS, attitude);
        }
    }

    FakeRobot& fake_;
    std::atomic<bool> done_;
    std::size_t cursor_;
    std::mutex lock_;
    double yaw_, pitch_, chassisYaw_;
    double yawSpeed_, pitchSpeed_, chassisRate_;
    double attitudeSign_, wobble_, stopCoast_;
    unsigned tick_;
    bool coasted_;
    std::thread thread_;
};

bool sawDuss(const std::vector<Captured>& items, std::uint8_t cmdset, std::uint8_t cmdid,
             const Bytes& payload) {
    for (std::size_t i = 0; i < items.size(); ++i) {
        const DussFrame& f = items[i].frame;
        if (f.cmdset == cmdset && f.cmdid == cmdid && f.payload == payload) return true;
    }
    return false;
}

}  // namespace

namespace {

void checkTurnWithWobble(int direction, double degrees, double attitudeSign) {
    FakeRobot fake;
    Robot robot("127.0.0.1", APPID);
    robot.connection.suspendIdleKeepalive = true;
    CHECK(robot.initialize("sta", "udp", 5.0));
    MotionSimulator sim(fake, 170, attitudeSign, 3, 4);
    ctrl::RobotCtrl rm(robot);
    rm.chassis_ctrl.set_rotate_speed(60);
    rm.chassis_ctrl.rotate_with_degree(direction, degrees);
    double yaw, pitch, chassisYaw;
    sim.angles(yaw, pitch, chassisYaw);
    double target = 170 + (direction == ctrl::rm_define::clockwise ? degrees : -degrees);
    CHECK(std::fabs(ctrl::angleDifference(chassisYaw, target)) <= 1.1);
    double stopped = chassisYaw;
    sleepMs(350);
    sim.angles(yaw, pitch, chassisYaw);
    CHECK(std::fabs(ctrl::angleDifference(chassisYaw, stopped)) < 0.25);
    CHECK(rm.setpoint().z == 0);
    CHECK_EQ(rm.chassis_ctrl.yaw_polarity(), static_cast<int>(attitudeSign));
}

}  // namespace

TEST(chassis_turn_rejects_wobble_and_corrects_stop_coast) {
    checkTurnWithWobble(ctrl::rm_define::clockwise, 90, 1);
}

TEST(chassis_turn_supports_reversed_yaw_left_turn_and_multiple_wraps) {
    checkTurnWithWobble(ctrl::rm_define::anticlockwise, 450, -1);
}

TEST(lab_style_ctrl_closes_angle_loops_on_telemetry) {
    FakeRobot fake;
    fake.ack(CMDSET_RM, CMD_RM_LED_COLOR_SET, Bytes(1, 0));
    Robot robot("127.0.0.1", APPID);
    robot.connection.suspendIdleKeepalive = true;
    CHECK(robot.initialize("sta", "udp", 5.0));
    // Start near the +/-180 wrap so the chassis loop must unwrap yaw.
    MotionSimulator sim(fake, 170.0);
    double yaw, pitch, chassisYaw;
    {
        std::size_t mark = fake.mark();
        ctrl::RobotCtrl rm(robot);
        CHECK(robot.controlMode());
        CHECK(robot.armed());
        sleepMs(100);
        CHECK(sawDuss(fake.since(mark), CMDSET_VIRTUAL_BUS, CMD_VBUS_ADD_MSG,
                      buildDdsSubscriptionPayload(13, 0x000200096B986306ULL, 10)));

        rm.gimbal_ctrl.angle_ctrl(20, 10);
        sim.angles(yaw, pitch, chassisYaw);
        CHECK(std::fabs(yaw - 20) < 1.5);
        CHECK(std::fabs(pitch - 10) < 1.5);
        CHECK(std::fabs(rm.gimbal_ctrl.get_axis_angle(ctrl::rm_define::gimbal_axis_yaw) - 20) < 1.5);

        rm.chassis_ctrl.set_rotate_speed(60);
        rm.chassis_ctrl.rotate_with_degree(ctrl::rm_define::clockwise, 90);
        sleepMs(200);
        sim.angles(yaw, pitch, chassisYaw);
        CHECK(std::fabs(ctrl::angleDifference(chassisYaw, 260)) < 3.0);

        rm.gimbal_ctrl.recenter();
        sim.angles(yaw, pitch, chassisYaw);
        CHECK(std::fabs(yaw) < 1.5);
        CHECK(std::fabs(pitch) < 1.5);

        std::size_t ledMark = fake.mark();
        rm.led_ctrl.set_top_led(ctrl::rm_define::armor_all, 255, 0, 0,
                                ctrl::rm_define::effect_always_on);
        CHECK(sawDuss(fake.since(ledMark), CMDSET_RM, CMD_RM_LED_COLOR_SET,
                      buildLedPayload(0x30, 255, 0, 0, "on")));
        CHECK_THROWS(rm.chassis_ctrl.move_with_distance(0, 6), std::invalid_argument);
    }
    CHECK(!robot.controlMode());
    CHECK(!robot.armed());
}

namespace {

void appendFloatLe(Bytes& out, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
}

// Robot-side vision and camera: answers enable/read-back and camera commands,
// and every 100 ms pushes camera status plus, while markers are enabled, one
// marker whose frame position follows the simulated gimbal.
class VisionSimulator {
public:
    VisionSimulator(FakeRobot& fake, MotionSimulator& motion)
        : fake_(fake), motion_(motion), done_(false), mask_(0), otherOwner_(false), mode_(0),
          recording_(false), markerId_(ctrl::rm_define::marker_trans_red_heart),
          markerYaw_(15.0), markerPitch_(8.0) {
        fake_.respond([this](const DussFrame& f, Bytes& answer) { return answer_(f, answer); });
        thread_ = std::thread(&VisionSimulator::run, this);
    }
    ~VisionSimulator() {
        done_ = true;
        thread_.join();
        fake_.respond(FakeRobot::Responder());
    }
    void setOtherOwner(bool value) {
        std::lock_guard<std::mutex> lock(lock_);
        otherOwner_ = value;
    }
    void setMarker(int id) {
        std::lock_guard<std::mutex> lock(lock_);
        markerId_ = id;
    }
    std::uint16_t mask() {
        std::lock_guard<std::mutex> lock(lock_);
        return mask_;
    }
    bool recording() {
        std::lock_guard<std::mutex> lock(lock_);
        return recording_;
    }

private:
    bool answer_(const DussFrame& f, Bytes& answer) {
        std::lock_guard<std::mutex> lock(lock_);
        if (f.cmdset == CMDSET_VISION && f.cmdid == 0xA5) {
            std::uint16_t reported = otherOwner_ ? 0x40 : mask_;
            answer = Bytes(1, 0);
            answer.push_back(static_cast<std::uint8_t>(reported & 0xFF));
            answer.push_back(static_cast<std::uint8_t>(reported >> 8));
            return true;
        }
        if (f.cmdset == CMDSET_VISION && f.cmdid == 0xA3 && f.payload.size() == 2) {
            mask_ = static_cast<std::uint16_t>(f.payload[0] | (f.payload[1] << 8));
            answer = Bytes(1, 0);
            return true;
        }
        if (f.cmdset == CMDSET_CAMERA && f.cmdid == 0x10 && f.payload.size() == 1) {
            mode_ = f.payload[0];
            answer = Bytes(1, 0);
            return true;
        }
        if (f.cmdset == CMDSET_CAMERA && f.cmdid == 0x02 && f.payload.size() == 1) {
            recording_ = mode_ == 1 && f.payload[0] == 1;
            answer = Bytes(1, 0);
            return true;
        }
        return false;
    }

    void run() {
        while (!done_) {
            sleepMs(100);
            double yaw, pitch, chassisYaw;
            motion_.angles(yaw, pitch, chassisYaw);
            Bytes camera(44, 0);
            Bytes marker;
            {
                std::lock_guard<std::mutex> lock(lock_);
                camera[2] = 0x10;
                unsigned flags = 0xC0 | mode_ | (recording_ ? 0x1000 : 0);
                camera[5] = static_cast<std::uint8_t>(flags & 0xFF);
                camera[6] = static_cast<std::uint8_t>(flags >> 8);
                double x = 0.5 + (markerYaw_ - yaw) / 96.0;
                double y = 0.53 - (markerPitch_ - pitch) / 54.0;
                if (mask_ == 0x20 && x > 0 && x < 1 && y > 0 && y < 1) {
                    marker = fromHex("050300000000000001");
                    appendFloatLe(marker, static_cast<float>(x));
                    appendFloatLe(marker, static_cast<float>(y));
                    appendFloatLe(marker, 0.1f);
                    appendFloatLe(marker, 0.15f);
                    std::uint32_t info = static_cast<std::uint32_t>(markerId_) | (120u << 16);
                    for (int i = 0; i < 4; ++i) marker.push_back(static_cast<std::uint8_t>(info >> (8 * i)));
                }
            }
            fake_.pushFrom(HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_CAMERA, 0xE4, camera);
            if (!marker.empty()) fake_.pushFrom(HOST_VISION, ATTR_NO_ACK, CMDSET_VISION, 0xA4, marker);
        }
    }

    FakeRobot& fake_;
    MotionSimulator& motion_;
    std::atomic<bool> done_;
    std::mutex lock_;
    std::uint16_t mask_;
    bool otherOwner_;
    std::uint8_t mode_;
    bool recording_;
    int markerId_;
    double markerYaw_, markerPitch_;
    std::thread thread_;
};

std::vector<std::string> visionCommands(const std::vector<Captured>& items) {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < items.size(); ++i) {
        const DussFrame& f = items[i].frame;
        if (f.cmdset == CMDSET_VISION) out.push_back(toHex(Bytes(1, f.cmdid)) + ":" + toHex(f.payload));
    }
    return out;
}

}  // namespace

TEST(lab_style_vision_and_recording_over_loopback) {
    using namespace ctrl;
    FakeRobot fake;
    Robot robot("127.0.0.1", APPID);
    robot.connection.suspendIdleKeepalive = true;
    CHECK(robot.initialize("sta", "udp", 5.0));
    MotionSimulator motion(fake, 0.0);
    VisionSimulator sim(fake, motion);
    {
        RobotCtrl rm(robot);
        CHECK_THROWS(rm.vision_ctrl.get_marker_detection_info(), std::logic_error);

        // Another owner's detector is never reset.
        sim.setOtherOwner(true);
        std::size_t mark = fake.mark();
        CHECK_THROWS(rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker), VisionError);
        std::vector<std::string> sent = visionCommands(fake.since(mark));
        CHECK_EQ(sent.size(), 1u);
        CHECK_EQ(sent[0], std::string("a5:"));
        CHECK(!robot.vision.active());
        sim.setOtherOwner(false);

        // Inspect, reset, enable, verify; as robot-core VisionControl.
        mark = fake.mark();
        rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
        sent = visionCommands(fake.since(mark));
        CHECK_EQ(sent.size(), 4u);
        CHECK_EQ(sent[0], std::string("a5:"));
        CHECK_EQ(sent[1], std::string("a3:0000"));
        CHECK_EQ(sent[2], std::string("a3:2000"));
        CHECK_EQ(sent[3], std::string("a5:"));
        CHECK(robot.vision.active());

        CHECK(rm.vision_ctrl.cond_wait_marker(rm_define::marker_trans_red_heart, 2.0));
        CHECK(!rm.vision_ctrl.cond_wait_marker(rm_define::marker_trans_dice, 0.3));
        std::vector<double> info = rm.vision_ctrl.get_marker_detection_info();
        CHECK_EQ(info.size(), 6u);
        CHECK_EQ(info[0], 1.0);
        CHECK_EQ(info[1], static_cast<double>(rm_define::marker_trans_red_heart));
        CHECK(rm.vision_ctrl.check_marker(rm_define::marker_all));

        // Centre the marker (15, 8 degrees off) using only vision feedback.
        CHECK(rm.vision_ctrl.detect_marker_and_aim(rm_define::marker_trans_red_heart));
        double yaw, pitch, chassisYaw;
        motion.angles(yaw, pitch, chassisYaw);
        CHECK(std::fabs(yaw - 15.0) < 2.0);
        CHECK(std::fabs(pitch - 8.0) < 2.0);
        sim.setMarker(rm_define::marker_number_zero + 1);
        CHECK(!rm.vision_ctrl.detect_marker_and_aim(rm_define::marker_trans_target));

        // Switching detector stops the running one first.
        mark = fake.mark();
        rm.vision_ctrl.enable_detection(rm_define::vision_detection_pose);
        sent = visionCommands(fake.since(mark));
        CHECK_EQ(sent.size(), 6u);
        CHECK_EQ(sent[0], std::string("a3:0000"));
        CHECK_EQ(sent[4], std::string("a3:0400"));
        CHECK_EQ(sim.mask(), 0x04);
        CHECK(!rm.vision_ctrl.check_pose(rm_define::pose_capture));
        rm.vision_ctrl.disable_detection(rm_define::vision_detection_pose);
        CHECK_EQ(sim.mask(), 0);
        CHECK(!robot.vision.active());

        // Recording switches to video mode and waits for the status to confirm.
        rm.media_ctrl.record(rm_define::record_on);
        CHECK(sim.recording());
        CameraStatus status;
        CHECK(robot.cameraStatus(status));
        CHECK_EQ(status.workMode, 1);
        rm.media_ctrl.record(rm_define::record_off);
        CHECK(!sim.recording());
        // Taking a photo afterwards returns the camera to photo mode.
        fake.ack(CMDSET_CAMERA, CMD_CAPTURE, Bytes(1, 0));
        mark = fake.mark();
        rm.media_ctrl.capture();
        CHECK(sawDuss(fake.since(mark), CMDSET_CAMERA, 0x10, Bytes(1, 0)));
        CHECK(sawDuss(fake.since(mark), CMDSET_CAMERA, CMD_CAPTURE, Bytes(1, 1)));

        rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
    }
    // Leaving the Lab controller disables vision.
    CHECK(!robot.vision.active());
    CHECK_EQ(sim.mask(), 0);
}

namespace {

// Robot status broadcast on UDP 45678 carrying the loopback address and APPID,
// repeated like the S1 does while it is not yet driven by a controller.
class Broadcaster {
public:
    Broadcaster() : done_(false) {
        Bytes decoded(24, 0);
        decoded[0] = 0x5A;
        decoded[1] = 0x5B;
        decoded[6] = 127;
        decoded[9] = 1;
        std::memcpy(&decoded[16], APPID, 8);
        packet_ = test::encodeBroadcast(decoded);
        sys::startup();
        fd_ = sys::openSocket(SOCK_DGRAM);
        thread_ = std::thread(&Broadcaster::run, this);
    }
    ~Broadcaster() {
        done_ = true;
        thread_.join();
        sys::closeHandle(fd_);
    }

private:
    void run() {
        while (!done_) {
            sendTo(fd_, packet_, loopback(APP_PORT));
            sleepMs(100);
        }
    }

    std::atomic<bool> done_;
    Bytes packet_;
    sys::Handle fd_;
    std::thread thread_;
};

}  // namespace

TEST(edu_entry_finds_the_robot_and_moves_within_limits) {
    CHECK(!edu::forward(1));  // not connected yet
    FakeRobot fake;
    fake.ack(CMDSET_RM, CMD_RM_PLAY_SOUND, Bytes(1, 0));
    MotionSimulator sim(fake, 0.0);
    bool connected;
    {
        Broadcaster broadcaster;
        connected = edu::connect();
    }
    CHECK(connected);
    CHECK(edu::connected());

    std::size_t mark = fake.mark();
    double start = monotonicSeconds();
    CHECK(edu::forward(0.3));
    CHECK(monotonicSeconds() - start >= 0.29);
    CHECK(countMotion(fake.since(mark), isChassisSpeed,
                      buildChassisSpeedPayload(edu::MOVE_SPEED, 0, 0)) >= 2);

    mark = fake.mark();
    CHECK(edu::left(0.2));
    std::vector<Captured> sliding = fake.since(mark);
    bool slidLeft = false;
    for (std::size_t i = 0; i < sliding.size(); ++i) {
        const DussFrame& f = sliding[i].frame;
        if (!isChassisSpeed(f) || f.payload.size() < 8) continue;
        float x, y;
        std::memcpy(&x, &f.payload[0], 4);
        std::memcpy(&y, &f.payload[4], 4);
        slidLeft = slidLeft || (std::fabs(x) < 1e-3 && std::fabs(y + edu::MOVE_SPEED) < 1e-3);
    }
    CHECK(slidLeft);

    double yaw, pitch, chassisYaw;
    CHECK(edu::turnRight(90));
    sleepMs(200);
    sim.angles(yaw, pitch, chassisYaw);
    CHECK(std::fabs(ctrl::angleDifference(chassisYaw, 90)) < 3.0);

    // Translation can drift yaw; the next turn must end on the original path's
    // target, rather than adding ninety degrees to that disturbed heading.
    for (int side = 1; side < 4; ++side) {
        sim.driftHeading(4);
        sleepMs(200);
        CHECK(edu::turnRight(90));
        sim.angles(yaw, pitch, chassisYaw);
        CHECK(std::fabs(ctrl::angleDifference(chassisYaw, (side + 1) * 90)) < 1.5);
    }

    mark = fake.mark();
    CHECK(edu::beep());
    bool beeped = false;
    std::vector<Captured> sounds = fake.since(mark);
    for (std::size_t i = 0; i < sounds.size(); ++i) {
        beeped = beeped || (sounds[i].frame.cmdset == CMDSET_RM &&
                            sounds[i].frame.cmdid == CMD_RM_PLAY_SOUND);
    }
    CHECK(beeped);

    CHECK(!edu::forward(-1));
    CHECK(!edu::forward(edu::MAX_SECONDS + 1));
    CHECK(!edu::turnLeft(edu::MAX_DEGREES + 1));
    edu::wait(0);

    edu::disconnect();
    CHECK(!edu::connected());
    CHECK(!edu::stop());
}

#ifndef _WIN32
// A runner's stop button sends SIGTERM to the student program, which must zero
// the chassis before it dies. Forked first so the child is single-threaded.
TEST(edu_entry_stops_the_robot_on_sigterm) {
    pid_t child = ::fork();
    CHECK(child >= 0);
    if (child == 0) {
        bool ok = edu::connect() && edu::forward(edu::MAX_SECONDS);
        ::_exit(ok ? 0 : 3);
    }
    FakeRobot fake;
    MotionSimulator sim(fake, 0.0);
    {
        Broadcaster broadcaster;
        double deadline = monotonicSeconds() + 10;
        while (monotonicSeconds() < deadline &&
               countMotion(fake.since(0), isChassisSpeed,
                           buildChassisSpeedPayload(edu::MOVE_SPEED, 0, 0)) == 0) {
            sleepMs(50);
        }
    }
    std::size_t mark = fake.mark();
    CHECK(countMotion(fake.since(0), isChassisSpeed,
                      buildChassisSpeedPayload(edu::MOVE_SPEED, 0, 0)) > 0);
    ::kill(child, SIGTERM);
    int status = 0;
    CHECK_EQ(::waitpid(child, &status, 0), child);
    CHECK(WIFSIGNALED(status) && WTERMSIG(status) == SIGTERM);
    CHECK(countMotion(fake.since(mark), isChassisStop, idleChassisPayload()) >= 2);
}
#endif

TEST(lab_style_entry_owns_its_robot) {
    FakeRobot fake;
    {
        ctrl::RobotCtrl rm("127.0.0.1", APPID);
        CHECK(rm.robot.connected());
        CHECK(rm.robot.controlMode());
        CHECK(rm.robot.armed());
    }
    // An exhausted discovery budget must time out even if a real robot is
    // broadcasting on this network; never connect to hardware in this check.
    CHECK_THROWS(ctrl::RobotCtrl::connectDiscovered(0), TimeoutError);
}
