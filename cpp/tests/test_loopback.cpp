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
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return address;
}

sys::Handle udpSocket(std::uint16_t port) {
    sys::startup();
    sys::Handle fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1;
    sys::setOption(fd, SOL_SOCKET, SO_REUSEADDR, one);
    sys::ignoreUdpConnectionReset(fd);
    sockaddr_in address = loopback(port);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
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
        Bytes duss = buildDuss(sender, HOST_MOBILE, attr, cmdset, cmdid, payload, sequence_++);
        Bytes packet(20, 0);
        packet[2] = session_[0];
        packet[3] = session_[1];
        packet[6] = 0x09;
        packet.insert(packet.end(), duss.begin(), duss.end());
        sendTo(controlFd_, packet, peer_);
    }

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
        while (!stop_) {
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

std::vector<Captured> directOnly(const std::vector<Captured>& items) {
    std::vector<Captured> out;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x05) out.push_back(items[i]);
    }
    return out;
}

void checkDirectSequence(const std::vector<Captured>& actual, const golden::GoldenEntry* expected,
                         std::size_t count) {
    std::vector<Captured> direct = directOnly(actual);
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
        CHECK_EQ(toHex(got.flags), std::string(expected[i].flags));
    }
    CHECK_EQ(direct.size(), expectedDirect);
}

bool sawControlPayload(const std::vector<Captured>& items, const Bytes& payload) {
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].type == 0x04 && items[i].frame.payload == payload) return true;
    }
    return false;
}

}  // namespace

TEST(loopback_session_matches_python_reference_flow) {
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
    checkDirectSequence(setup, golden::CONNECT_SEQUENCE, GOLDEN_COUNT(golden::CONNECT_SEQUENCE));

    // Product identification and working devices.
    fake.push(ATTR_RESP_NEED_ACK, CMDSET_RM, CMD_RM_PRODUCT_ATTRIBUTE_GET, fromHex("0001"));
    fake.push(ATTR_NO_ACK, CMDSET_RM, CMD_RM_MODULE_STATUS_PUSH, fromHex("01000300"));
    sleepMs(100);
    RobotProduct product = robot.info().product;
    CHECK(product.model == RobotModel::ROBOMASTER_S1);
    CHECK(product.capabilities.components().count(RobotComponent::CHASSIS) == 1);

    // Mechanical commands are refused before arm().
    CHECK_THROWS(robot.chassis.driveSpeed(0.1), std::runtime_error);
    CHECK_THROWS(robot.arm(), std::runtime_error);

    std::size_t enterMark = fake.mark();
    robot.enterControlMode();
    sleepMs(100);
    checkDirectSequence(fake.since(enterMark), golden::ENTER_SEQUENCE,
                        GOLDEN_COUNT(golden::ENTER_SEQUENCE));
    CHECK(robot.controlMode());
    CHECK(robot.enterControlMode().empty());

    // Request/ACK matching: accepted, rejected and timed out.
    RobotAck ack = robot.setLed("all", 0, 255, 0);
    CHECK(ack.accepted());
    CHECK_THROWS(robot.setSpeakerVolume(50), RequestRejected);
    CHECK_THROWS(robot.capture(0.2), TimeoutError);

    // Chassis velocity is repeated on the control channel until its lease expires.
    robot.arm();
    CHECK(robot.armed());
    std::size_t driveMark = fake.mark();
    robot.chassis.driveSpeed(0.5, -0.5, 30, 0.15);
    sleepMs(100);
    CHECK(sawControlPayload(fake.since(driveMark), buildChassisControlPayload(0.5, -0.5, 30)));
    sleepMs(150);
    std::size_t expiredMark = fake.mark();
    sleepMs(100);
    std::vector<Captured> expired = fake.since(expiredMark);
    CHECK(!sawControlPayload(expired, buildChassisControlPayload(0.5, -0.5, 30)));
    CHECK(sawControlPayload(expired, neutralControl()));

    // Gimbal speed rides along each tick as a periodic direct command.
    std::size_t gimbalMark = fake.mark();
    robot.gimbal.driveSpeed(30, -30, 0.1);
    sleepMs(80);
    std::vector<Captured> gimbal = fake.since(gimbalMark);
    int gimbalFrames = 0;
    for (std::size_t i = 0; i < gimbal.size(); ++i) {
        if (gimbal[i].frame.cmdset == CMDSET_GIMBAL &&
            gimbal[i].frame.payload == buildGimbalSpeedPayload(30, -30)) {
            ++gimbalFrames;
        }
    }
    CHECK(gimbalFrames >= 2);

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
    checkDirectSequence(fake.since(exitMark), golden::EXIT_SEQUENCE,
                        GOLDEN_COUNT(golden::EXIT_SEQUENCE));
}

namespace {

sys::Handle listenTcp(std::uint16_t& port) {
    sys::startup();
    sys::Handle fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address = loopback(0);
    ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    ::listen(fd, 1);
    sys::AddressLength length = static_cast<sys::AddressLength>(sizeof(address));
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length);
    port = ntohs(address.sin_port);
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
        sys::Handle control = ::accept(server, NULL, NULL);
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
                sys::Handle stream = ::accept(data, NULL, NULL);
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
    MotionSimulator(FakeRobot& fake, double chassisYaw)
        : fake_(fake), done_(false), cursor_(fake.mark()), yaw_(0), pitch_(0),
          chassisYaw_(chassisYaw), yawSpeed_(0), pitchSpeed_(0), chassisRate_(0) {
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

private:
    static double int16At(const Bytes& data, std::size_t offset) {
        return static_cast<std::int16_t>(data[offset] | (data[offset + 1] << 8));
    }

    void observe(const std::vector<Captured>& items) {
        bool sawGimbal = false;
        for (std::size_t i = 0; i < items.size(); ++i) {
            const DussFrame& f = items[i].frame;
            if (f.cmdset == CMDSET_GIMBAL && f.cmdid == CMD_GIMBAL_ROTATE_SPEED &&
                f.payload.size() == 6) {
                pitchSpeed_ = -int16At(f.payload, 2) / 1024.0 * 120.0;
                yawSpeed_ = -int16At(f.payload, 4) / 1024.0 * 120.0;
                sawGimbal = true;
            } else if (items[i].type == 0x04 && f.payload.size() == 14) {
                int angular = (f.payload[5] >> 4) | (f.payload[6] << 4);
                if (angular & 0x800) angular -= 0x1000;
                chassisRate_ = angular / 256.0 * 150.0;
            }
        }
        // Periodic gimbal commands stop when their lease expires.
        if (!sawGimbal) yawSpeed_ = pitchSpeed_ = 0;
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
                observe(items);
                yaw_ += yawSpeed_ * 0.1;
                pitch_ += pitchSpeed_ * 0.1;
                chassisYaw_ += chassisRate_ * 0.1;
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
                float angles[3] = {static_cast<float>(chassisYaw_), 0.0f, 0.0f};
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
