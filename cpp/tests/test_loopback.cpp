// Loopback integration tests: a scripted fake S1 on 127.0.0.1 exercises the
// AppID claim, outer session, setup replay, request/ACK matching, command
// leases, telemetry, media dispatch and Lab FTP upload without hardware.
// Requires the fixed App ports (45678, 56789, 10607, 10609) to be free.
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "golden.hpp"
#include "hanppie/hanppie.hpp"
#include "test_support.hpp"

using namespace hanppie;
using namespace hanppie::protocol;

namespace {

const char* const APPID = "b6359877";

struct Captured {
    std::uint8_t type;  // outer envelope type: 5 direct, 4 control
    Bytes flags;        // direct envelope flags (bytes 18..20)
    DussFrame frame;
};

int udpSocket(std::uint16_t port) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(fd);
        throw std::runtime_error("fake robot could not bind a required port");
    }
    timeval timeout = {0, 20000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return fd;
}

void sendTo(int fd, const Bytes& data, const sockaddr_in& address) {
    ::sendto(fd, data.data(), data.size(), 0, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address));
}

sockaddr_in loopback(std::uint16_t port) {
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return address;
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
        ::close(appFd_);
        ::close(controlFd_);
    }

    // Reply to requests for (cmdset, cmdid) with the given ACK payload.
    void ack(std::uint8_t cmdset, std::uint8_t cmdid, const Bytes& payload) {
        std::lock_guard<std::mutex> lock(lock_);
        acks_[std::make_pair(cmdset, cmdid)] = payload;
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
        Bytes duss = buildDuss(HOST_HDVT_UAV, HOST_MOBILE, attr, cmdset, cmdid, payload,
                               sequence_++);
        Bytes packet(20, 0);
        std::lock_guard<std::mutex> lock(lock_);
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
            pollfd items[2] = {{appFd_, POLLIN, 0}, {controlFd_, POLLIN, 0}};
            if (::poll(items, 2, 20) <= 0) continue;
            sockaddr_in from;
            socklen_t length = sizeof(from);
            if (items[0].revents & POLLIN) {
                ssize_t size = ::recvfrom(appFd_, buffer.data(), buffer.size(), 0,
                                          reinterpret_cast<sockaddr*>(&from), &length);
                if (size == 8) answerClaim(Bytes(buffer.begin(), buffer.begin() + size));
            }
            if (items[1].revents & POLLIN) {
                length = sizeof(from);
                ssize_t size = ::recvfrom(controlFd_, buffer.data(), buffer.size(), 0,
                                          reinterpret_cast<sockaddr*>(&from), &length);
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
                if ((frames[i].attr & 0x40) && found != acks_.end()) {
                    Bytes duss = buildDuss(frames[i].receiver, frames[i].sender, ATTR_ACK,
                                           frames[i].cmdset, frames[i].cmdid, found->second,
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

    int appFd_;
    int controlFd_;
    std::atomic<bool> stop_;
    std::uint16_t sequence_;
    std::thread thread_;
    std::mutex lock_;
    bool hasPeer_;
    sockaddr_in peer_;
    Bytes session_;
    std::vector<Captured> captured_;
    std::map<std::pair<std::uint8_t, std::uint8_t>, Bytes> acks_;
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

int listenTcp(std::uint16_t& port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in address = loopback(0);
    ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    ::listen(fd, 1);
    socklen_t length = sizeof(address);
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length);
    port = ntohs(address.sin_port);
    return fd;
}

void writeLine(int fd, const std::string& line) {
    std::string data = line + "\r\n";
    ::send(fd, data.data(), data.size(), 0);
}

std::string readLine(int fd) {
    std::string line;
    char c;
    while (::recv(fd, &c, 1, 0) == 1 && c != '\n') {
        if (c != '\r') line.push_back(c);
    }
    return line;
}

}  // namespace

TEST(lab_upload_uses_anonymous_binary_passive_ftp) {
    std::uint16_t port = 0;
    int server = listenTcp(port);
    std::vector<std::string> commands;
    std::string stored;
    std::thread ftp([&] {
        int control = ::accept(server, NULL, NULL);
        writeLine(control, "220-Fake S1");
        writeLine(control, "220 ready");
        int data = -1;
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
                int stream = ::accept(data, NULL, NULL);
                writeLine(control, "150 sending");
                char buffer[4096];
                ssize_t size;
                while ((size = ::recv(stream, buffer, sizeof(buffer), 0)) > 0) {
                    stored.append(buffer, static_cast<std::size_t>(size));
                }
                ::close(stream);
                ::close(data);
                writeLine(control, "226 done");
            } else if (line == "QUIT") {
                writeLine(control, "221 bye");
                break;
            } else {
                writeLine(control, "200 ok");
            }
        }
        ::close(control);
    });

    LabProgramIdentity identity;
    std::string dsp = buildLabProgram("def start():\n    pass\n", identity);
    std::string digest = uploadLabProgram("127.0.0.1", dsp, DEFAULT_UPLOAD_NAME, 5.0, port);
    ftp.join();
    ::close(server);

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
