// Offline unit tests for the Hanppie C++ SDK. Vectors either mirror tests/ in
// the Python package or come from golden_vectors.inc generated from it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "hanppie/hanppie.hpp"
#include "golden.hpp"
#include "test_support.hpp"

using namespace hanppie;
using namespace hanppie::protocol;
using namespace golden;

namespace {


Bytes str(const char* text) { return Bytes(text, text + std::strlen(text)); }

Bytes slice(const Bytes& data, std::size_t begin, std::size_t end) {
    return Bytes(data.begin() + begin, data.begin() + end);
}

std::uint16_t le16(const Bytes& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset] | (data[offset + 1] << 8));
}

void putLe16(Bytes& data, std::size_t offset, std::uint16_t value) {
    data[offset] = static_cast<std::uint8_t>(value & 0xFF);
    data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

}  // namespace

TEST(crc_known_vectors) {
    std::uint8_t header[] = {0x55, 0x0D, 0x04};
    CHECK_EQ(crc8(header, 3), 0x33);
    Bytes frame = buildDuss(0x02, 0x09, 0x40, 0x3F, 0x57, Bytes(), 10072);
    CHECK_EQ(crc16(frame.data(), frame.size() - 2), le16(frame, frame.size() - 2));
}

TEST(appid_validation) {
    CHECK_EQ(normalizeAppid(" B6359877 "), std::string("b6359877"));
    CHECK_THROWS(normalizeAppid("not-an-id"), std::invalid_argument);
}

TEST(duss_frame_matches_captured_crc_vector) {
    Bytes frame = buildDuss(0x02, 0x09, 0x40, 0x3F, 0x57, Bytes(), 10072);
    CHECK_EQ(toHex(frame), std::string("550d043302095827403f573c34"));
    Bytes prefixed = str("prefix");
    prefixed.insert(prefixed.end(), frame.begin(), frame.end());
    std::vector<DussFrame> found = parseDussFrames(prefixed);
    CHECK_EQ(found.size(), 1u);
    CHECK_EQ(found[0].offset, 6u);
    DussFrame decoded = parseDussFrames(frame)[0];
    CHECK(decoded.valid);
    CHECK_EQ(decoded.sender, 0x02);
    CHECK_EQ(decoded.receiver, 0x09);
    CHECK_EQ(decoded.cmdset, 0x3F);
    CHECK_EQ(decoded.cmdid, 0x57);
    CHECK(decoded.payload.empty());
    Bytes damaged = frame;
    damaged.back() ^= 0xFF;
    CHECK(!parseDussFrames(damaged)[0].valid);
    CHECK_THROWS(buildDuss(0, 0, 0, 0, 0, Bytes(0x3FF), 0), std::invalid_argument);
}

TEST(invalid_outer_header_candidate_does_not_hide_real_frame) {
    for (int validHeader = 0; validHeader < 2; ++validHeader) {
        // S1 video transfer: an outer checksum of 0x55 falsely claimed a 64-byte frame.
        Bytes frame = buildDuss(1, 2, 0, 0, 0x27, Bytes(80, 0), 123);
        Bytes data(20, 0);
        data[7] = 0x55;
        data[8] = 0x40;
        data[9] = 0x04;
        if (validHeader) data[10] = crc8(&data[7], 3);
        data.insert(data.end(), frame.begin(), frame.end());
        std::vector<DussFrame> decoded = parseDussFrames(data);
        bool sawInvalid = false;
        std::vector<DussFrame> valid;
        for (std::size_t i = 0; i < decoded.size(); ++i) {
            if (!decoded[i].valid && decoded[i].offset == 7) sawInvalid = true;
            if (decoded[i].valid) valid.push_back(decoded[i]);
        }
        CHECK(sawInvalid);
        CHECK_EQ(valid.size(), 1u);
        CHECK_EQ(valid[0].offset, 20u);
        CHECK_EQ(valid[0].cmdid, 0x27);
        CHECK_EQ(valid[0].sequence, 123);
        CHECK(valid[0].payload == Bytes(80, 0));
    }
}

TEST(valid_frame_payload_is_not_rescanned) {
    Bytes inner = buildDuss(9, 2, 0, 0x3F, 0x70, Bytes(), 5);
    Bytes outer = buildDuss(1, 2, 0, 0, 0x27, inner, 6);
    std::vector<DussFrame> decoded = parseDussFrames(outer);
    CHECK_EQ(decoded.size(), 1u);
    CHECK(decoded[0].valid);
    CHECK_EQ(decoded[0].cmdid, 0x27);
    CHECK(decoded[0].payload == inner);
}

TEST(outer_envelope_matches_captured_preconnect_and_direct_packet) {
    Bytes frame = buildDuss(0x02, 0x09, 0x40, 0x3F, 0x57, Bytes(), 10072);
    AppEnvelope envelope(fromHex("dc68"), 0x46D8);
    CHECK_EQ(toHex(envelope.preconnect()),
             std::string("3080dc6800000004d84664006400c005140000640064006400c005140000"
                         "640014006400c00514000064000101040102"));
    CHECK_EQ(toHex(envelope.wrapDirect(frame)),
             std::string("2180dc68e04605b6d846e0460000000001010000550d043302095827403f573c34"));
    Bytes control = envelope.wrapControl(frame);
    CHECK_EQ(control.size(), 34 + frame.size());
    CHECK(slice(control, 2, 4) == fromHex("dc68"));
    CHECK_EQ(le16(control, 32), frame.size());

    Bytes inbound(28, 0);
    inbound[2] = envelope.session[0];
    inbound[3] = envelope.session[1];
    inbound[6] = 1;
    putLe16(inbound, 16, 123);
    putLe16(inbound, 24, 456);
    envelope.observe(inbound);
    CHECK(envelope.hasControlReference);
    CHECK_EQ(envelope.controlReference, 123);
    CHECK_EQ(envelope.directReference, 456);

    Bytes streamed(34, 0);
    streamed.insert(streamed.end(), frame.begin(), frame.end());
    streamed[2] = envelope.session[0];
    streamed[3] = envelope.session[1];
    streamed[6] = 1;
    putLe16(streamed, 10, 789);
    envelope.observe(streamed);
    CHECK_EQ(envelope.controlTick, 789);
}

namespace {

Bytes reliablePacket(std::uint16_t tick, std::uint16_t base = 8, const Bytes& session = fromHex("3412")) {
    AppEnvelope source(session, base);
    Bytes packet = source.wrapDirect(buildDuss(9, 2, 0, 0, 0x70, Bytes(), 0));
    putLe16(packet, 4, tick);
    packet[6] = 3;
    packet[7] = xorHeader(packet.data(), 7);
    putLe16(packet, 8, base);
    putLe16(packet, 10, tick);
    return packet;
}

}  // namespace

TEST(reliable_receive_window_acknowledges_all_native_packets) {
    AppEnvelope envelope(fromHex("3412"), 80);
    for (int tick = 16; tick < 809; tick += 8) envelope.observe(reliablePacket(tick));
    Bytes control = envelope.wrapControl(Bytes());
    CHECK_EQ(le16(control, 16), 808);
    CHECK_EQ(le16(control, 18), 808);
    CHECK_EQ(le16(control, 24), 80);
    CHECK_EQ(le16(control, 26), 80);
    CHECK_EQ(le16(envelope.wrapDirect(Bytes()), 4), 88);
}

TEST(reliable_receive_window_waits_for_gaps_and_ignores_duplicates) {
    AppEnvelope envelope(fromHex("3412"), 80);
    envelope.observe(reliablePacket(16));
    envelope.observe(reliablePacket(32));
    CHECK_EQ(toHex(slice(envelope.wrapControl(Bytes()), 16, 20)), std::string("10001000"));
    envelope.observe(reliablePacket(24));
    CHECK_EQ(toHex(slice(envelope.wrapControl(Bytes()), 16, 20)), std::string("20002000"));
    envelope.observe(reliablePacket(16));
    envelope.observe(reliablePacket(32));
    envelope.observe(reliablePacket(40, 8, fromHex("7856")));
    CHECK_EQ(toHex(slice(envelope.wrapControl(Bytes()), 16, 20)), std::string("20002000"));
}

TEST(reliable_tick_zero_wraps_without_reading_payload_as_control_table) {
    AppEnvelope envelope(fromHex("3412"), 80);
    Bytes packet = reliablePacket(0, 65528);
    packet[24] = 0xD2;
    packet[25] = 0x04;
    packet[26] = 0x2E;
    packet[27] = 0x16;
    envelope.observe(packet);
    CHECK_EQ(toHex(slice(envelope.wrapControl(Bytes()), 16, 20)), std::string("00000000"));
    envelope.observe(reliablePacket(8, 65528));
    CHECK_EQ(toHex(slice(envelope.wrapControl(Bytes()), 16, 20)), std::string("08000800"));
    CHECK_EQ(envelope.directReference, 80);
}

TEST(control_table_does_not_replace_contiguous_receive_acknowledgement) {
    AppEnvelope envelope(fromHex("3412"), 80);
    for (int tick = 16; tick < 33; tick += 8) envelope.observe(reliablePacket(tick));
    Bytes packet(34, 0);
    packet[2] = envelope.session[0];
    packet[3] = envelope.session[1];
    packet[6] = 1;
    const std::uint16_t references[] = {24, 8, 40};
    for (int i = 0; i < 3; ++i) {
        putLe16(packet, 16, references[i]);
        envelope.observe(packet);
        CHECK_EQ(toHex(slice(envelope.wrapControl(Bytes()), 16, 20)), std::string("20002000"));
    }
}

TEST(session_rollover_and_packet_classification) {
    CHECK(nextSession(fromHex("ffff")) == fromHex("0100"));
    CHECK_THROWS(nextSession(str("x")), std::invalid_argument);
    Bytes session = fromHex("3412");
    Bytes video(24, 0);
    video[2] = 0x34;
    video[3] = 0x12;
    video[6] = 2;
    CHECK(isVideoPacket(video, session));
    CHECK(!isVideoPacket(slice(video, 0, 20), session));
    CHECK(!isVideoPacket(video, fromHex("3512")));
    for (int i = 0; i < 100; ++i) {
        CHECK_EQ(newTick() % 8, 0);
        CHECK(newSession() != fromHex("0000"));
    }
}

TEST(robot_broadcast_decode) {
    Bytes decoded(24, 0);
    decoded[0] = 0x5A;
    decoded[1] = 0x5B;
    decoded[2] = 1;
    decoded[6] = 192;
    decoded[7] = 168;
    decoded[8] = 10;
    decoded[9] = 42;
    Bytes mac = fromHex("001122334455");
    std::copy(mac.begin(), mac.end(), decoded.begin() + 10);
    std::memcpy(&decoded[16], "b6359877", 8);
    RobotBroadcast broadcast;
    CHECK(parseRobotBroadcast(test::encodeBroadcast(decoded), broadcast));
    CHECK_EQ(broadcast.robotIp, std::string("192.168.10.42"));
    CHECK_EQ(broadcast.robotMac, std::string("00:11:22:33:44:55"));
    CHECK_EQ(broadcast.appid, std::string("b6359877"));
    CHECK(broadcast.pairing);
    CHECK(!parseRobotBroadcast(str("short"), broadcast));
}

TEST(setup_sequence_is_the_minimal_app_prefix) {
    const std::vector<SetupEntry>& setup = appConnectionSetup();
    CHECK_EQ(setup.size(), GOLDEN_COUNT(CONNECT_SEQUENCE));
    for (std::size_t i = 0; i < setup.size(); ++i) {
        const GoldenEntry& g = CONNECT_SEQUENCE[i];
        CHECK_EQ(setup[i].kind, std::string(g.kind));
        CHECK_EQ(toHex(setup[i].payload), std::string(g.payload));
        if (setup[i].kind == "direct") {
            CHECK_EQ(setup[i].receiver, g.receiver);
            CHECK_EQ(setup[i].cmdset, g.cmdset);
            CHECK_EQ(setup[i].cmdid, g.cmdid);
            CHECK_EQ(toHex(setup[i].flags), std::string(g.flags));
        }
    }
}

namespace {

DussFrame productFrame(std::uint8_t cmdid, const Bytes& payload, bool valid = true,
                       std::uint8_t attr = 0xC0) {
    DussFrame frame;
    frame.offset = 0;
    frame.sender = 0x28;
    frame.receiver = 0x02;
    frame.sequence = 1;
    frame.attr = attr;
    frame.cmdset = 0x3F;
    frame.cmdid = cmdid;
    frame.payload = payload;
    frame.valid = valid;
    return frame;
}

}  // namespace

TEST(product_type_requires_an_explicit_valid_response) {
    RobotModel model;
    CHECK(decodeModel(productFrame(0xFE, fromHex("0001")), model) && model == RobotModel::ROBOMASTER_S1);
    CHECK(decodeModel(productFrame(0xFE, fromHex("0002")), model) && model == RobotModel::ROBOMASTER_EP);
    CHECK(decodeModel(productFrame(0xFE, fromHex("0007")), model) && model == RobotModel::UNKNOWN);
    CHECK(!decodeModel(productFrame(0xFE, fromHex("00")), model));
    CHECK(!decodeModel(productFrame(0x12, fromHex("0001")), model));
    CHECK(!decodeModel(productFrame(0xFE, fromHex("0001"), false), model));
    CHECK(!decodeModel(productFrame(0xFE, fromHex("0001"), true, 0x80), model));
}

TEST(working_devices_are_independent_from_product_type) {
    RobotCapabilities capabilities;
    CHECK(decodeCapabilities(
        productFrame(0x12, fromHex("0300030134121e03020000fe00777700"), true, 0x00), capabilities));
    std::set<RobotComponent> components = capabilities.components();
    CHECK(components.count(RobotComponent::CHASSIS) == 1);
    CHECK(components.count(RobotComponent::ARM) == 1);
    CHECK(capabilities.workingDevices[0].details == std::vector<std::uint16_t>(1, 0x1234));
    CHECK_EQ(capabilities.workingDevices[1].details.size(), 2u);
    CHECK_EQ(capabilities.workingDevices[1].details[1], 0xFE);
    CHECK(capabilities.unknownDeviceIds() == std::set<std::uint16_t>(std::set<std::uint16_t>{0x7777}));

    RobotProduct current;
    current.model = RobotModel::ROBOMASTER_EP;
    RobotProduct updated;
    CHECK(updatedProduct(current, productFrame(0x12, fromHex("01000300"), true, 0x00), updated));
    CHECK(updated.model == RobotModel::ROBOMASTER_EP);
    CHECK(updated.capabilities.components().count(RobotComponent::CHASSIS) == 1);
    CHECK(!updatedProduct(updated, productFrame(0x12, fromHex("01000300"), true, 0x00), current));
}

TEST(truncated_working_device_push_is_rejected) {
    RobotCapabilities capabilities;
    CHECK(!decodeCapabilities(productFrame(0x12, fromHex("0100030100"), true, 0x00), capabilities));
    CHECK(!decodeCapabilities(productFrame(0x12, Bytes(), true, 0x00), capabilities));
    CHECK(!decodeCapabilities(productFrame(0x12, fromHex("00"), true, 0x40), capabilities));
}

TEST(payload_builders_match_python_reference) {
    for (std::size_t i = 0; i < sizeof(CHASSIS_VECTORS) / sizeof(CHASSIS_VECTORS[0]); ++i) {
        const ChassisVector& v = CHASSIS_VECTORS[i];
        CHECK_EQ(toHex(buildChassisControlPayload(v.x, v.y, v.z)), std::string(v.hex));
    }
    for (std::size_t i = 0; i < sizeof(GIMBAL_VECTORS) / sizeof(GIMBAL_VECTORS[0]); ++i) {
        const GimbalVector& v = GIMBAL_VECTORS[i];
        CHECK_EQ(toHex(buildGimbalSpeedPayload(v.pitch, v.yaw)), std::string(v.hex));
    }
    for (std::size_t i = 0; i < sizeof(LED_VECTORS) / sizeof(LED_VECTORS[0]); ++i) {
        const LedVector& v = LED_VECTORS[i];
        CHECK_EQ(toHex(buildLedPayload(v.component, v.red, v.green, v.blue, v.effect)),
                 std::string(v.hex));
    }
    CHECK_THROWS(buildLedPayload(0x3F, 0, 0, 0, "rainbow"), std::invalid_argument);
}

TEST(robot_decodes_native_telemetry) {
    DussFrame frame = productFrame(0x08, fromHex(ODOMETRY_PAYLOAD));
    frame.cmdset = 0x48;
    Odometry odometry;
    CHECK(decodeOdometry(frame, odometry));
    CHECK_EQ(odometry.batteryPercent, 87);
    CHECK(odometry.headingLike == 1.25f);
    CHECK_EQ(odometry.values.size(), 9u);
    CHECK(odometry.x() == 0.5f && odometry.y() == -1.5f);
    float a, b, c;
    CHECK(odometry.rawMotionValues(a, b, c) && a == 2.0f && b == 3.0f && c == 4.0f);

    DussFrame gimbalFrame = productFrame(0x08, fromHex("000a8403f6ff2c01ceff01"));
    gimbalFrame.cmdset = 0x48;
    GimbalTelemetry gimbal;
    CHECK(decodeGimbal(gimbalFrame, gimbal));
    CHECK(gimbal.groundYawDegrees() == 90.0);
    CHECK(gimbal.groundPitchDegrees() == -1.0);
    CHECK(gimbal.yawDegrees() == 30.0);
    CHECK(gimbal.pitchDegrees() == -5.0);
    CHECK_EQ(gimbal.flag, 1);
    gimbalFrame.payload[1] = 0x0B;
    CHECK(!decodeGimbal(gimbalFrame, gimbal));
}

TEST(audio_metadata_and_block_headers_match_observed_packets) {
    CHECK_EQ(toHex(media::buildAudioStartPayload(5, 4562)),
             std::string("00000001000500d2110000000000000000"));
    CHECK_EQ(toHex(media::buildAudioStopPayload(str("captured-pcm"))),
             std::string("023262e9161cd25bdf6e070a1a14b627c3"));
    CHECK_EQ(toHex(media::buildAudioBlock(str("audio"), 1)),
             std::string("00010000000500617564696f"));
    std::vector<Bytes> packets;
    packets.push_back(fromHex("2801"));
    packets.push_back(fromHex("28"));
    CHECK_EQ(toHex(media::packOpusPackets(packets)), std::string("02002801010028"));
    CHECK_THROWS(media::buildAudioStartPayload(0, 1), std::invalid_argument);
}

TEST(md5_matches_reference_vectors) {
    CHECK_EQ(md5Hex(""), std::string("d41d8cd98f00b204e9800998ecf8427e"));
    CHECK_EQ(md5Hex("The quick brown fox jumps over the lazy dog"),
             std::string("9e107d9d372bb6826bd81d3542a419d6"));
    CHECK_EQ(md5Hex(std::string(1000, 'a')), std::string("cabe45dcc9ae5b66ba86600cca6b8ba8"));
}

TEST(build_lab_program_generates_valid_python_dsp) {
    LabProgramIdentity identity;
    std::string dsp = buildLabProgram("print(\"]]>\")", identity, "Hanppie & Test");
    CHECK_EQ(identity.guid.size(), 32u);
    CHECK_EQ(identity.sign.size(), 16u);
    CHECK_EQ(identity.fullMarker, 0x21);
    CHECK_EQ(identity.guidMarker, 0x2D);
    CHECK(dsp.find("<guid>" + identity.guid + "</guid>") != std::string::npos);
    CHECK(dsp.find("<sign>" + identity.sign + "</sign>") != std::string::npos);
    CHECK(dsp.find("<title>Hanppie &amp; Test</title>") != std::string::npos);
    CHECK(dsp.find("<![CDATA[print(\"]]]]><![CDATA[>\")]]>") != std::string::npos);
    LabProgramIdentity other;
    buildLabProgram("", other);
    CHECK(other.guid != identity.guid);
}

int main() { return test::runAll(); }

TEST(lab_style_ctrl_helpers) {
    CHECK_EQ(ctrl::approachRate(0.5, 30), 0.0);
    CHECK_EQ(ctrl::approachRate(5, 30), 15.0);
    CHECK_EQ(ctrl::approachRate(-100, 30), -30.0);
    CHECK(std::fabs(ctrl::approachRate(1.2, 30) - 3.6) < 1e-9);
    CHECK_EQ(ctrl::approachRate(1.1, 30) > 0, true);
    CHECK_EQ(ctrl::angleDifference(-170, 170), 20.0);
    CHECK_EQ(ctrl::angleDifference(170, -170), -20.0);
    CHECK_EQ(ctrl::angleDifference(180, 0), 180.0);
    std::pair<double, double> forward = ctrl::directionVelocity(0, 0.5);
    CHECK_EQ(forward.first, 0.5);
    CHECK(std::fabs(forward.second) < 1e-12);
    std::pair<double, double> right = ctrl::directionVelocity(90, 0.5);
    CHECK(std::fabs(right.first) < 1e-12);
    CHECK(std::fabs(right.second - 0.5) < 1e-12);
}

TEST(chassis_attitude_topic_matches_robot_core) {
    // robot-core ChassisSubscriptions.addPayload(13, 0x000200096b986306).
    CHECK_EQ(toHex(buildDdsSubscriptionPayload(13, 0x000200096B986306ULL, 10)),
             std::string("020d0000010663986b090002000a00"));
    DussFrame frame;
    frame.valid = true;
    frame.cmdset = CMDSET_VIRTUAL_BUS;
    frame.cmdid = CMD_VBUS_DATA_ANALYSIS;
    // yaw 90.0, pitch -1.5, roll 0.25 as little-endian floats.
    frame.payload = fromHex("000d" "0000b442" "0000c0bf" "0000803e");
    ChassisAttitude attitude;
    CHECK(decodeChassisAttitude(frame, attitude));
    CHECK_EQ(attitude.yawDegrees, 90.0f);
    CHECK_EQ(attitude.pitchDegrees, -1.5f);
    CHECK_EQ(attitude.rollDegrees, 0.25f);
    frame.payload = fromHex("000d" "00004843" "00000000" "00000000");  // yaw 200 is out of range
    CHECK(!decodeChassisAttitude(frame, attitude));
    frame.payload[1] = 14;
    CHECK(!decodeChassisAttitude(frame, attitude));
}
