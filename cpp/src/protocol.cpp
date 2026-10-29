#include "hanppie/protocol.hpp"

#include <cstdio>
#include <random>
#include <stdexcept>

namespace hanppie {

namespace {

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

std::uint16_t readLe16(const Bytes& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset] | (data[offset + 1] << 8));
}

void appendLe16(Bytes& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

std::mt19937& randomEngine() {
    static std::mt19937 engine{std::random_device{}()};
    return engine;
}

}  // namespace

Bytes fromHex(const std::string& hex) {
    if (hex.size() % 2 != 0) throw std::invalid_argument("hex string must have an even length");
    Bytes out;
    out.reserve(hex.size() / 2);
    for (std::size_t i = 0; i < hex.size(); i += 2) {
        int high = hexValue(hex[i]);
        int low = hexValue(hex[i + 1]);
        if (high < 0 || low < 0) throw std::invalid_argument("invalid hex character");
        out.push_back(static_cast<std::uint8_t>((high << 4) | low));
    }
    return out;
}

std::string toHex(const Bytes& data) {
    static const char digits[] = "0123456789abcdef";
    std::string out;
    out.reserve(data.size() * 2);
    for (std::size_t i = 0; i < data.size(); ++i) {
        out.push_back(digits[data[i] >> 4]);
        out.push_back(digits[data[i] & 0x0F]);
    }
    return out;
}

namespace protocol {

const char* const MODE_NORMAL = "000300";
const char* const MODE_LAB = "020302";
const char* const MODE_REMOTE = "0b0300";
const char* const MODE_EXIT_PREF = "010300";
const char* const GAME_STATE_SYNC_LAB_PARAMS =
    "05000000ea03000000000000ef0300000a000000f003000000000000"
    "f1030000b80b0000f2030000dc0500000000000000000000";

namespace {

const char* const PRECONNECT_TEMPLATE =
    "3080dc6800000004d84664006400c005140000640064006400c005140000"
    "640014006400c00514000064000101040102";

const std::uint8_t CRC8_TABLE[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20,
    0xA3, 0xFD, 0x1F, 0x41, 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1,
    0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E,
    0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07, 0xDB, 0x85, 0x67, 0x39,
    0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45,
    0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E,
    0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31,
    0xB2, 0xEC, 0x0E, 0x50, 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0,
    0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA,
    0x69, 0x37, 0xD5, 0x8B, 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, 0xE9, 0xB7, 0x55, 0x0B,
    0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54,
    0xD7, 0x89, 0x6B, 0x35,};

const std::uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,};

SetupEntry entry(const char* kind, std::uint8_t sender, std::uint8_t receiver, std::uint8_t attr,
                 std::uint8_t cmdset, std::uint8_t cmdid, const std::string& payloadHex,
                 const std::string& flagsHex) {
    SetupEntry e;
    e.kind = kind;
    e.sender = sender;
    e.receiver = receiver;
    e.attr = attr;
    e.cmdset = cmdset;
    e.cmdid = cmdid;
    e.payload = fromHex(payloadHex);
    e.flags = fromHex(flagsHex);
    return e;
}

std::vector<SetupEntry> buildAppConnectionSetup() {
    const std::string neutral = toHex(neutralControl());
    const std::string vbusMsg =
        "02010000059f22626809000200c49ac5c409000200fd7b4c7809000200"
        "ceceb7ee090002009c00a449090002000100";
    std::vector<SetupEntry> s;
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_DEVICE_VERSION, "", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_RM,
                      CMD_RM_PRODUCT_ATTRIBUTE_GET, "00", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "0100000000ffffffff", "4036"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "01d4030000ffffffff", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_WIFI, ATTR_NEED_ACK, CMDSET_WIFI,
                      CMD_WIFI_AP_SET_COUNTRY_CODE, "4a5000004a5000000100", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "01a8070000ffffffff", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "017c0b0000ffffffff", "0000"));
    s.push_back(entry("control", HOST_MOBILE, HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_SPECIAL,
                      CMD_SPECIAL_RM_CONTROL, neutral, "c8ec"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "01500f0000ffffffff", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "0124130000ffffffff", "6000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "01f8160000ffffffff", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_SYSTEM, ATTR_NEED_ACK, CMDSET_COMMON,
                      CMD_GET_CFG_FILE, "01cc1a0000ffffffff", "0000"));
    for (int i = 0; i < 3; ++i) {
        s.push_back(entry("control", HOST_MOBILE, HOST_HDVT_UAV, ATTR_NO_ACK, CMDSET_SPECIAL,
                          CMD_SPECIAL_RM_CONTROL, neutral, "f0ec"));
    }
    s.push_back(entry("direct", HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS,
                      CMD_VBUS_ADD_NODE, "0200000003", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS,
                      CMD_VBUS_DEL_MSG, "000201", "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS,
                      CMD_VBUS_ADD_MSG, vbusMsg, "0000"));
    s.push_back(entry("direct", HOST_MOBILE, HOST_HDVT_UAV, ATTR_NEED_ACK, CMDSET_VIRTUAL_BUS,
                      CMD_VBUS_ADD_MSG, vbusMsg, "0000"));
    return s;
}

}  // namespace

const Bytes& neutralControl() {
    static const Bytes value = fromHex("0000042000010840000210");
    return value;
}

const std::vector<SetupEntry>& appConnectionSetup() {
    static const std::vector<SetupEntry> value = buildAppConnectionSetup();
    return value;
}

std::string normalizeAppid(const std::string& value) {
    std::size_t begin = value.find_first_not_of(" \t\r\n");
    std::size_t end = value.find_last_not_of(" \t\r\n");
    std::string appid = begin == std::string::npos ? "" : value.substr(begin, end - begin + 1);
    for (std::size_t i = 0; i < appid.size(); ++i) {
        if (appid[i] >= 'A' && appid[i] <= 'Z') appid[i] = static_cast<char>(appid[i] - 'A' + 'a');
    }
    bool valid = appid.size() == 8;
    for (std::size_t i = 0; valid && i < appid.size(); ++i) {
        valid = (appid[i] >= '0' && appid[i] <= '9') || (appid[i] >= 'a' && appid[i] <= 'f');
    }
    if (!valid) throw std::invalid_argument("AppID must be exactly 8 hexadecimal characters");
    return appid;
}

std::uint8_t crc8(const std::uint8_t* data, std::size_t size, std::uint8_t init) {
    std::uint8_t crc = init;
    for (std::size_t i = 0; i < size; ++i) crc = CRC8_TABLE[crc ^ data[i]];
    return crc;
}

std::uint8_t crc8(const Bytes& data, std::uint8_t init) {
    return crc8(data.data(), data.size(), init);
}

std::uint16_t crc16(const std::uint8_t* data, std::size_t size, std::uint16_t init) {
    std::uint16_t crc = init;
    for (std::size_t i = 0; i < size; ++i) {
        crc = static_cast<std::uint16_t>(((crc >> 8) & 0xFF) ^ CRC16_TABLE[(crc ^ data[i]) & 0xFF]);
    }
    return crc;
}

std::uint16_t crc16(const Bytes& data, std::uint16_t init) {
    return crc16(data.data(), data.size(), init);
}

std::uint8_t xorHeader(const std::uint8_t* data, std::size_t size) {
    std::uint8_t result = 0;
    for (std::size_t i = 0; i < size; ++i) result ^= data[i];
    return result;
}

Bytes newSession() {
    std::uniform_int_distribution<int> dist(1, 0xFFFF);
    Bytes out;
    appendLe16(out, static_cast<std::uint16_t>(dist(randomEngine())));
    return out;
}

Bytes nextSession(const Bytes& session) {
    if (session.size() != 2) throw std::invalid_argument("session must contain exactly two bytes");
    std::uint16_t value = static_cast<std::uint16_t>(readLe16(session, 0) + 1);
    Bytes out;
    appendLe16(out, value ? value : 1);
    return out;
}

std::uint16_t newTick() {
    std::uniform_int_distribution<int> dist(0, 0x2000 - 1);
    return static_cast<std::uint16_t>(dist(randomEngine()) * 8);
}

Bytes buildDuss(std::uint8_t sender, std::uint8_t receiver, std::uint8_t attr, std::uint8_t cmdset,
                std::uint8_t cmdid, const Bytes& payload, std::uint16_t sequence) {
    std::size_t length = 13 + payload.size();
    if (length > 0x3FF) {
        char message[64];
        std::snprintf(message, sizeof(message), "DUSS frame length out of range: %u",
                      static_cast<unsigned>(length));
        throw std::invalid_argument(message);
    }
    Bytes frame;
    frame.reserve(length);
    frame.push_back(DUSS_MAGIC);
    frame.push_back(static_cast<std::uint8_t>(length & 0xFF));
    frame.push_back(static_cast<std::uint8_t>(0x04 | ((length >> 8) & 0x03)));
    frame.push_back(0);
    frame.push_back(sender);
    frame.push_back(receiver);
    appendLe16(frame, sequence);
    frame.push_back(attr);
    frame.push_back(cmdset);
    frame.push_back(cmdid);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame[3] = crc8(frame.data(), 3);
    appendLe16(frame, crc16(frame));
    return frame;
}

std::vector<DussFrame> parseDussFrames(const Bytes& data) {
    std::vector<DussFrame> frames;
    std::size_t offset = 0;
    while (data.size() >= 13 && offset <= data.size() - 13) {
        if (data[offset] != DUSS_MAGIC || (data[offset + 2] & 0xFC) != 0x04) {
            ++offset;
            continue;
        }
        std::size_t length = data[offset + 1] | ((data[offset + 2] & 0x03) << 8);
        if (length < 13 || offset + length > data.size()) {
            ++offset;
            continue;
        }
        const std::uint8_t* raw = data.data() + offset;
        DussFrame frame;
        frame.offset = offset;
        frame.sender = raw[4];
        frame.receiver = raw[5];
        frame.sequence = static_cast<std::uint16_t>(raw[6] | (raw[7] << 8));
        frame.attr = raw[8];
        frame.cmdset = raw[9];
        frame.cmdid = raw[10];
        frame.payload.assign(raw + 11, raw + length - 2);
        frame.valid = crc8(raw, 3) == raw[3] &&
                      crc16(raw, length - 2) ==
                          static_cast<std::uint16_t>(raw[length - 2] | (raw[length - 1] << 8));
        frames.push_back(frame);
        // A false marker in the outer header must not skip an overlapping valid frame.
        offset += frame.valid ? length : 1;
    }
    return frames;
}

bool parseRobotBroadcast(const Bytes& data, RobotBroadcast& out) {
    if (data.size() != 24) return false;
    std::uint8_t key = 7;
    Bytes decoded;
    decoded.reserve(24);
    for (std::size_t i = 0; i < data.size(); ++i) {
        decoded.push_back(static_cast<std::uint8_t>(data[i] ^ key));
        key = static_cast<std::uint8_t>(((key + 7) ^ 178) & 0xFF);
    }
    if (decoded[0] != 0x5A || decoded[1] != 0x5B) return false;
    bool zero = true;
    for (std::size_t i = 16; i < 24; ++i) zero = zero && decoded[i] == 0;
    std::string appid;
    if (zero) {
        appid = "00000000";
    } else {
        for (std::size_t i = 16; i < 24; ++i) {
            appid.push_back(decoded[i] < 0x80 ? static_cast<char>(decoded[i]) : '?');
        }
    }
    char ip[16];
    std::snprintf(ip, sizeof(ip), "%u.%u.%u.%u", decoded[6], decoded[7], decoded[8], decoded[9]);
    char mac[18];
    std::snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", decoded[10], decoded[11],
                  decoded[12], decoded[13], decoded[14], decoded[15]);
    out.robotIp = ip;
    out.robotMac = mac;
    out.appid = appid;
    out.pairing = (decoded[2] & 1) != 0;
    return true;
}

bool isVideoPacket(const Bytes& data, const Bytes& session) {
    return data.size() > 20 && session.size() == 2 && data[2] == session[0] &&
           data[3] == session[1] && !(data[2] == 0 && data[3] == 0) && data[6] == 0x02;
}

AppEnvelope::AppEnvelope() { init(newSession(), newTick()); }

AppEnvelope::AppEnvelope(const Bytes& session, std::uint16_t tick) {
    init(session.empty() ? newSession() : session, tick);
}

void AppEnvelope::init(const Bytes& s, std::uint16_t t) {
    session = s;
    tick = t;
    directTick = static_cast<std::uint16_t>(tick + 8);
    directReference = tick;
    latestDirect = tick;
    controlTick = static_cast<std::uint16_t>(tick + 0xA8);
    hasControlReference = false;
    controlReference = 0;
    packetIndex = 1;
    receivedAhead_.clear();
}

Bytes AppEnvelope::preconnect() const {
    Bytes packet = fromHex(PRECONNECT_TEMPLATE);
    packet[2] = session[0];
    packet[3] = session[1];
    packet[7] = xorHeader(packet.data(), 7);
    packet[8] = static_cast<std::uint8_t>(tick & 0xFF);
    packet[9] = static_cast<std::uint8_t>(tick >> 8);
    return packet;
}

Bytes AppEnvelope::wrapDirect(const Bytes& duss, const Bytes& flags) {
    std::uint16_t current = directTick;
    directTick = static_cast<std::uint16_t>(directTick + 8);
    std::size_t total = 20 + duss.size();
    Bytes packet;
    packet.reserve(total);
    packet.push_back(static_cast<std::uint8_t>(total & 0xFF));
    packet.push_back(static_cast<std::uint8_t>(0x80 | ((total >> 8) & 0x03)));
    packet.insert(packet.end(), session.begin(), session.end());
    appendLe16(packet, current);
    packet.push_back(0x05);
    packet.push_back(0x00);
    packet[7] = xorHeader(packet.data(), 7);
    appendLe16(packet, directReference);
    appendLe16(packet, current);
    packet.insert(packet.end(), 4, 0);
    packet.push_back(packetIndex);
    packet.push_back(0x01);
    packet.push_back(flags.size() > 0 ? flags[0] : 0);
    packet.push_back(flags.size() > 1 ? flags[1] : 0);
    latestDirect = current;
    packetIndex = static_cast<std::uint8_t>(packetIndex + 1);
    packet.insert(packet.end(), duss.begin(), duss.end());
    return packet;
}

Bytes AppEnvelope::wrapControl(const Bytes& duss) {
    std::uint16_t current = controlTick;
    controlTick = static_cast<std::uint16_t>(controlTick + 8);
    std::size_t total = 34 + duss.size();
    Bytes packet;
    packet.reserve(total);
    packet.push_back(static_cast<std::uint8_t>(total & 0xFF));
    packet.push_back(0x80);
    packet.insert(packet.end(), session.begin(), session.end());
    packet.push_back(0x00);
    packet.push_back(0x00);
    packet.push_back(0x04);
    packet.push_back(0x00);
    packet[7] = xorHeader(packet.data(), 7);
    appendLe16(packet, current);
    appendLe16(packet, current);
    packet.insert(packet.end(), 4, 0);
    std::uint16_t received = hasControlReference ? controlReference : tick;
    appendLe16(packet, received);
    appendLe16(packet, received);
    packet.insert(packet.end(), 4, 0);
    appendLe16(packet, directReference);
    appendLe16(packet, latestDirect);
    packet.insert(packet.end(), 4, 0);
    appendLe16(packet, static_cast<std::uint16_t>(duss.size()));
    packet.insert(packet.end(), duss.begin(), duss.end());
    return packet;
}

void AppEnvelope::observe(const Bytes& data) {
    if (data.size() < 12 || data[2] != session[0] || data[3] != session[1]) return;
    if (data[6] == 3) {
        std::uint16_t head = hasControlReference ? controlReference : readLe16(data, 8);
        std::uint16_t current = readLe16(data, 4);
        std::uint16_t distance = static_cast<std::uint16_t>(current - head);
        if (distance >= 8 && distance < 0x8000 && distance % 8 == 0) receivedAhead_.insert(current);
        for (;;) {
            std::uint16_t following = static_cast<std::uint16_t>(head + 8);
            std::set<std::uint16_t>::iterator found = receivedAhead_.find(following);
            if (found == receivedAhead_.end()) break;
            receivedAhead_.erase(found);
            head = following;
        }
        hasControlReference = true;
        controlReference = head;
        return;
    }
    if (data[6] != 1 || data.size() < 28 || data[4] != 0 || data[5] != 0) return;
    std::uint16_t direct = readLe16(data, 24);
    std::uint16_t control = readLe16(data, 16);
    if (direct) directReference = direct;
    if (!hasControlReference && control) {
        hasControlReference = true;
        controlReference = control;
    }
    // Stream packets carry the robot's current control window at bytes
    // 10..12. Following it prevents a long-running media session from
    // drifting outside the window used by subsequent control packets.
    if (data.size() != 34) {
        std::vector<DussFrame> frames = parseDussFrames(data);
        for (std::size_t i = 0; i < frames.size(); ++i) {
            if (frames[i].valid && frames[i].offset >= 34) {
                std::uint16_t controlValue = readLe16(data, 10);
                if (controlValue) controlTick = controlValue;
                break;
            }
        }
    }
}

}  // namespace protocol
}  // namespace hanppie
