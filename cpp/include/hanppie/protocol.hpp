// Binary protocol primitives for the RoboMaster App-compatible transport.
//
// C++11 port of src/hanppie/protocol.py. The outer envelope and setup sequence
// are interoperability facts recovered from verified S1/App traffic; keep the
// captured bytes identical to the Python implementation.
#ifndef HANPPIE_PROTOCOL_HPP
#define HANPPIE_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace hanppie {

typedef std::vector<std::uint8_t> Bytes;

Bytes fromHex(const std::string& hex);
std::string toHex(const Bytes& data);

namespace protocol {

const std::uint16_t APP_PORT = 45678;
const std::uint16_t ROBOT_APP_PORT = 56789;
const std::uint16_t LOCAL_CONTROL_PORT = 10609;
const std::uint16_t ROBOT_CONTROL_PORT = 10607;
const std::uint16_t ROBOT_FTP_PORT = 21;
const std::uint16_t INITIAL_DUSS_SEQUENCE = 10072;
const std::uint8_t DUSS_MAGIC = 0x55;
const std::uint8_t CRC8_INIT = 0x77;
const std::uint16_t CRC16_INIT = 0x3692;

// Host / Node IDs (host2byte: (index << 5) | (host & 0x1F))
const std::uint8_t HOST_CAMERA = 0x01;
const std::uint8_t HOST_MOBILE = 0x02;
const std::uint8_t HOST_CHASSIS_CAN = 0x03;
const std::uint8_t HOST_GIMBAL = 0x04;
const std::uint8_t HOST_WIFI = 0x07;
const std::uint8_t HOST_HDVT_UAV = 0x09;
const std::uint8_t HOST_GUN = 0x17;
const std::uint8_t HOST_SYSTEM = 0x28;
const std::uint8_t HOST_SCRATCH_CLIENT = 0x42;
const std::uint8_t HOST_SCRATCH_SYS = 0xA9;
const std::uint8_t HOST_CHASSIS = 0xC3;
const std::uint8_t HOST_SCRATCH_SCRIPT = 0xC9;
const std::uint8_t HOST_VISION = 0xF1;

// Attr
const std::uint8_t ATTR_NO_ACK = 0x00;
const std::uint8_t ATTR_NEED_ACK_NO_FINISH = 0x20;
const std::uint8_t ATTR_NEED_ACK = 0x40;
const std::uint8_t ATTR_ACK = 0x80;
const std::uint8_t ATTR_RESP_NEED_ACK = 0xC0;

// CmdSet
const std::uint8_t CMDSET_COMMON = 0x00;
const std::uint8_t CMDSET_SPECIAL = 0x01;
const std::uint8_t CMDSET_CAMERA = 0x02;
const std::uint8_t CMDSET_FC = 0x03;
const std::uint8_t CMDSET_GIMBAL = 0x04;
const std::uint8_t CMDSET_WIFI = 0x07;
const std::uint8_t CMDSET_DM368 = 0x08;
const std::uint8_t CMDSET_HDVT = 0x09;
const std::uint8_t CMDSET_VISION = 0x0A;
const std::uint8_t CMDSET_RM = 0x3F;
const std::uint8_t CMDSET_VIRTUAL_BUS = 0x48;

// Common (cmdset 0x00)
const std::uint8_t CMD_GET_DEVICE_VERSION = 0x01;
const std::uint8_t CMD_FW_TRANSMIT = 0x09;
const std::uint8_t CMD_GET_CFG_FILE = 0x4F;

// Special (cmdset 0x01)
const std::uint8_t CMD_SPECIAL_RM_CONTROL = 0x04;

// Camera (cmdset 0x02)
const std::uint8_t CMD_CAPTURE = 0x01;
const std::uint8_t CMD_SET_VIDEO_FORMAT = 0x18;
const std::uint8_t CMD_SET_ZOOM_PARAM = 0x34;

// Gimbal (cmdset 0x04)
const std::uint8_t CMD_GIMBAL_EXT_CTRL_ACCEL = 0x0C;
const std::uint8_t CMD_GIMBAL_ROTATE_SPEED = 0x69;

// Wi-Fi (cmdset 0x07)
const std::uint8_t CMD_WIFI_AP_PUSH_RSSI = 0x09;
const std::uint8_t CMD_WIFI_AP_KEEPALIVE = 0x17;
const std::uint8_t CMD_WIFI_AP_SET_COUNTRY_CODE = 0x30;
const std::uint8_t CMD_WIFI_GET_WORK_MODE = 0x39;
const std::uint8_t CMD_WIFI_CONFIG_BY_QRCODE = 0x3B;

// RM (cmdset 0x3F)
const std::uint8_t CMD_RM_SPECIAL_CONTROL = 0x04;
const std::uint8_t CMD_RM_GAME_STATE_SYNC = 0x09;
const std::uint8_t CMD_RM_GAMECTRL_CMD = 0x0A;
const std::uint8_t CMD_RM_MODULE_STATUS_PUSH = 0x12;
const std::uint8_t CMD_RM_WORK_MODE_SET = 0x19;
const std::uint8_t CMD_RM_PLAY_SOUND = 0x1A;
const std::uint8_t CMD_RM_SET_SPEAKER_VOLUME = 0x1B;
const std::uint8_t CMD_RM_GET_SPEAKER_VOLUME = 0x1C;
const std::uint8_t CMD_RM_AUDIO_TO_APP = 0x1D;
const std::uint8_t CMD_RM_SET_AUDIO_STATUS = 0x1E;
const std::uint8_t CMD_RM_WHEEL_SPEED_SET = 0x20;
const std::uint8_t CMD_RM_SPEED_SET = 0x21;
const std::uint8_t CMD_RM_SPEED_MODE_SET = 0x28;
const std::uint8_t CMD_RM_LED_COLOR_SET = 0x33;
const std::uint8_t CMD_RM_EXIT_LOW_POWER_MODE = 0x4C;
const std::uint8_t CMD_RM_SHOOT_CMD = 0x51;
const std::uint8_t CMD_RM_GUN_LED_SET = 0x55;
const std::uint8_t CMD_RM_GET_SIGHT_BEAD_POSITION = 0x57;
const std::uint8_t CMD_RM_SYSTEM_FUNCTION_CONFIG = 0x59;
// Exported spelling retained: 00 requests barrel cooling; 01 resets it.
const std::uint8_t CMD_RM_SYSTEM_STATUS_CONFIG = 0x5B;
const std::uint8_t CMD_RM_AUDIO_TRANSFER = 0x5F;
const std::uint8_t CMD_RM_FC_RMC = 0x66;
const std::uint8_t CMD_RM_SAVE_PREF = 0x77;
const std::uint8_t CMD_RM_SCRIPT_DOWNLOAD_DATA = 0xA1;
const std::uint8_t CMD_RM_SCRIPT_DOWNLOAD_FINSH = 0xA2;
const std::uint8_t CMD_RM_SCRIPT_CTRL = 0xA3;
const std::uint8_t CMD_RM_SCRIPT_CUSTOM_INFO_PUSH = 0xA4;
const std::uint8_t CMD_RM_SCRIPT_BLOCK_STATUS_PUSH = 0xA5;
const std::uint8_t CMD_RM_SUB_MOBILE_INFO = 0xAB;
const std::uint8_t CMD_RM_PLAY_SOUND_TASK = 0xB3;
const std::uint8_t CMD_RM_CUSTOM_UI_ATTRIBUTE_SET = 0xBA;
const std::uint8_t CMD_RM_STREAM_CTRL = 0xD2;
const std::uint8_t CMD_RM_PRODUCT_ATTRIBUTE_GET = 0xFE;

// Virtual Bus (cmdset 0x48)
const std::uint8_t CMD_VBUS_ADD_NODE = 0x01;
const std::uint8_t CMD_VBUS_NODE_RESET = 0x02;
const std::uint8_t CMD_VBUS_ADD_MSG = 0x03;
const std::uint8_t CMD_VBUS_DEL_MSG = 0x04;
const std::uint8_t CMD_VBUS_DATA_ANALYSIS = 0x08;

// Vision (cmdset 0x0A)
const std::uint8_t CMD_VISION_CUSTOM = 0xA3;

// System mode payloads (hex)
extern const char* const MODE_NORMAL;
extern const char* const MODE_LAB;
extern const char* const MODE_REMOTE;
extern const char* const MODE_EXIT_PREF;

// SCRIPT_CTRL markers (cmd 0xA3)
const std::uint8_t SCRIPT_CTRL_METADATA_FULL = 0x21;
const std::uint8_t SCRIPT_CTRL_METADATA_GUID = 0x2D;
const std::uint8_t SCRIPT_CTRL_START = 0x52;
const std::uint8_t SCRIPT_CTRL_STOP = 0x55;

// Lab game state sync parameters (hex)
extern const char* const GAME_STATE_SYNC_LAB_PARAMS;

// Neutral 50 Hz control-channel payload.
const Bytes& neutralControl();

// One entry of a captured command sequence. kind is "direct" or "control".
struct SetupEntry {
    std::string kind;
    std::uint8_t sender;
    std::uint8_t receiver;
    std::uint8_t attr;
    std::uint8_t cmdset;
    std::uint8_t cmdid;
    Bytes payload;
    Bytes flags;
};

// The minimal App connection setup observed before entering Lab mode.
const std::vector<SetupEntry>& appConnectionSetup();

// Validate and lower-case an 8-character hexadecimal AppID.
std::string normalizeAppid(const std::string& value);

std::uint8_t crc8(const std::uint8_t* data, std::size_t size, std::uint8_t init = CRC8_INIT);
std::uint8_t crc8(const Bytes& data, std::uint8_t init = CRC8_INIT);
std::uint16_t crc16(const std::uint8_t* data, std::size_t size, std::uint16_t init = CRC16_INIT);
std::uint16_t crc16(const Bytes& data, std::uint16_t init = CRC16_INIT);
std::uint8_t xorHeader(const std::uint8_t* data, std::size_t size);

Bytes newSession();
Bytes nextSession(const Bytes& session);
std::uint16_t newTick();

Bytes buildDuss(std::uint8_t sender, std::uint8_t receiver, std::uint8_t attr, std::uint8_t cmdset,
                std::uint8_t cmdid, const Bytes& payload, std::uint16_t sequence);

struct DussFrame {
    std::size_t offset;
    std::uint8_t sender;
    std::uint8_t receiver;
    std::uint16_t sequence;
    std::uint8_t attr;
    std::uint8_t cmdset;
    std::uint8_t cmdid;
    Bytes payload;
    bool valid;
};

std::vector<DussFrame> parseDussFrames(const Bytes& data);

struct RobotBroadcast {
    std::string robotIp;
    std::string robotMac;
    std::string appid;
    bool pairing;
};

// Returns false when data is not a robot broadcast.
bool parseRobotBroadcast(const Bytes& data, RobotBroadcast& out);

bool isVideoPacket(const Bytes& data, const Bytes& session);

// Maintain the dynamic window and sequence fields of the App outer packet.
class AppEnvelope {
public:
    AppEnvelope();
    AppEnvelope(const Bytes& session, std::uint16_t tick);

    Bytes preconnect() const;
    Bytes wrapDirect(const Bytes& duss, const Bytes& flags = Bytes(2, 0));
    Bytes wrapControl(const Bytes& duss);
    void observe(const Bytes& data);

    Bytes session;
    std::uint16_t tick;
    std::uint16_t directTick;
    std::uint16_t directReference;
    std::uint16_t latestDirect;
    std::uint16_t controlTick;
    bool hasControlReference;
    std::uint16_t controlReference;
    std::uint8_t packetIndex;

private:
    void init(const Bytes& session, std::uint16_t tick);
    std::set<std::uint16_t> receivedAhead_;
};

}  // namespace protocol
}  // namespace hanppie

#endif  // HANPPIE_PROTOCOL_HPP
