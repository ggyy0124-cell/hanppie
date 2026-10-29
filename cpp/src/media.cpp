#include "hanppie/media.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include "md5.hpp"

namespace hanppie {
namespace media {

namespace {

const std::size_t VIDEO_CHUNK_LIMIT = 120;
const std::size_t AUDIO_PACKET_LIMIT = 32;

void sleepSeconds(double seconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(seconds * 1e6)));
}

void appendLe16(Bytes& out, std::size_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

}  // namespace

const Bytes& audioTransferId() {
    static const Bytes value = fromHex("00000100");
    return value;
}

const Bytes& audioPlayPayload() {
    static const Bytes value = fromHex("0b040000010001000001");
    return value;
}

Bytes buildAudioStartPayload(std::size_t packetCount, std::size_t totalBytes) {
    if (packetCount < 1 || packetCount > 0xFFFF) {
        throw std::invalid_argument("packetCount must be between 1 and 65535");
    }
    if (totalBytes < 1 || totalBytes > 0xFFFF) {
        throw std::invalid_argument("totalBytes must be between 1 and 65535");
    }
    Bytes payload(1, 0x00);
    payload.insert(payload.end(), audioTransferId().begin(), audioTransferId().end());
    appendLe16(payload, packetCount);
    appendLe16(payload, totalBytes);
    payload.insert(payload.end(), 8, 0);
    return payload;
}

Bytes buildAudioStopPayload(const Bytes& encoded) {
    Bytes payload(1, 0x02);
    Bytes digest = md5(encoded.data(), encoded.size());
    payload.insert(payload.end(), digest.begin(), digest.end());
    return payload;
}

Bytes buildAudioBlock(const Bytes& payload, std::size_t index) {
    if (index > 0xFFFF) throw std::invalid_argument("audio block index must be between 0 and 65535");
    Bytes block;
    block.push_back(static_cast<std::uint8_t>((index >> 8) & 0xFF));
    block.push_back(static_cast<std::uint8_t>(index & 0xFF));
    block.insert(block.end(), 3, 0);
    appendLe16(block, payload.size());
    block.insert(block.end(), payload.begin(), payload.end());
    return block;
}

Bytes packOpusPackets(const std::vector<Bytes>& packets) {
    Bytes encoded;
    for (std::size_t i = 0; i < packets.size(); ++i) {
        if (packets[i].size() > 0xFFFF) throw std::invalid_argument("Opus packet is too large");
        appendLe16(encoded, packets[i].size());
        encoded.insert(encoded.end(), packets[i].begin(), packets[i].end());
    }
    return encoded;
}

Audio::Audio(AppConnection& connection) : connection_(connection) {}

std::size_t Audio::playOpus(const Bytes& encoded) {
    if (encoded.empty()) return 0;
    std::vector<Bytes> chunks;
    for (std::size_t offset = 0; offset < encoded.size(); offset += AUDIO_TRANSFER_CHUNK_BYTES) {
        std::size_t end = offset + AUDIO_TRANSFER_CHUNK_BYTES;
        if (end > encoded.size()) end = encoded.size();
        chunks.push_back(Bytes(encoded.begin() + offset, encoded.begin() + end));
    }
    connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV, protocol::ATTR_NEED_ACK,
                         protocol::CMDSET_RM, protocol::CMD_RM_AUDIO_TRANSFER,
                         buildAudioStartPayload(chunks.size(), encoded.size()));
    sleepSeconds(AUDIO_START_DELAY_SECONDS);
    for (std::size_t index = 0; index < chunks.size(); ++index) {
        connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV, protocol::ATTR_NO_ACK,
                             protocol::CMDSET_COMMON, protocol::CMD_FW_TRANSMIT,
                             buildAudioBlock(chunks[index], index));
        if (index + 1 < chunks.size()) sleepSeconds(AUDIO_PACKET_INTERVAL_SECONDS);
    }
    sleepSeconds(AUDIO_STOP_DELAY_SECONDS);
    connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV, protocol::ATTR_NEED_ACK,
                         protocol::CMDSET_RM, protocol::CMD_RM_AUDIO_TRANSFER,
                         buildAudioStopPayload(encoded));
    sleepSeconds(AUDIO_PLAY_DELAY_SECONDS);
    connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV, protocol::ATTR_NEED_ACK,
                         protocol::CMDSET_RM, protocol::CMD_RM_PLAY_SOUND_TASK, audioPlayPayload());
    return chunks.size();
}

Camera::Camera(AppConnection& connection)
    : connection_(connection), streaming_(false), audioStreaming_(false) {
    videoListener_ = connection_.onVideo([this](const Bytes& payload) { acceptChunk(payload); });
    audioListener_ = connection_.onAudio([this](const Bytes& payload) { acceptAudio(payload); });
}

Camera::~Camera() {
    connection_.removeListener(videoListener_);
    connection_.removeListener(audioListener_);
}

void Camera::acceptChunk(const Bytes& payload) {
    std::lock_guard<std::mutex> lock(videoLock_);
    // A full queue means the consumer fell behind; restart from fresh data.
    if (chunks_.size() >= VIDEO_CHUNK_LIMIT) chunks_.clear();
    chunks_.push_back(payload);
    videoCondition_.notify_one();
}

void Camera::acceptAudio(const Bytes& payload) {
    if (!audioStreaming_) return;
    std::lock_guard<std::mutex> lock(audioLock_);
    if (audioPackets_.size() >= AUDIO_PACKET_LIMIT) audioPackets_.pop_front();
    audioPackets_.push_back(payload);
    audioCondition_.notify_one();
}

bool Camera::startVideoStream(const std::string& resolution) {
    std::string key;
    for (std::size_t i = 0; i < resolution.size(); ++i) {
        char c = resolution[i];
        key.push_back(c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c);
    }
    Bytes format;
    if (key == "720p" || key == "720") {
        format = fromHex("0403000000");
    } else if (key == "1080p" || key == "1080") {
        format = fromHex("0a03000000");
    } else {
        throw std::invalid_argument("resolution must be 720p or 1080p");
    }
    {
        std::lock_guard<std::mutex> lock(videoLock_);
        chunks_.clear();
    }
    connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_CAMERA, protocol::ATTR_NEED_ACK,
                         protocol::CMDSET_CAMERA, protocol::CMD_SET_VIDEO_FORMAT, format);
    streamControl(1, 1, 0);
    streamControl(2, 1, 0);
    streaming_ = true;
    return true;
}

bool Camera::stopVideoStream() {
    bool wasStreaming = streaming_.exchange(false);
    if (wasStreaming) {
        streamControl(2, 0, 0);
        streamControl(1, 0, 0);
    }
    return true;
}

bool Camera::startAudioStream() {
    {
        std::lock_guard<std::mutex> lock(audioLock_);
        audioPackets_.clear();
    }
    audioStreaming_ = true;
    connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_CAMERA, protocol::ATTR_NEED_ACK,
                         protocol::CMDSET_RM, protocol::CMD_RM_SET_AUDIO_STATUS, Bytes(1, 0x01));
    return true;
}

bool Camera::stopAudioStream() {
    audioStreaming_ = false;
    std::lock_guard<std::mutex> lock(audioLock_);
    audioPackets_.clear();
    return true;
}

bool Camera::pop(std::mutex& lock, std::condition_variable& condition, std::deque<Bytes>& queue,
                 Bytes& out, double timeoutSeconds) {
    std::unique_lock<std::mutex> guard(lock);
    if (!condition.wait_for(guard,
                            std::chrono::duration<double>(timeoutSeconds > 0 ? timeoutSeconds : 0),
                            [&queue] { return !queue.empty(); })) {
        return false;
    }
    out = queue.front();
    queue.pop_front();
    return true;
}

bool Camera::readVideoChunk(Bytes& out, double timeoutSeconds) {
    return pop(videoLock_, videoCondition_, chunks_, out, timeoutSeconds);
}

bool Camera::readAudioOpus(Bytes& out, double timeoutSeconds) {
    return pop(audioLock_, audioCondition_, audioPackets_, out, timeoutSeconds);
}

void Camera::close() {
    try {
        stopVideoStream();
    } catch (const std::runtime_error&) {
    }
    stopAudioStream();
}

void Camera::streamControl(std::uint8_t control, std::uint8_t state, std::uint8_t resolution) {
    Bytes payload;
    payload.push_back(control);
    payload.push_back(static_cast<std::uint8_t>(state & 0x0F));
    payload.push_back(resolution);
    connection_.sendDuss(protocol::HOST_MOBILE, protocol::HOST_CAMERA, protocol::ATTR_NEED_ACK,
                         protocol::CMDSET_RM, protocol::CMD_RM_STREAM_CTRL, payload);
}

}  // namespace media
}  // namespace hanppie
