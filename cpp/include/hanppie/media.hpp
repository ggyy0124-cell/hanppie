// RoboMaster App audio/video stream control, currently verified on S1.
// C++11 port of src/hanppie/media/{audio,camera}.py.
//
// Codec work is left to the caller so the SDK stays dependency-free: video is
// delivered as raw H.264 chunks, microphone audio as Opus packets, and speaker
// uploads take Opus packets encoded at 12 kHz mono (see packOpusPackets).
#ifndef HANPPIE_MEDIA_HPP
#define HANPPIE_MEDIA_HPP

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "hanppie/connection.hpp"

namespace hanppie {
namespace media {

const int MICROPHONE_SAMPLE_RATE = 48000;
const int MICROPHONE_CHANNELS = 1;
const int MICROPHONE_SAMPLE_WIDTH = 2;
const int SPEAKER_SAMPLE_RATE = 12000;
const int SPEAKER_SAMPLE_WIDTH = 2;
const int SPEAKER_FRAME_SAMPLES = 240;
const int SPEAKER_FRAME_BYTES = SPEAKER_FRAME_SAMPLES * SPEAKER_SAMPLE_WIDTH;
const std::size_t AUDIO_TRANSFER_CHUNK_BYTES = 960;
const double AUDIO_PACKET_INTERVAL_SECONDS = 0.006;
const double AUDIO_START_DELAY_SECONDS = 0.055;
const double AUDIO_STOP_DELAY_SECONDS = 0.055;
const double AUDIO_PLAY_DELAY_SECONDS = 0.107;

const Bytes& audioTransferId();
const Bytes& audioPlayPayload();

// Build the metadata command that starts one encoded audio upload.
Bytes buildAudioStartPayload(std::size_t packetCount, std::size_t totalBytes);
// Build the commit command for one encoded audio upload.
Bytes buildAudioStopPayload(const Bytes& encoded);
// Wrap one encoded transfer chunk in the S1-verified audio block header.
Bytes buildAudioBlock(const Bytes& payload, std::size_t index);
// Length-prefix Opus packets (12 kHz mono, 20 ms frames) into the upload format.
Bytes packOpusPackets(const std::vector<Bytes>& packets);

// Upload Opus audio through the S1-verified speaker route.
class Audio {
public:
    explicit Audio(AppConnection& connection);

    // Upload, commit, and play one length-prefixed Opus clip; return packet count.
    std::size_t playOpus(const Bytes& encoded);
    void close() {}

private:
    AppConnection& connection_;
};

class Camera {
public:
    explicit Camera(AppConnection& connection);
    ~Camera();

    // resolution is "720p" or "1080p".
    bool startVideoStream(const std::string& resolution = "720p");
    bool stopVideoStream();
    // Request the robot microphone's Opus stream over the App session.
    bool startAudioStream();
    // Stop accepting microphone packets locally. The observed App protocol has
    // no independently verified receive-stop command; closing the session stops it.
    bool stopAudioStream();

    // Next raw H.264 chunk in arrival order; false on timeout.
    bool readVideoChunk(Bytes& out, double timeoutSeconds = 3.0);
    // Next Opus microphone packet (48 kHz mono when decoded); false on timeout.
    bool readAudioOpus(Bytes& out, double timeoutSeconds = 1.0);
    void close();

private:
    Camera(const Camera&);
    Camera& operator=(const Camera&);

    void acceptChunk(const Bytes& payload);
    void acceptAudio(const Bytes& payload);
    void streamControl(std::uint8_t control, std::uint8_t state, std::uint8_t resolution);
    static bool pop(std::mutex& lock, std::condition_variable& condition, std::deque<Bytes>& queue,
                    Bytes& out, double timeoutSeconds);

    AppConnection& connection_;
    int videoListener_;
    int audioListener_;
    std::atomic<bool> streaming_;
    std::atomic<bool> audioStreaming_;

    std::mutex videoLock_;
    std::condition_variable videoCondition_;
    std::deque<Bytes> chunks_;

    std::mutex audioLock_;
    std::condition_variable audioCondition_;
    std::deque<Bytes> audioPackets_;
};

}  // namespace media
}  // namespace hanppie

#endif  // HANPPIE_MEDIA_HPP
