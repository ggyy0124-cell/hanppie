// RoboMaster App-compatible UDP session used to reach the native Lab subsystem.
// C++11 port of src/hanppie/connection.py.
#ifndef HANPPIE_CONNECTION_HPP
#define HANPPIE_CONNECTION_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hanppie/product.hpp"
#include "hanppie/protocol.hpp"

namespace hanppie {

namespace net {
class UdpSocket;
}

// Seconds on a monotonic clock; used for telemetry timestamps and leases.
double monotonicSeconds();

class TimeoutError : public std::runtime_error {
public:
    explicit TimeoutError(const std::string& what) : std::runtime_error(what) {}
};

struct AppConnectionInfo {
    AppConnectionInfo() {}
    AppConnectionInfo(const std::string& ip_, const std::string& appid_, const std::string& state_,
                      const std::string& mac_ = "")
        : ip(ip_), appid(appid_), state(state_), mac(mac_) {}

    std::string ip;
    std::string appid;
    std::string state;
    std::string mac;
    RobotProduct product;
};

// One DUSS command repeated with every 50 Hz control tick.
struct PeriodicDuss {
    std::uint8_t sender;
    std::uint8_t receiver;
    std::uint8_t attr;
    std::uint8_t cmdset;
    std::uint8_t cmdid;
    Bytes payload;
};

// Own the AppID claim, outer session and DUSS receive loop.
class AppConnection {
public:
    typedef std::function<void(const protocol::DussFrame&)> DussListener;
    typedef std::function<void(const Bytes&)> BytesListener;
    typedef std::function<void(const RobotProduct&)> ProductListener;

    // A filter value of ANY matches every cmdset / cmdid / ack state.
    static const int ANY = -1;

    AppConnection(const std::string& robotIp, const std::string& appid,
                  const std::string& localIp = "0.0.0.0",
                  std::uint16_t localPort = protocol::LOCAL_CONTROL_PORT, bool debug = false);
    ~AppConnection();

    // Listeners run on the receive thread; exceptions they throw are logged and dropped.
    int onDuss(const DussListener& listener);
    int onVideo(const BytesListener& listener);
    int onAudio(const BytesListener& listener);
    int onProduct(const ProductListener& listener);
    bool removeListener(int id);

    bool initialize(double timeoutSeconds = 20.0);
    void close();

    std::uint16_t sendDuss(std::uint8_t sender, std::uint8_t receiver, std::uint8_t attr,
                           std::uint8_t cmdset, std::uint8_t cmdid, const Bytes& payload = Bytes(),
                           const Bytes& flags = Bytes());
    std::uint16_t sendControl(const Bytes& payload = protocol::neutralControl());

    // Set the control-channel payload repeated by the receive loop, optionally
    // reverting to neutral after leaseSeconds.
    void setControlPayload(const Bytes& payload);
    void setControlPayload(const Bytes& payload, double leaseSeconds);

    // Add or replace one DUSS command sent with every control tick.
    void setPeriodicDuss(const std::string& name, const PeriodicDuss& command);
    void setPeriodicDuss(const std::string& name, const PeriodicDuss& command, double leaseSeconds);
    void clearPeriodicDuss(const std::string& name);

    // Configure the one-second App mode keepalive used by the active backend.
    void configureModeKeepalive(const Bytes& payload, bool sendSdkReady);

    // Wait for a matching received DUSS frame without losing an early response.
    bool waitForDuss(std::uint16_t sequence, protocol::DussFrame& out, int cmdset = ANY,
                     int cmdid = ANY, int ack = ANY, double timeoutSeconds = 1.0);

    bool battery(int& out) const;
    bool connected() const { return connected_; }
    std::string robotIp() const;
    std::string appid() const { return appid_; }
    AppConnectionInfo info() const;
    RobotProduct product() const;

    // Pause the one-second mode keepalive while another workflow owns it.
    std::atomic<bool> suspendIdleKeepalive;

    // Outer envelope state; guarded internally, exposed for diagnostics.
    protocol::AppEnvelope envelope() const;

private:
    AppConnection(const AppConnection&);
    AppConnection& operator=(const AppConnection&);

    struct ControlTick {
        Bytes payload;
        std::vector<PeriodicDuss> periodic;
    };

    struct PeriodicEntry {
        PeriodicDuss command;
        bool hasDeadline;
        double deadline;
    };

    void claimAppid(double timeoutSeconds);
    void openOuterSession(double timeoutSeconds);
    void sendConnectionSetup();
    std::uint16_t nextSequence();
    void sendPacket(const Bytes& packet);
    ControlTick controlTickState(double now);
    void receiveLoop();
    void handlePacket(const Bytes& data);
    void setInfo(const AppConnectionInfo& info);
    void log(const std::string& message) const;

    template <typename Listener, typename Value>
    void emit(const std::map<int, Listener>& listeners, const Value& value, const char* event);

    std::string robotIp_;
    std::string appid_;
    std::string localIp_;
    std::uint16_t localPort_;
    bool debug_;

    std::unique_ptr<net::UdpSocket> socket_;
    std::atomic<bool> connected_;
    std::atomic<bool> stop_;
    std::thread thread_;

    mutable std::recursive_mutex txLock_;
    protocol::AppEnvelope envelope_;
    std::uint16_t sequence_;

    mutable std::mutex stateLock_;
    AppConnectionInfo info_;
    bool hasBattery_;
    int battery_;
    Bytes controlPayload_;
    bool hasControlDeadline_;
    double controlDeadline_;
    std::map<std::string, PeriodicEntry> periodicDuss_;
    Bytes modeKeepalive_;
    bool sendSdkReady_;

    mutable std::mutex listenerLock_;
    int nextListenerId_;
    std::map<int, DussListener> dussListeners_;
    std::map<int, BytesListener> videoListeners_;
    std::map<int, BytesListener> audioListeners_;
    std::map<int, ProductListener> productListeners_;

    std::mutex frameLock_;
    std::condition_variable frameCondition_;
    std::deque<protocol::DussFrame> recentFrames_;
};

}  // namespace hanppie

#endif  // HANPPIE_CONNECTION_HPP
