#include "hanppie/connection.hpp"

#include <chrono>
#include <cstdio>
#include <exception>

#include "net.hpp"

namespace hanppie {

namespace {

const std::size_t RECENT_FRAME_LIMIT = 256;

void sleepSeconds(double seconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(seconds * 1e6)));
}

}  // namespace

double monotonicSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

AppConnection::AppConnection(const std::string& robotIp, const std::string& appid,
                             const std::string& localIp, std::uint16_t localPort, bool debug)
    : suspendIdleKeepalive(false),
      appid_(protocol::normalizeAppid(appid)),
      localIp_(localIp),
      localPort_(localPort),
      debug_(debug),
      connected_(false),
      stop_(false),
      sequence_(protocol::INITIAL_DUSS_SEQUENCE),
      hasBattery_(false),
      battery_(0),
      controlPayload_(protocol::neutralControl()),
      hasControlDeadline_(false),
      controlDeadline_(0),
      modeKeepalive_(fromHex(protocol::MODE_NORMAL)),
      sendSdkReady_(true),
      nextListenerId_(1) {
    std::size_t begin = robotIp.find_first_not_of(" \t\r\n");
    std::size_t end = robotIp.find_last_not_of(" \t\r\n");
    robotIp_ = begin == std::string::npos ? "" : robotIp.substr(begin, end - begin + 1);
    info_ = AppConnectionInfo(robotIp_, appid_, "disconnected");
}

AppConnection::~AppConnection() {
    try {
        close();
    } catch (...) {
    }
}

int AppConnection::onDuss(const DussListener& listener) {
    std::lock_guard<std::mutex> lock(listenerLock_);
    dussListeners_[nextListenerId_] = listener;
    return nextListenerId_++;
}

int AppConnection::onVideo(const BytesListener& listener) {
    std::lock_guard<std::mutex> lock(listenerLock_);
    videoListeners_[nextListenerId_] = listener;
    return nextListenerId_++;
}

int AppConnection::onAudio(const BytesListener& listener) {
    std::lock_guard<std::mutex> lock(listenerLock_);
    audioListeners_[nextListenerId_] = listener;
    return nextListenerId_++;
}

int AppConnection::onProduct(const ProductListener& listener) {
    std::lock_guard<std::mutex> lock(listenerLock_);
    productListeners_[nextListenerId_] = listener;
    return nextListenerId_++;
}

bool AppConnection::removeListener(int id) {
    std::lock_guard<std::mutex> lock(listenerLock_);
    return dussListeners_.erase(id) + videoListeners_.erase(id) + audioListeners_.erase(id) +
               productListeners_.erase(id) >
           0;
}

template <typename Listener, typename Value>
void AppConnection::emit(const std::map<int, Listener>& listeners, const Value& value,
                         const char* event) {
    std::vector<Listener> snapshot;
    {
        std::lock_guard<std::mutex> lock(listenerLock_);
        for (typename std::map<int, Listener>::const_iterator it = listeners.begin();
             it != listeners.end(); ++it) {
            snapshot.push_back(it->second);
        }
    }
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        try {
            snapshot[i](value);
        } catch (const std::exception& error) {
            log(std::string("callback '") + event + "' failed: " + error.what());
        } catch (...) {
            log(std::string("callback '") + event + "' failed");
        }
    }
}

bool AppConnection::initialize(double timeoutSeconds) {
    if (connected_) return true;
    claimAppid(timeoutSeconds);
    sleepSeconds(0.2);
    socket_.reset(new net::UdpSocket());
    socket_->open(localIp_, localPort_);
    socket_->setReceiveBuffer(4 * 1024 * 1024);
    openOuterSession(timeoutSeconds < 5.0 ? timeoutSeconds : 5.0);
    sendConnectionSetup();
    connected_ = true;
    stop_ = false;
    thread_ = std::thread(&AppConnection::receiveLoop, this);
    return true;
}

void AppConnection::close() {
    stop_ = true;
    if (thread_.joinable()) {
        if (thread_.get_id() == std::this_thread::get_id()) {
            thread_.detach();
        } else {
            thread_.join();
        }
    }
    {
        std::lock_guard<std::recursive_mutex> lock(txLock_);
        if (socket_) socket_->close();
        socket_.reset();
    }
    connected_ = false;
    {
        std::lock_guard<std::mutex> lock(stateLock_);
        info_ = AppConnectionInfo(robotIp_, appid_, "disconnected", info_.mac);
        controlPayload_ = protocol::neutralControl();
        hasControlDeadline_ = false;
        periodicDuss_.clear();
        modeKeepalive_ = fromHex(protocol::MODE_NORMAL);
        sendSdkReady_ = true;
    }
    std::lock_guard<std::mutex> lock(frameLock_);
    recentFrames_.clear();
    frameCondition_.notify_all();
}

void AppConnection::claimAppid(double timeoutSeconds) {
    Bytes claim(appid_.begin(), appid_.end());
    net::UdpSocket connection;
    connection.open("0.0.0.0", protocol::APP_PORT, true);
    double deadline = monotonicSeconds() + timeoutSeconds;
    double broadcastDeadline = monotonicSeconds() + 4.0;
    if (broadcastDeadline > deadline) broadcastDeadline = deadline;
    double lastClaim = 0.0;
    bool sawBroadcast = false;
    if (!robotIp_.empty()) {
        connection.sendTo(claim, robotIp_, protocol::ROBOT_APP_PORT);
        lastClaim = monotonicSeconds();
    }
    while (monotonicSeconds() < deadline) {
        double now = monotonicSeconds();
        if (!robotIp_.empty() && !sawBroadcast && now >= broadcastDeadline) {
            setInfo(AppConnectionInfo(robotIp_, appid_, "reconnect"));
            return;
        }
        if (!robotIp_.empty() && sawBroadcast && now - lastClaim >= 1.0) {
            connection.sendTo(claim, robotIp_, protocol::ROBOT_APP_PORT);
            lastClaim = now;
        }
        if (!connection.waitReadable(0.2)) continue;
        Bytes data;
        std::string address;
        while (connection.receive(data, address)) {
            protocol::RobotBroadcast broadcast;
            if (!protocol::parseRobotBroadcast(data, broadcast)) continue;
            sawBroadcast = true;
            {
                std::lock_guard<std::mutex> lock(stateLock_);
                robotIp_ = broadcast.robotIp.empty() ? address : broadcast.robotIp;
            }
            setInfo(AppConnectionInfo(robotIp_, broadcast.appid,
                                      broadcast.pairing ? "pairing" : "idle", broadcast.robotMac));
            if (broadcast.appid == appid_) {
                connection.sendTo(claim, robotIp_, protocol::ROBOT_APP_PORT);
                return;
            }
            if (broadcast.appid == "00000000" || broadcast.pairing) {
                connection.sendTo(claim, robotIp_, protocol::ROBOT_APP_PORT);
                lastClaim = monotonicSeconds();
            }
        }
    }
    throw TimeoutError("AppID claim timed out for " + appid_);
}

void AppConnection::openOuterSession(double timeoutSeconds) {
    if (!socket_) throw std::runtime_error("control socket is not open");
    double deadline = monotonicSeconds() + timeoutSeconds;
    double lastSend = 0.0;
    while (monotonicSeconds() < deadline) {
        double now = monotonicSeconds();
        if (now - lastSend >= 0.2) {
            Bytes packet;
            {
                std::lock_guard<std::recursive_mutex> lock(txLock_);
                packet = envelope_.preconnect();
            }
            socket_->sendTo(packet, robotIp_, protocol::ROBOT_CONTROL_PORT);
            lastSend = now;
        }
        if (!socket_->waitReadable(0.03)) continue;
        Bytes data;
        std::string address;
        while (socket_->receive(data, address)) {
            if (data.size() < 4) continue;
            std::lock_guard<std::recursive_mutex> lock(txLock_);
            if (data[2] == envelope_.session[0] && data[3] == envelope_.session[1]) {
                envelope_.observe(data);
                return;
            }
            envelope_.session = protocol::nextSession(Bytes(data.begin() + 2, data.begin() + 4));
            lastSend = 0.0;
        }
    }
    log("outer session did not acknowledge preconnect; continuing");
}

std::uint16_t AppConnection::nextSequence() {
    std::lock_guard<std::recursive_mutex> lock(txLock_);
    std::uint16_t current = sequence_;
    sequence_ = static_cast<std::uint16_t>(sequence_ + 1);
    return current;
}

void AppConnection::sendPacket(const Bytes& packet) {
    std::lock_guard<std::recursive_mutex> lock(txLock_);
    if (!socket_) throw std::runtime_error("App connection is not initialized");
    socket_->sendTo(packet, robotIp_, protocol::ROBOT_CONTROL_PORT);
}

std::uint16_t AppConnection::sendDuss(std::uint8_t sender, std::uint8_t receiver,
                                      std::uint8_t attr, std::uint8_t cmdset, std::uint8_t cmdid,
                                      const Bytes& payload, const Bytes& flags) {
    std::lock_guard<std::recursive_mutex> lock(txLock_);
    std::uint16_t sequence = nextSequence();
    Bytes duss = protocol::buildDuss(sender, receiver, attr, cmdset, cmdid, payload, sequence);
    Bytes effectiveFlags = flags;
    if (effectiveFlags.empty()) {
        effectiveFlags.push_back(attr);
        effectiveFlags.push_back(0);
    }
    sendPacket(envelope_.wrapDirect(duss, effectiveFlags));
    return sequence;
}

std::uint16_t AppConnection::sendControl(const Bytes& payload) {
    std::lock_guard<std::recursive_mutex> lock(txLock_);
    std::uint16_t sequence = nextSequence();
    Bytes duss = protocol::buildDuss(protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV,
                                     protocol::ATTR_NO_ACK, protocol::CMDSET_SPECIAL,
                                     protocol::CMD_SPECIAL_RM_CONTROL, payload, sequence);
    sendPacket(envelope_.wrapControl(duss));
    return sequence;
}

void AppConnection::setControlPayload(const Bytes& payload) {
    if (payload.empty()) throw std::invalid_argument("control payload must not be empty");
    std::lock_guard<std::mutex> lock(stateLock_);
    controlPayload_ = payload;
    hasControlDeadline_ = false;
}

void AppConnection::setControlPayload(const Bytes& payload, double leaseSeconds) {
    if (payload.empty()) throw std::invalid_argument("control payload must not be empty");
    if (leaseSeconds <= 0) throw std::invalid_argument("leaseSeconds must be positive");
    std::lock_guard<std::mutex> lock(stateLock_);
    controlPayload_ = payload;
    hasControlDeadline_ = true;
    controlDeadline_ = monotonicSeconds() + leaseSeconds;
}

void AppConnection::setPeriodicDuss(const std::string& name, const PeriodicDuss& command) {
    std::lock_guard<std::mutex> lock(stateLock_);
    PeriodicEntry entry;
    entry.command = command;
    entry.hasDeadline = false;
    entry.deadline = 0;
    periodicDuss_[name] = entry;
}

void AppConnection::setPeriodicDuss(const std::string& name, const PeriodicDuss& command,
                                    double leaseSeconds) {
    if (leaseSeconds <= 0) throw std::invalid_argument("leaseSeconds must be positive");
    std::lock_guard<std::mutex> lock(stateLock_);
    PeriodicEntry entry;
    entry.command = command;
    entry.hasDeadline = true;
    entry.deadline = monotonicSeconds() + leaseSeconds;
    periodicDuss_[name] = entry;
}

void AppConnection::clearPeriodicDuss(const std::string& name) {
    std::lock_guard<std::mutex> lock(stateLock_);
    periodicDuss_.erase(name);
}

void AppConnection::configureModeKeepalive(const Bytes& payload, bool sendSdkReady) {
    std::lock_guard<std::mutex> lock(stateLock_);
    modeKeepalive_ = payload;
    sendSdkReady_ = sendSdkReady;
}

AppConnection::ControlTick AppConnection::controlTickState(double now) {
    std::lock_guard<std::mutex> lock(stateLock_);
    if (hasControlDeadline_ && now >= controlDeadline_) {
        controlPayload_ = protocol::neutralControl();
        hasControlDeadline_ = false;
    }
    ControlTick tick;
    tick.payload = controlPayload_;
    std::map<std::string, PeriodicEntry>::iterator it = periodicDuss_.begin();
    while (it != periodicDuss_.end()) {
        if (it->second.hasDeadline && now >= it->second.deadline) {
            periodicDuss_.erase(it++);
        } else {
            tick.periodic.push_back(it->second.command);
            ++it;
        }
    }
    return tick;
}

bool AppConnection::waitForDuss(std::uint16_t sequence, protocol::DussFrame& out, int cmdset,
                                int cmdid, int ack, double timeoutSeconds) {
    double deadline = monotonicSeconds() + (timeoutSeconds > 0 ? timeoutSeconds : 0.0);
    std::unique_lock<std::mutex> lock(frameLock_);
    for (;;) {
        for (std::deque<protocol::DussFrame>::reverse_iterator it = recentFrames_.rbegin();
             it != recentFrames_.rend(); ++it) {
            if (it->sequence == sequence && (cmdset == ANY || it->cmdset == cmdset) &&
                (cmdid == ANY || it->cmdid == cmdid) &&
                (ack == ANY || ((it->attr & 0x80) != 0) == (ack != 0))) {
                out = *it;
                return true;
            }
        }
        double remaining = deadline - monotonicSeconds();
        if (remaining <= 0 || !connected_) return false;
        frameCondition_.wait_for(lock, std::chrono::duration<double>(remaining));
    }
}

void AppConnection::sendConnectionSetup() {
    const std::vector<protocol::SetupEntry>& setup = protocol::appConnectionSetup();
    for (std::size_t i = 0; i < setup.size(); ++i) {
        const protocol::SetupEntry& entry = setup[i];
        if (entry.kind == "control") {
            sendControl(entry.payload);
            sleepSeconds(0.02);
        } else {
            sendDuss(entry.sender, entry.receiver, entry.attr, entry.cmdset, entry.cmdid,
                     entry.payload, entry.flags);
            sleepSeconds(0.006);
        }
    }
}

void AppConnection::receiveLoop() {
    double nextControl = monotonicSeconds();
    double nextKeepalive = monotonicSeconds();
    try {
        while (!stop_) {
            net::UdpSocket* socket = socket_.get();
            if (socket != NULL && socket->waitReadable(0.005)) {
                Bytes data;
                std::string address;
                while (!stop_ && socket->receive(data, address)) handlePacket(data);
            }
            double now = monotonicSeconds();
            if (now >= nextControl) {
                ControlTick tick = controlTickState(now);
                sendControl(tick.payload);
                for (std::size_t i = 0; i < tick.periodic.size(); ++i) {
                    const PeriodicDuss& c = tick.periodic[i];
                    sendDuss(c.sender, c.receiver, c.attr, c.cmdset, c.cmdid, c.payload);
                }
                nextControl = now + 0.02;
            }
            if (now >= nextKeepalive && !suspendIdleKeepalive) {
                Bytes keepalive;
                bool sdkReady;
                {
                    std::lock_guard<std::mutex> lock(stateLock_);
                    keepalive = modeKeepalive_;
                    sdkReady = sendSdkReady_;
                }
                sendDuss(protocol::HOST_MOBILE, protocol::HOST_HDVT_UAV, protocol::ATTR_NO_ACK,
                         protocol::CMDSET_RM, protocol::CMD_RM_SPECIAL_CONTROL, keepalive);
                if (sdkReady) {
                    sendDuss(protocol::HOST_MOBILE, protocol::HOST_WIFI, protocol::ATTR_NEED_ACK,
                             protocol::CMDSET_WIFI, protocol::CMD_WIFI_AP_KEEPALIVE);
                }
                nextKeepalive = now + 1.0;
            }
        }
    } catch (const std::exception& error) {
        // A send failure ends the session; waiters observe the disconnect.
        if (!stop_) {
            log(std::string("receive loop stopped: ") + error.what());
            connected_ = false;
            std::lock_guard<std::mutex> lock(frameLock_);
            frameCondition_.notify_all();
        }
    }
}

void AppConnection::handlePacket(const Bytes& data) {
    bool video;
    {
        std::lock_guard<std::recursive_mutex> lock(txLock_);
        video = protocol::isVideoPacket(data, envelope_.session);
        if (!video) envelope_.observe(data);
    }
    if (video) {
        emit(videoListeners_, Bytes(data.begin() + 20, data.end()), "video");
        return;
    }
    // The scanner already finds DUSS frames after either a 20-byte direct
    // header or a 34-byte control/stream header. Parsing an inner slice a
    // second time would dispatch the same frame twice.
    std::vector<protocol::DussFrame> frames = protocol::parseDussFrames(data);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const protocol::DussFrame& frame = frames[i];
        if (!frame.valid) continue;
        {
            std::lock_guard<std::mutex> lock(frameLock_);
            recentFrames_.push_back(frame);
            if (recentFrames_.size() > RECENT_FRAME_LIMIT) recentFrames_.pop_front();
            frameCondition_.notify_all();
        }
        RobotProduct product;
        bool changed;
        {
            std::lock_guard<std::mutex> lock(stateLock_);
            changed = updatedProduct(info_.product, frame, product);
            if (changed) info_.product = product;
        }
        if (frame.cmdset == protocol::CMDSET_VIRTUAL_BUS &&
            frame.cmdid == protocol::CMD_VBUS_DATA_ANALYSIS && frame.payload.size() == 62) {
            // Record battery before listeners run so telemetry waiters see it.
            std::lock_guard<std::mutex> lock(stateLock_);
            hasBattery_ = true;
            battery_ = frame.payload[10];
        }
        if (changed) emit(productListeners_, product, "product");
        emit(dussListeners_, frame, "duss");
        if (frame.cmdset == protocol::CMDSET_RM && frame.cmdid == protocol::CMD_RM_AUDIO_TO_APP) {
            emit(audioListeners_, frame.payload, "audio");
        }
    }
}

bool AppConnection::battery(int& out) const {
    std::lock_guard<std::mutex> lock(stateLock_);
    if (!hasBattery_) return false;
    out = battery_;
    return true;
}

std::string AppConnection::robotIp() const {
    std::lock_guard<std::mutex> lock(stateLock_);
    return robotIp_;
}

AppConnectionInfo AppConnection::info() const {
    std::lock_guard<std::mutex> lock(stateLock_);
    return info_;
}

RobotProduct AppConnection::product() const {
    std::lock_guard<std::mutex> lock(stateLock_);
    return info_.product;
}

protocol::AppEnvelope AppConnection::envelope() const {
    std::lock_guard<std::recursive_mutex> lock(txLock_);
    return envelope_;
}

void AppConnection::setInfo(const AppConnectionInfo& info) {
    std::lock_guard<std::mutex> lock(stateLock_);
    info_ = info;
}

void AppConnection::log(const std::string& message) const {
    if (debug_) std::fprintf(stderr, "[app] %s\n", message.c_str());
}

}  // namespace hanppie
