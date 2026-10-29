#include "net.hpp"

#include <stdexcept>

#include "socket_compat.hpp"

namespace hanppie {
namespace net {

static_assert(sizeof(NativeSocket) == sizeof(sys::Handle), "NativeSocket must hold a socket handle");

namespace {

sockaddr_in makeAddress(const std::string& ip, std::uint16_t port) {
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
        throw std::invalid_argument("invalid IPv4 address: " + ip);
    }
    return address;
}

int pollTimeout(double seconds) {
    if (seconds <= 0) return 0;
    return static_cast<int>(seconds * 1000.0 + 0.5);
}

bool waitFor(sys::Handle handle, short events, double timeoutSeconds) {
    pollfd item;
    item.fd = handle;
    item.events = events;
    item.revents = 0;
    return sys::pollHandles(&item, 1, pollTimeout(timeoutSeconds)) > 0 &&
           (item.revents & (events | POLLERR | POLLHUP)) != 0;
}

}  // namespace

using sys::socketError;

UdpSocket::UdpSocket() : fd_(sys::INVALID_HANDLE) {}

UdpSocket::~UdpSocket() { close(); }

void UdpSocket::open(const std::string& bindIp, std::uint16_t port, bool broadcast) {
    close();
    sys::startup();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ == sys::INVALID_HANDLE) throw socketError("socket");
    int one = 1;
    sys::setOption(fd_, SOL_SOCKET, SO_REUSEADDR, one);
    if (broadcast) sys::setOption(fd_, SOL_SOCKET, SO_BROADCAST, one);
    sys::ignoreUdpConnectionReset(fd_);
    sockaddr_in address = makeAddress(bindIp, port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        std::runtime_error error = socketError("bind");
        close();
        throw error;
    }
    sys::setNonBlocking(fd_);
}

bool UdpSocket::isOpen() const { return fd_ != sys::INVALID_HANDLE; }

void UdpSocket::close() {
    if (fd_ != sys::INVALID_HANDLE) sys::closeHandle(fd_);
    fd_ = sys::INVALID_HANDLE;
}

void UdpSocket::setReceiveBuffer(int bytes) {
    if (fd_ != sys::INVALID_HANDLE) sys::setOption(fd_, SOL_SOCKET, SO_RCVBUF, bytes);
}

void UdpSocket::sendTo(const Bytes& data, const std::string& ip, std::uint16_t port) {
    if (fd_ == sys::INVALID_HANDLE) throw std::runtime_error("UDP socket is not open");
    sockaddr_in address = makeAddress(ip, port);
    if (sys::sendDatagram(fd_, data.data(), data.size(), address) < 0) throw socketError("sendto");
}

bool UdpSocket::waitReadable(double timeoutSeconds) {
    if (fd_ == sys::INVALID_HANDLE) return false;
    return waitFor(fd_, POLLIN, timeoutSeconds);
}

bool UdpSocket::receive(Bytes& data, std::string& fromIp) {
    if (fd_ == sys::INVALID_HANDLE) return false;
    data.resize(65535);
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    sys::IoSize received = sys::receiveDatagram(fd_, &data[0], data.size(), address);
    if (received < 0) {
        data.clear();
        return false;
    }
    data.resize(static_cast<std::size_t>(received));
    char ip[INET_ADDRSTRLEN] = {0};
    ::inet_ntop(AF_INET, &address.sin_addr, ip, sizeof(ip));
    fromIp = ip;
    return true;
}

TcpSocket::TcpSocket() : fd_(sys::INVALID_HANDLE), timeout_(10.0) {}

TcpSocket::~TcpSocket() { close(); }

void TcpSocket::connect(const std::string& host, std::uint16_t port, double timeoutSeconds) {
    close();
    sys::startup();
    timeout_ = timeoutSeconds;
    sockaddr_in address = makeAddress(host, port);
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ == sys::INVALID_HANDLE) throw socketError("socket");
#ifdef SO_NOSIGPIPE
    int one = 1;
    sys::setOption(fd_, SOL_SOCKET, SO_NOSIGPIPE, one);
#endif
    sys::setNonBlocking(fd_);
    int result = ::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    if (result != 0 && !sys::connectPending(sys::lastError())) {
        std::runtime_error error = socketError("connect");
        close();
        throw error;
    }
    if (result != 0) {
        if (!waitFor(fd_, POLLOUT, timeout_)) {
            close();
            throw std::runtime_error("connect timed out: " + host);
        }
        int error = 0;
        sys::AddressLength length = static_cast<sys::AddressLength>(sizeof(error));
        ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &length);
        if (error != 0) {
            std::runtime_error failure = sys::socketError("connect", error);
            close();
            throw failure;
        }
    }
    buffer_.clear();
}

void TcpSocket::close() {
    if (fd_ != sys::INVALID_HANDLE) sys::closeHandle(fd_);
    fd_ = sys::INVALID_HANDLE;
    buffer_.clear();
}

void TcpSocket::sendAll(const std::string& data) {
    sendAll(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
}

void TcpSocket::sendAll(const std::uint8_t* data, std::size_t size) {
    if (fd_ == sys::INVALID_HANDLE) throw std::runtime_error("TCP socket is not connected");
    std::size_t offset = 0;
    while (offset < size) {
        if (!waitFor(fd_, POLLOUT, timeout_)) throw std::runtime_error("send timed out");
        sys::IoSize sent = sys::sendBytes(fd_, data + offset, size - offset);
        if (sent < 0) {
            if (sys::wouldBlock(sys::lastError())) continue;
            throw socketError("send");
        }
        offset += static_cast<std::size_t>(sent);
    }
}

std::string TcpSocket::readLine() {
    if (fd_ == sys::INVALID_HANDLE) throw std::runtime_error("TCP socket is not connected");
    for (;;) {
        std::size_t newline = buffer_.find('\n');
        if (newline != std::string::npos) {
            std::string line = buffer_.substr(0, newline);
            buffer_.erase(0, newline + 1);
            if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
            return line;
        }
        if (!waitFor(fd_, POLLIN, timeout_)) throw std::runtime_error("read timed out");
        char chunk[1024];
        sys::IoSize received = sys::receiveBytes(fd_, chunk, sizeof(chunk));
        if (received < 0) {
            if (sys::wouldBlock(sys::lastError())) continue;
            throw socketError("recv");
        }
        if (received == 0) throw std::runtime_error("connection closed");
        buffer_.append(chunk, static_cast<std::size_t>(received));
    }
}

}  // namespace net
}  // namespace hanppie
