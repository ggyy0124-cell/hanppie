#include "net.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace hanppie {
namespace net {

namespace {

std::runtime_error socketError(const std::string& what) {
    return std::runtime_error(what + ": " + std::strerror(errno));
}

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

}  // namespace

UdpSocket::UdpSocket() : fd_(-1) {}

UdpSocket::~UdpSocket() { close(); }

void UdpSocket::open(const std::string& bindIp, std::uint16_t port, bool broadcast) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw socketError("socket");
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (broadcast) ::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
    sockaddr_in address = makeAddress(bindIp, port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        std::runtime_error error = socketError("bind");
        close();
        throw error;
    }
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
}

void UdpSocket::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

void UdpSocket::setReceiveBuffer(int bytes) {
    if (fd_ >= 0) ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof(bytes));
}

void UdpSocket::sendTo(const Bytes& data, const std::string& ip, std::uint16_t port) {
    if (fd_ < 0) throw std::runtime_error("UDP socket is not open");
    sockaddr_in address = makeAddress(ip, port);
    ssize_t sent = ::sendto(fd_, data.data(), data.size(), 0, reinterpret_cast<sockaddr*>(&address),
                            sizeof(address));
    if (sent < 0) throw socketError("sendto");
}

bool UdpSocket::waitReadable(double timeoutSeconds) {
    if (fd_ < 0) return false;
    pollfd item;
    item.fd = fd_;
    item.events = POLLIN;
    item.revents = 0;
    return ::poll(&item, 1, pollTimeout(timeoutSeconds)) > 0 && (item.revents & POLLIN);
}

bool UdpSocket::receive(Bytes& data, std::string& fromIp) {
    if (fd_ < 0) return false;
    data.resize(65535);
    sockaddr_in address;
    socklen_t length = sizeof(address);
    ssize_t received = ::recvfrom(fd_, data.data(), data.size(), 0,
                                  reinterpret_cast<sockaddr*>(&address), &length);
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

TcpSocket::TcpSocket() : fd_(-1), timeout_(10.0) {}

TcpSocket::~TcpSocket() { close(); }

void TcpSocket::connect(const std::string& host, std::uint16_t port, double timeoutSeconds) {
    close();
    timeout_ = timeoutSeconds;
    sockaddr_in address = makeAddress(host, port);
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) throw socketError("socket");
#ifdef SO_NOSIGPIPE
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    int result = ::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    if (result != 0 && errno != EINPROGRESS) {
        std::runtime_error error = socketError("connect");
        close();
        throw error;
    }
    if (result != 0) {
        pollfd item;
        item.fd = fd_;
        item.events = POLLOUT;
        item.revents = 0;
        if (::poll(&item, 1, pollTimeout(timeout_)) <= 0) {
            close();
            throw std::runtime_error("connect timed out: " + host);
        }
        int error = 0;
        socklen_t length = sizeof(error);
        ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &length);
        if (error != 0) {
            errno = error;
            std::runtime_error failure = socketError("connect");
            close();
            throw failure;
        }
    }
    buffer_.clear();
}

void TcpSocket::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    buffer_.clear();
}

void TcpSocket::sendAll(const std::string& data) {
    sendAll(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
}

void TcpSocket::sendAll(const std::uint8_t* data, std::size_t size) {
    if (fd_ < 0) throw std::runtime_error("TCP socket is not connected");
    std::size_t offset = 0;
    while (offset < size) {
        pollfd item;
        item.fd = fd_;
        item.events = POLLOUT;
        item.revents = 0;
        if (::poll(&item, 1, pollTimeout(timeout_)) <= 0) throw std::runtime_error("send timed out");
#ifdef MSG_NOSIGNAL
        ssize_t sent = ::send(fd_, data + offset, size - offset, MSG_NOSIGNAL);
#else
        ssize_t sent = ::send(fd_, data + offset, size - offset, 0);
#endif
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            throw socketError("send");
        }
        offset += static_cast<std::size_t>(sent);
    }
}

std::string TcpSocket::readLine() {
    if (fd_ < 0) throw std::runtime_error("TCP socket is not connected");
    for (;;) {
        std::size_t newline = buffer_.find('\n');
        if (newline != std::string::npos) {
            std::string line = buffer_.substr(0, newline);
            buffer_.erase(0, newline + 1);
            if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
            return line;
        }
        pollfd item;
        item.fd = fd_;
        item.events = POLLIN;
        item.revents = 0;
        if (::poll(&item, 1, pollTimeout(timeout_)) <= 0) throw std::runtime_error("read timed out");
        char chunk[1024];
        ssize_t received = ::recv(fd_, chunk, sizeof(chunk), 0);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            throw socketError("recv");
        }
        if (received == 0) throw std::runtime_error("connection closed");
        buffer_.append(chunk, static_cast<std::size_t>(received));
    }
}

}  // namespace net
}  // namespace hanppie
