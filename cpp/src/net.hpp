// Minimal UDP/TCP socket helpers used by the SDK (POSIX and Winsock). Internal
// header; platform specifics live in socket_compat.hpp so that this header does
// not pull <winsock2.h> into the rest of the SDK.
#ifndef HANPPIE_NET_HPP
#define HANPPIE_NET_HPP

#include <cstddef>
#include <cstdint>
#include <string>

#include "hanppie/protocol.hpp"

namespace hanppie {
namespace net {

// Same width as SOCKET on Windows and a file descriptor elsewhere.
#ifdef _WIN32
typedef std::uintptr_t NativeSocket;
#else
typedef int NativeSocket;
#endif

class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();

    void open(const std::string& bindIp, std::uint16_t port, bool broadcast = false);
    void close();
    bool isOpen() const;
    void setReceiveBuffer(int bytes);
    void sendTo(const Bytes& data, const std::string& ip, std::uint16_t port);
    // Wait up to timeoutSeconds for a readable datagram.
    bool waitReadable(double timeoutSeconds);
    // Non-blocking receive; returns false when nothing is pending.
    bool receive(Bytes& data, std::string& fromIp);

private:
    UdpSocket(const UdpSocket&);
    UdpSocket& operator=(const UdpSocket&);
    NativeSocket fd_;
};

class TcpSocket {
public:
    TcpSocket();
    ~TcpSocket();

    void connect(const std::string& host, std::uint16_t port, double timeoutSeconds);
    void close();
    void sendAll(const std::string& data);
    void sendAll(const std::uint8_t* data, std::size_t size);
    // Read one CRLF/LF terminated line without the terminator.
    std::string readLine();

private:
    TcpSocket(const TcpSocket&);
    TcpSocket& operator=(const TcpSocket&);
    NativeSocket fd_;
    double timeout_;
    std::string buffer_;
};

}  // namespace net
}  // namespace hanppie

#endif  // HANPPIE_NET_HPP
