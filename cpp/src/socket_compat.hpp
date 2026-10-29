// Winsock / POSIX socket shims shared by net.cpp and the loopback tests.
// Internal header; keeps platform conditionals out of the rest of the SDK.
#ifndef HANPPIE_SOCKET_COMPAT_HPP
#define HANPPIE_SOCKET_COMPAT_HPP

#ifdef _WIN32
// WSAPoll and inet_pton/inet_ntop need Windows Vista or newer.
#if !defined(_WIN32_WINNT) || _WIN32_WINNT < 0x0600
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>

namespace hanppie {
namespace net {
namespace sys {

#ifdef _WIN32
typedef SOCKET Handle;
typedef int IoSize;
typedef int AddressLength;
const Handle INVALID_HANDLE = INVALID_SOCKET;

class WinsockSession {
public:
    WinsockSession() {
        WSADATA data;
        result_ = ::WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~WinsockSession() {
        if (result_ == 0) ::WSACleanup();
    }
    int result() const { return result_; }

private:
    int result_;
};

// Winsock must be started before the first socket call; the session lives for
// the whole process and is cleaned up at exit.
inline void startup() {
    static WinsockSession session;
    if (session.result() != 0) throw std::runtime_error("WSAStartup failed");
}

inline int lastError() { return ::WSAGetLastError(); }
inline bool wouldBlock(int error) { return error == WSAEWOULDBLOCK || error == WSAEINTR; }
inline bool connectPending(int error) { return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS; }

inline std::string errorText(int error) {
    char text[256] = {0};
    DWORD size = ::FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
                                  static_cast<DWORD>(error), 0, text, sizeof(text), NULL);
    while (size > 0 && (text[size - 1] == '\r' || text[size - 1] == '\n' || text[size - 1] == ' ')) {
        text[--size] = '\0';
    }
    return std::string(text) + " (WSA " + std::to_string(error) + ")";
}

inline void closeHandle(Handle handle) { ::closesocket(handle); }

inline void setNonBlocking(Handle handle) {
    u_long enabled = 1;
    ::ioctlsocket(handle, FIONBIO, &enabled);
}

// A UDP send to a closed port makes the next recvfrom fail with WSAECONNRESET
// on Windows; POSIX silently drops the ICMP error. Match POSIX.
inline void ignoreUdpConnectionReset(Handle handle) {
    BOOL enabled = FALSE;
    DWORD returned = 0;
    ::WSAIoctl(handle, SIO_UDP_CONNRESET, &enabled, sizeof(enabled), NULL, 0, &returned, NULL, NULL);
}

inline int pollHandles(pollfd* items, std::size_t count, int timeoutMs) {
    return ::WSAPoll(items, static_cast<ULONG>(count), timeoutMs);
}

const int SEND_FLAGS = 0;
#else
typedef int Handle;
typedef ssize_t IoSize;
typedef socklen_t AddressLength;
const Handle INVALID_HANDLE = -1;

inline void startup() {}
inline int lastError() { return errno; }
inline bool wouldBlock(int error) { return error == EAGAIN || error == EWOULDBLOCK || error == EINTR; }
inline bool connectPending(int error) { return error == EINPROGRESS; }
inline std::string errorText(int error) { return std::strerror(error); }
inline void closeHandle(Handle handle) { ::close(handle); }

inline void setNonBlocking(Handle handle) {
    int flags = ::fcntl(handle, F_GETFL, 0);
    ::fcntl(handle, F_SETFL, flags | O_NONBLOCK);
}

inline void ignoreUdpConnectionReset(Handle) {}

inline int pollHandles(pollfd* items, std::size_t count, int timeoutMs) {
    return ::poll(items, static_cast<nfds_t>(count), timeoutMs);
}

#ifdef MSG_NOSIGNAL
const int SEND_FLAGS = MSG_NOSIGNAL;
#else
const int SEND_FLAGS = 0;
#endif
#endif

inline std::runtime_error socketError(const std::string& what, int error) {
    return std::runtime_error(what + ": " + errorText(error));
}

inline std::runtime_error socketError(const std::string& what) {
    return socketError(what, lastError());
}

// setsockopt takes const char* on Winsock and const void* on POSIX.
template <typename T>
inline int setOption(Handle handle, int level, int name, const T& value) {
    return ::setsockopt(handle, level, name, reinterpret_cast<const char*>(&value),
                        static_cast<AddressLength>(sizeof(value)));
}

inline IoSize sendBytes(Handle handle, const void* data, std::size_t size) {
    return ::send(handle, static_cast<const char*>(data), static_cast<int>(size), SEND_FLAGS);
}

inline IoSize receiveBytes(Handle handle, void* data, std::size_t size) {
    return ::recv(handle, static_cast<char*>(data), static_cast<int>(size), 0);
}

inline IoSize sendDatagram(Handle handle, const void* data, std::size_t size, const sockaddr_in& to) {
    return ::sendto(handle, static_cast<const char*>(data), static_cast<int>(size), 0,
                    reinterpret_cast<const sockaddr*>(&to), static_cast<AddressLength>(sizeof(to)));
}

inline IoSize receiveDatagram(Handle handle, void* data, std::size_t size, sockaddr_in& from) {
    AddressLength length = static_cast<AddressLength>(sizeof(from));
    return ::recvfrom(handle, static_cast<char*>(data), static_cast<int>(size), 0,
                      reinterpret_cast<sockaddr*>(&from), &length);
}

}  // namespace sys
}  // namespace net
}  // namespace hanppie

#endif  // HANPPIE_SOCKET_COMPAT_HPP
