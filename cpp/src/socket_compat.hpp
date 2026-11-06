// Winsock / POSIX socket shims shared by net.cpp and the loopback tests.
// Internal header; keeps platform conditionals out of the rest of the SDK.
#ifndef HANPPIE_SOCKET_COMPAT_HPP
#define HANPPIE_SOCKET_COMPAT_HPP

#ifdef _WIN32
// WSAPoll needs Windows Vista or newer.
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
#include <cstdint>
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

// ws2_32.dll is loaded at run time so that programs link without -lws2_32;
// single-file builds such as Dev-C++ cannot add link options.
struct Winsock {
    decltype(&::WSAStartup) WSAStartup;
    decltype(&::WSACleanup) WSACleanup;
    decltype(&::WSAGetLastError) WSAGetLastError;
    decltype(&::WSAIoctl) WSAIoctl;
    decltype(&::WSAPoll) WSAPoll;
    decltype(&::socket) socket;
    decltype(&::bind) bind;
    decltype(&::connect) connect;
    decltype(&::listen) listen;
    decltype(&::accept) accept;
    decltype(&::getsockname) getsockname;
    decltype(&::getsockopt) getsockopt;
    decltype(&::setsockopt) setsockopt;
    decltype(&::ioctlsocket) ioctlsocket;
    decltype(&::closesocket) closesocket;
    decltype(&::send) send;
    decltype(&::recv) recv;
    decltype(&::sendto) sendto;
    decltype(&::recvfrom) recvfrom;
};

template <typename Function>
inline void loadFunction(HMODULE module, const char* name, Function& out) {
    FARPROC address = ::GetProcAddress(module, name);
    if (address == NULL) throw std::runtime_error(std::string("ws2_32.dll lacks ") + name);
    // Casting through a generic function pointer keeps -Wcast-function-type quiet.
    out = reinterpret_cast<Function>(reinterpret_cast<void (*)()>(address));
}

class WinsockSession {
public:
    WinsockSession() : module_(::LoadLibraryA("ws2_32.dll")), result_(-1) {
        if (module_ == NULL) return;
        loadFunction(module_, "WSAStartup", api_.WSAStartup);
        loadFunction(module_, "WSACleanup", api_.WSACleanup);
        loadFunction(module_, "WSAGetLastError", api_.WSAGetLastError);
        loadFunction(module_, "WSAIoctl", api_.WSAIoctl);
        loadFunction(module_, "WSAPoll", api_.WSAPoll);
        loadFunction(module_, "socket", api_.socket);
        loadFunction(module_, "bind", api_.bind);
        loadFunction(module_, "connect", api_.connect);
        loadFunction(module_, "listen", api_.listen);
        loadFunction(module_, "accept", api_.accept);
        loadFunction(module_, "getsockname", api_.getsockname);
        loadFunction(module_, "getsockopt", api_.getsockopt);
        loadFunction(module_, "setsockopt", api_.setsockopt);
        loadFunction(module_, "ioctlsocket", api_.ioctlsocket);
        loadFunction(module_, "closesocket", api_.closesocket);
        loadFunction(module_, "send", api_.send);
        loadFunction(module_, "recv", api_.recv);
        loadFunction(module_, "sendto", api_.sendto);
        loadFunction(module_, "recvfrom", api_.recvfrom);
        WSADATA data;
        result_ = api_.WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~WinsockSession() {
        if (result_ == 0) api_.WSACleanup();
    }
    bool ready() const { return result_ == 0; }
    const Winsock& api() const { return api_; }

private:
    HMODULE module_;  // stays loaded for the process lifetime
    int result_;
    Winsock api_;
};

inline WinsockSession& session() {
    static WinsockSession instance;
    return instance;
}

inline const Winsock& api() { return session().api(); }

// Winsock must be started before the first socket call; the session lives for
// the whole process and is cleaned up at exit.
inline void startup() {
    if (!session().ready()) throw std::runtime_error("cannot start Winsock (ws2_32.dll)");
}

inline int lastError() { return api().WSAGetLastError(); }
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

inline Handle openSocket(int type) { return api().socket(AF_INET, type, 0); }
inline void closeHandle(Handle handle) { api().closesocket(handle); }
inline int bindAddress(Handle handle, const sockaddr_in& address) {
    return api().bind(handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
}
inline int connectAddress(Handle handle, const sockaddr_in& address) {
    return api().connect(handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
}
inline int listenOn(Handle handle, int backlog) { return api().listen(handle, backlog); }
inline Handle acceptConnection(Handle handle) { return api().accept(handle, NULL, NULL); }
inline int localAddress(Handle handle, sockaddr_in& address) {
    AddressLength length = static_cast<AddressLength>(sizeof(address));
    return api().getsockname(handle, reinterpret_cast<sockaddr*>(&address), &length);
}
inline int pendingError(Handle handle) {
    int error = 0;
    AddressLength length = static_cast<AddressLength>(sizeof(error));
    api().getsockopt(handle, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &length);
    return error;
}

inline void setNonBlocking(Handle handle) {
    u_long enabled = 1;
    api().ioctlsocket(handle, FIONBIO, &enabled);
}

// A UDP send to a closed port makes the next recvfrom fail with WSAECONNRESET
// on Windows; POSIX silently drops the ICMP error. Match POSIX.
inline void ignoreUdpConnectionReset(Handle handle) {
    BOOL enabled = FALSE;
    DWORD returned = 0;
    api().WSAIoctl(handle, SIO_UDP_CONNRESET, &enabled, sizeof(enabled), NULL, 0, &returned, NULL,
                   NULL);
}

inline int pollHandles(pollfd* items, std::size_t count, int timeoutMs) {
    return api().WSAPoll(items, static_cast<ULONG>(count), timeoutMs);
}

template <typename T>
inline int setOption(Handle handle, int level, int name, const T& value) {
    return api().setsockopt(handle, level, name, reinterpret_cast<const char*>(&value),
                            static_cast<AddressLength>(sizeof(value)));
}

inline IoSize sendBytes(Handle handle, const void* data, std::size_t size) {
    return api().send(handle, static_cast<const char*>(data), static_cast<int>(size), 0);
}

inline IoSize receiveBytes(Handle handle, void* data, std::size_t size) {
    return api().recv(handle, static_cast<char*>(data), static_cast<int>(size), 0);
}

inline IoSize sendDatagram(Handle handle, const void* data, std::size_t size, const sockaddr_in& to) {
    return api().sendto(handle, static_cast<const char*>(data), static_cast<int>(size), 0,
                        reinterpret_cast<const sockaddr*>(&to), static_cast<AddressLength>(sizeof(to)));
}

inline IoSize receiveDatagram(Handle handle, void* data, std::size_t size, sockaddr_in& from) {
    AddressLength length = static_cast<AddressLength>(sizeof(from));
    return api().recvfrom(handle, static_cast<char*>(data), static_cast<int>(size), 0,
                          reinterpret_cast<sockaddr*>(&from), &length);
}
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

inline Handle openSocket(int type) { return ::socket(AF_INET, type, 0); }
inline int bindAddress(Handle handle, const sockaddr_in& address) {
    return ::bind(handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
}
inline int connectAddress(Handle handle, const sockaddr_in& address) {
    return ::connect(handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
}
inline int listenOn(Handle handle, int backlog) { return ::listen(handle, backlog); }
inline Handle acceptConnection(Handle handle) { return ::accept(handle, NULL, NULL); }
inline int localAddress(Handle handle, sockaddr_in& address) {
    AddressLength length = static_cast<AddressLength>(sizeof(address));
    return ::getsockname(handle, reinterpret_cast<sockaddr*>(&address), &length);
}
inline int pendingError(Handle handle) {
    int error = 0;
    AddressLength length = static_cast<AddressLength>(sizeof(error));
    ::getsockopt(handle, SOL_SOCKET, SO_ERROR, &error, &length);
    return error;
}

template <typename T>
inline int setOption(Handle handle, int level, int name, const T& value) {
    return ::setsockopt(handle, level, name, &value, static_cast<AddressLength>(sizeof(value)));
}

inline IoSize sendBytes(Handle handle, const void* data, std::size_t size) {
    return ::send(handle, data, size, SEND_FLAGS);
}

inline IoSize receiveBytes(Handle handle, void* data, std::size_t size) {
    return ::recv(handle, data, size, 0);
}

inline IoSize sendDatagram(Handle handle, const void* data, std::size_t size, const sockaddr_in& to) {
    return ::sendto(handle, data, size, 0, reinterpret_cast<const sockaddr*>(&to),
                    static_cast<AddressLength>(sizeof(to)));
}

inline IoSize receiveDatagram(Handle handle, void* data, std::size_t size, sockaddr_in& from) {
    AddressLength length = static_cast<AddressLength>(sizeof(from));
    return ::recvfrom(handle, data, size, 0, reinterpret_cast<sockaddr*>(&from), &length);
}
#endif

inline std::runtime_error socketError(const std::string& what, int error) {
    return std::runtime_error(what + ": " + errorText(error));
}

inline std::runtime_error socketError(const std::string& what) {
    return socketError(what, lastError());
}

// Byte-order and IPv4 text conversions written out here rather than taken from
// the socket library: on Windows they live in ws2_32.dll, which is loaded late.
inline std::uint16_t networkPort(std::uint16_t port) {
    unsigned char bytes[2] = {static_cast<unsigned char>(port >> 8),
                              static_cast<unsigned char>(port & 0xFF)};
    std::uint16_t value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

inline std::uint16_t hostPort(std::uint16_t networkValue) {
    unsigned char bytes[2];
    std::memcpy(bytes, &networkValue, sizeof(bytes));
    return static_cast<std::uint16_t>((bytes[0] << 8) | bytes[1]);
}

// Dotted-quad IPv4 only, as inet_pton(AF_INET) accepts.
inline bool parseIpv4(const std::string& text, in_addr& out) {
    unsigned char bytes[4];
    std::size_t position = 0;
    for (int part = 0; part < 4; ++part) {
        if (part > 0) {
            if (position >= text.size() || text[position] != '.') return false;
            ++position;
        }
        std::size_t start = position;
        unsigned value = 0;
        while (position < text.size() && text[position] >= '0' && text[position] <= '9') {
            value = value * 10 + static_cast<unsigned>(text[position] - '0');
            if (position - start >= 3 || value > 255) return false;
            ++position;
        }
        if (position == start) return false;
        if (position - start > 1 && text[start] == '0') return false;  // no leading zeros
        bytes[part] = static_cast<unsigned char>(value);
    }
    if (position != text.size()) return false;
    std::memcpy(&out, bytes, sizeof(bytes));
    return true;
}

inline std::string formatIpv4(const in_addr& address) {
    unsigned char bytes[4];
    std::memcpy(bytes, &address, sizeof(bytes));
    std::string text;
    for (int part = 0; part < 4; ++part) {
        if (part > 0) text += '.';
        text += std::to_string(static_cast<unsigned>(bytes[part]));
    }
    return text;
}

inline sockaddr_in makeAddress(const std::string& ip, std::uint16_t port) {
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = networkPort(port);
    if (!parseIpv4(ip, address.sin_addr)) throw std::invalid_argument("invalid IPv4 address: " + ip);
    return address;
}

}  // namespace sys
}  // namespace net
}  // namespace hanppie

#endif  // HANPPIE_SOCKET_COMPAT_HPP
