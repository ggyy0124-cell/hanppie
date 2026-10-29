#include "hanppie/lab.hpp"

#include <cstdio>
#include <random>
#include <stdexcept>

#include "md5.hpp"
#include "net.hpp"

namespace hanppie {

const char* const DEFAULT_UPLOAD_NAME = "python_raw.dsp";

namespace {

std::string tokenHex(std::size_t bytes) {
    static std::random_device device;
    Bytes data;
    for (std::size_t i = 0; i < bytes; ++i) data.push_back(static_cast<std::uint8_t>(device() & 0xFF));
    return toHex(data);
}

std::string replaceAll(std::string value, const std::string& from, const std::string& to) {
    std::size_t position = 0;
    while ((position = value.find(from, position)) != std::string::npos) {
        value.replace(position, from.size(), to);
        position += to.size();
    }
    return value;
}

// xml.sax.saxutils.escape: &, <, >.
std::string escapeXml(const std::string& value) {
    return replaceAll(replaceAll(replaceAll(value, "&", "&amp;"), ">", "&gt;"), "<", "&lt;");
}

std::string cdata(const std::string& value) { return replaceAll(value, "]]>", "]]]]><![CDATA[>"); }

int replyCode(const std::string& line) {
    if (line.size() < 3) return -1;
    int code = 0;
    for (int i = 0; i < 3; ++i) {
        if (line[i] < '0' || line[i] > '9') return -1;
        code = code * 10 + (line[i] - '0');
    }
    return code;
}

// Read one (possibly multi-line) FTP reply and return its code.
int readReply(net::TcpSocket& control, std::string& text) {
    text = control.readLine();
    int code = replyCode(text);
    if (text.size() > 3 && text[3] == '-') {
        std::string terminator = text.substr(0, 3) + " ";
        for (;;) {
            std::string line = control.readLine();
            text += "\n" + line;
            if (line.compare(0, 4, terminator) == 0) break;
        }
    }
    return code;
}

int command(net::TcpSocket& control, const std::string& line, std::string& reply) {
    control.sendAll(line + "\r\n");
    return readReply(control, reply);
}

void expect(int code, int low, int high, const std::string& reply) {
    if (code < low || code > high) throw std::runtime_error("FTP error: " + reply);
}

}  // namespace

std::string md5Hex(const std::string& data) {
    return toHex(md5(reinterpret_cast<const std::uint8_t*>(data.data()), data.size()));
}

std::string buildLabProgram(const std::string& pythonSource, LabProgramIdentity& identity,
                            const std::string& title) {
    identity = LabProgramIdentity();
    identity.guid = tokenHex(16);
    identity.sign = tokenHex(8);
    return "<dji><attribute>"
           "<creation_date>2026/08/30</creation_date>"
           "<modify_time>2026/08/30</modify_time>"
           "<sign>" + identity.sign + "</sign>"
           "<guid>" + identity.guid + "</guid>"
           "<creator>Hanppie</creator>"
           "<firmware_version_dependency>00.00.0000</firmware_version_dependency>"
           "<title>" + escapeXml(title) + "</title>"
           "<code_type>python</code_type>"
           "<app_min_version></app_min_version><app_max_version></app_max_version>"
           "</attribute><audio-list /><code><python_code><![CDATA[" +
           cdata(pythonSource) + "]]></python_code></code></dji>";
}

std::string uploadLabProgram(const std::string& robotIp, const std::string& dsp,
                             const std::string& filename, double timeoutSeconds,
                             std::uint16_t port) {
    std::string digest = md5Hex(dsp);
    net::TcpSocket control;
    control.connect(robotIp, port, timeoutSeconds);
    std::string reply;
    expect(readReply(control, reply), 200, 299, reply);
    int code = command(control, "USER anonymous", reply);
    if (code == 331) code = command(control, "PASS anonymous@", reply);
    expect(code, 200, 299, reply);
    expect(command(control, "CWD python", reply), 200, 299, reply);
    expect(command(control, "TYPE I", reply), 200, 299, reply);
    expect(command(control, "PASV", reply), 227, 227, reply);

    std::size_t open = reply.find('(');
    unsigned h1, h2, h3, h4, p1, p2;
    if (open == std::string::npos ||
        std::sscanf(reply.c_str() + open, "(%u,%u,%u,%u,%u,%u)", &h1, &h2, &h3, &h4, &p1, &p2) != 6) {
        throw std::runtime_error("FTP error: unexpected PASV reply: " + reply);
    }
    char host[16];
    std::snprintf(host, sizeof(host), "%u.%u.%u.%u", h1, h2, h3, h4);
    net::TcpSocket data;
    data.connect(host, static_cast<std::uint16_t>((p1 << 8) | p2), timeoutSeconds);
    expect(command(control, "STOR " + filename, reply), 100, 199, reply);
    data.sendAll(dsp);
    data.close();
    expect(readReply(control, reply), 200, 299, reply);
    try {
        command(control, "QUIT", reply);
    } catch (const std::runtime_error&) {
    }
    return digest;
}

}  // namespace hanppie
