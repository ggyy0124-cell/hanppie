// Build and upload RoboMaster Lab DSP program containers.
// C++11 port of src/hanppie/lab.py.
#ifndef HANPPIE_LAB_HPP
#define HANPPIE_LAB_HPP

#include <cstdint>
#include <string>

#include "hanppie/protocol.hpp"

namespace hanppie {

extern const char* const DEFAULT_UPLOAD_NAME;

struct LabProgramIdentity {
    LabProgramIdentity()
        : fullMarker(protocol::SCRIPT_CTRL_METADATA_FULL),
          guidMarker(protocol::SCRIPT_CTRL_METADATA_GUID) {}

    std::string guid;
    std::string sign;
    std::uint8_t fullMarker;
    std::uint8_t guidMarker;
};

// Wrap Python source into the RoboMaster S1 Lab DSP XML container.
std::string buildLabProgram(const std::string& pythonSource, LabProgramIdentity& identity,
                            const std::string& title = "Hanppie-Lab");

// Upload a packaged DSP container to the robot's onboard FTP server and
// return the payload MD5 as lowercase hex.
std::string uploadLabProgram(const std::string& robotIp, const std::string& dsp,
                             const std::string& filename = DEFAULT_UPLOAD_NAME,
                             double timeoutSeconds = 10.0,
                             std::uint16_t port = protocol::ROBOT_FTP_PORT);

// Lowercase hex MD5 digest helper exposed for upload verification.
std::string md5Hex(const std::string& data);

}  // namespace hanppie

#endif  // HANPPIE_LAB_HPP
