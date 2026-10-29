// Package a Lab Python script and upload it to the robot's onboard FTP server.
//
// Usage: hanppie_lab <robot-ip> <script.py>
#include <cstdio>
#include <exception>
#include <fstream>
#include <sstream>

#include "hanppie/hanppie.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <robot-ip> <script.py>\n", argv[0]);
        return 2;
    }
    std::ifstream file(argv[2], std::ios::binary);
    if (!file) {
        std::fprintf(stderr, "error: cannot read %s\n", argv[2]);
        return 2;
    }
    std::ostringstream source;
    source << file.rdbuf();
    try {
        hanppie::LabProgramIdentity identity;
        std::string dsp = hanppie::buildLabProgram(source.str(), identity);
        std::string digest = hanppie::uploadLabProgram(argv[1], dsp);
        std::printf("uploaded guid=%s md5=%s\n", identity.guid.c_str(), digest.c_str());
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
