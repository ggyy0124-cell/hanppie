// Single-header build check: compiled alone with the Dev-C++ / exam options
// (-O2 -std=c++11 -DONLINE_JUDGE, no link flags) against tools/amalgamate.py
// output. Vectors repeat test_main.cpp; the LED one caught a TDM-GCC 4.9.2 -O2
// miscompile, so it guards the generated file, not just the library build.
#include <cstdio>
#include <string>

#include "hanppie.h"

// Typical GESP-style student prologue: both using-directives together must not
// make the beginner calls ambiguous (std::left/std::right/std::forward, and
// POSIX connect() / wait(int*) from the system headers).
using namespace std;
using namespace hanppie::edu;

// Compiled, never run: proves a student program resolves to hanppie::edu.
int studentProgram() {
    if (!connect()) return 1;
    for (int i = 0; i < 3; i++) {
        forward(1);
        left(1);
        right(0.5);
        backward(1);
        turnLeft(90);
        turnRight(45.5);
        beep();
        wait(0);
        wait(1);
        wait(0.5);
    }
    stop();
    disconnect();
    return 0;
}

namespace {

int failures = 0;

void expect(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL %s\n", what);
        ++failures;
    }
}

}  // namespace

int main() {
    using hanppie::Robot;
    using hanppie::toHex;
    using hanppie::buildLedPayload;
    using hanppie::buildChassisSpeedPayload;
    using hanppie::buildGimbalSpeedPayload;
    using hanppie::md5Hex;
    using hanppie::sleepSeconds;
    namespace ctrl = hanppie::ctrl;
    expect(toHex(buildLedPayload(0x3F, 255, 0, 0, "on")) == "3f000000ff0071ff000000e803e803",
           "LED payload");
    expect(toHex(buildChassisSpeedPayload(0.5, -0.5, 60.0)) == "0000003f000000bf00007042",
           "chassis speed payload");
    expect(toHex(buildGimbalSpeedPayload(-60, 60)) == "58020000a8fddc", "gimbal speed payload");
    expect(md5Hex("abc") == "900150983cd24fb0d6963f7d28e17f72", "MD5");

    // Constructing a robot opens no socket; Lab-style names must resolve.
    Robot robot("192.168.2.1", "0123abcd");
    expect(!robot.connected(), "robot starts disconnected");
    expect(ctrl::rm_define::clockwise != ctrl::rm_define::anticlockwise, "Lab constants");
    sleepSeconds(0.01);

    std::printf("Hanppie %s single header: %s\n", HANPPIE_VERSION, failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
