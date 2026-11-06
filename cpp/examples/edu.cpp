// Beginner example: find the robot, then drive and beep three times.
// Single-file build: put hanppie.h next to this file and include it instead.
//
// Usage: hanppie_edu   (no arguments; the robot is found on the local network)
#include "hanppie/hanppie.hpp"

using namespace hanppie::edu;

int main() {
    if (!connect()) return 1;
    for (int i = 0; i < 3; i++) {
        forward(1);
        beep();
        turnRight(90);
    }
    disconnect();
    return 0;
}
