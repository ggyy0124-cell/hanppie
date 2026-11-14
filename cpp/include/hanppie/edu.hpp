// Beginner entry for classroom C++ (for example GESP practice in Dev-C++).
//
// Plain functions over one robot so that a first program needs only function
// calls, variables, conditions and loops:
//
//     #include "hanppie.h"
//     using namespace hanppie::edu;
//
//     int main() {
//         if (!connect()) return 1;
//         for (int i = 0; i < 3; i++) {
//             forward(1);
//             beep();
//         }
//         disconnect();
//     }
//
// connect() finds the robot from its network broadcast, so programs contain no
// address or AppID. Every movement runs at a fixed low speed for a bounded time
// and stops when it ends, fails, or the program exits. Functions never throw:
// failures print a short message, stop the robot and return false.
#ifndef HANPPIE_EDU_HPP
#define HANPPIE_EDU_HPP

namespace hanppie {
namespace edu {

// Fixed classroom limits.
const double MOVE_SPEED = 0.3;      // m/s
const double TURN_SPEED = 60.0;     // deg/s
const double MAX_SECONDS = 10.0;    // longest single movement or wait
const double MAX_DEGREES = 360.0;   // largest single turn

// Find the robot on the local network, connect, and allow movement.
bool connect();
// Stop the robot and release it. Also runs automatically when the program ends.
void disconnect();
bool connected();

// Drive for `seconds` (0 to MAX_SECONDS), then stop.
bool forward(double seconds);
bool backward(double seconds);
// Slide sideways without turning.
bool left(double seconds);
bool right(double seconds);
// Turn on the spot by `degrees` (0 to MAX_DEGREES), adding to the logical
// heading captured at connect(). Translation does not reset that target;
// each completed turn corrects drift back to the accumulated heading.
bool turnLeft(double degrees);
bool turnRight(double degrees);
bool stop();
// Short tone from the robot speaker.
bool beep();
// Pause the program; the robot keeps still. The int overload keeps wait(0)
// unambiguous next to POSIX wait(int*), which macOS <stdlib.h> declares.
void wait(double seconds);
void wait(int seconds);

}  // namespace edu
}  // namespace hanppie

#endif  // HANPPIE_EDU_HPP
