// edu: move forward for one second, turn left, then beep.
// Not yet verified on hardware. Keep 0.8 m clear in front and 0.5 m elsewhere.
// Close other control apps; an operator must remain beside the robot.
#include "hanppie/hanppie.hpp"

using namespace hanppie::edu;

int main() {
    if (!connect()) return 1;
    if (!forward(1) || !turnLeft(90) || !beep()) {
        disconnect();
        return 1;
    }
    disconnect();
    return 0;
}
