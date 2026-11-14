// edu: four one-second forward moves, each followed by a right turn.
// Hardware demonstration verified. Keep 1 m clear on all sides.
// Distance is estimated from time. Close other control apps; stay beside the robot.
#include <cstdio>
#include "hanppie/hanppie.hpp"

using namespace hanppie::edu;

int main() {
    if (!connect()) return 1;
    for (int side = 0; side < 4; ++side) {
        std::printf("Side %d/4: forward 1 s, then turn right 90 degrees\n", side + 1);
        std::fflush(stdout);
        if (!forward(1) || !turnRight(90)) {
            disconnect();
            return 1;
        }
    }
    disconnect();
    return 0;
}
