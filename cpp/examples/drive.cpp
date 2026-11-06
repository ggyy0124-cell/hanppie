// Drive a RoboMaster S1 forward for one second with the Lab-style interface.
//
// Usage: hanppie_drive [<robot-ip> <appid>]
// Without arguments the robot is found from its network broadcast.
#include <cstdio>
#include <exception>
#include <memory>

#include "hanppie/hanppie.hpp"

using namespace hanppie::ctrl;

int main(int argc, char** argv) {
    if (argc != 1 && argc != 3) {
        std::fprintf(stderr, "usage: %s [<robot-ip> <appid>]\n", argv[0]);
        return 2;
    }
    try {
        // Leaving this scope stops all motion and exits control mode.
        std::unique_ptr<RobotCtrl> rm(argc == 3 ? new RobotCtrl(argv[1], argv[2]) : new RobotCtrl());
        rm->led_ctrl.set_top_led(rm_define::armor_all, 0, 255, 0, rm_define::effect_always_on);
        rm->chassis_ctrl.set_trans_speed(0.2);
        rm->chassis_ctrl.move_with_time(0, 1);
        std::printf("chassis yaw %.1f degrees\n", rm->chassis_ctrl.get_attitude(rm_define::chassis_yaw));
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
