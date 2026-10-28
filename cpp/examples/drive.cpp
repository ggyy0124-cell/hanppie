// Drive a RoboMaster S1 forward for one second over the App protocol.
//
// Usage: hanppie_drive <robot-ip> <appid>
#include <chrono>
#include <cstdio>
#include <exception>
#include <thread>

#include "hanppie/hanppie.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <robot-ip> <appid>\n", argv[0]);
        return 2;
    }
    try {
        hanppie::Robot robot(argv[1], argv[2]);
        robot.initialize();
        robot.enterControlMode();
        robot.setLed("all", 0, 255, 0);
        robot.arm();
        // Each call renews a 250 ms lease; stopping the loop stops the robot.
        for (int i = 0; i < 20; ++i) {
            robot.chassis.driveSpeed(0.2, 0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        robot.disarm();
        hanppie::Odometry odometry;
        if (robot.odometry(odometry) && odometry.hasPosition()) {
            std::printf("odometry x=%.3f y=%.3f battery=%d%%\n", odometry.x(), odometry.y(),
                        odometry.batteryPercent);
        }
        robot.close();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
