// RoboMaster S1 programming challenge cards written with the Lab-style API.
// Each card follows the official answer sheet; card numbers match the deck.
// Cards that need events (clap, collision, armor hit), free mode, gimbal
// marquee or single LEDs are not available over the App protocol yet.
//
// Usage: hanppie_challenges <robot-ip> <appid> <card>
//        hanppie_challenges --list
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>

#include "hanppie/hanppie.hpp"

using namespace hanppie::ctrl;

namespace {

// Do Re Mi Fa Sol La Si in the first octave (rm_define media_sound_solmization_1C..1B).
const std::uint32_t SCALE[7] = {0x107, 0x109, 0x10B, 0x10C, 0x10E, 0x110, 0x112};

double now() { return hanppie::monotonicSeconds(); }

// 1. Walk a square: back, left, forward and right one metre each.
void square(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.5);
    rm.chassis_ctrl.move_with_distance(180, 1);
    rm.chassis_ctrl.move_with_distance(-90, 1);
    rm.chassis_ctrl.move_with_distance(0, 1);
    rm.chassis_ctrl.move_with_distance(90, 1);
}

// 2. Cross steps: out and back in each of the four directions.
void cross(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.5);
    const double directions[4] = {0, 180, -90, 90};
    for (int i = 0; i < 4; ++i) {
        rm.chassis_ctrl.move_with_distance(directions[i], 0.5);
        rm.chassis_ctrl.move_with_distance(directions[i] > 0 ? directions[i] - 180 : directions[i] + 180, 0.5);
    }
}

// 4. Sing and dance: turn right 90 degrees, then left 180 to pass the origin.
void singAndDance(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(60);
    rm.media_ctrl.play_sound(SCALE[0]);
    rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_right, 90);
    rm.media_ctrl.play_sound(SCALE[2]);
    rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_left, 180);
    rm.media_ctrl.play_sound(SCALE[4]);
    rm.gimbal_ctrl.recenter();
}

// 6. Aim at the red heart marker, then shoot (infrared; use gun_ctrl.fire_once()
//    for a gel bead).
void aimRedHeart(RobotCtrl& rm) {
    rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
    std::printf("show the red heart marker...\n");
    rm.vision_ctrl.cond_wait_marker(rm_define::marker_trans_red_heart);
    if (rm.vision_ctrl.detect_marker_and_aim(rm_define::marker_trans_red_heart)) {
        rm.ir_blaster_ctrl.fire_once();
        rm.media_ctrl.play_sound(rm_define::media_sound_shoot);
    }
}

// 9. Turn the gimbal to the back and strike.
void backStrike(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(120);
    rm.gimbal_ctrl.yaw_ctrl(180);
    rm.ir_blaster_ctrl.fire_once();
    rm.media_ctrl.play_sound(rm_define::media_sound_shoot);
    rm.gimbal_ctrl.recenter();
}

// 10. Sweep a full circle: from -250 to at least +110 degrees, firing on the way.
void sweepFire(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(90);
    rm.gimbal_ctrl.yaw_ctrl(-250);
    rm.gimbal_ctrl.rotate_with_speed(60, 0);
    double nextShot = 0;
    while (rm.gimbal_ctrl.get_axis_angle(rm_define::gimbal_axis_yaw) < 110) {
        if (now() >= nextShot) {
            rm.ir_blaster_ctrl.fire_once();
            nextShot = now() + 0.5;
        }
        RobotCtrl::sleep(0.05);
    }
    rm.gimbal_ctrl.stop();
    rm.gimbal_ctrl.recenter();
}

// 11. Camera operator: record a video while turning around.
void cameraOperator(RobotCtrl& rm) {
    rm.media_ctrl.record(rm_define::record_on);
    rm.chassis_ctrl.set_rotate_speed(45);
    rm.chassis_ctrl.rotate_with_time(rm_define::clockwise, 8);
    rm.media_ctrl.record(rm_define::record_off);
}

// 13. Figure eight: translate while rotating, one circle each way in 2.5 s.
void figureEight(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.5);
    rm.chassis_ctrl.set_rotate_speed(144);
    rm.chassis_ctrl.move_and_rotate(0, rm_define::clockwise);
    RobotCtrl::sleep(2.5);
    rm.chassis_ctrl.move_and_rotate(0, rm_define::anticlockwise);
    RobotCtrl::sleep(2.5);
    rm.chassis_ctrl.stop();
}

// 14. Gimbal as a sensor: drive towards the gimbal heading for 10 s. Turn the
//     gimbal with the App or another program while this runs.
void gimbalSensor(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.3);
    double end = now() + 10;
    while (now() < end) {
        double yaw = rm.gimbal_ctrl.get_axis_angle(rm_define::gimbal_axis_yaw);
        while (yaw > 180) yaw -= 360;
        while (yaw < -180) yaw += 360;
        rm.chassis_ctrl.move(yaw);
        RobotCtrl::sleep(0.1);
    }
    rm.chassis_ctrl.stop();
}

// 15. Gesture photo: keep waiting for the capture pose and take a photo.
void gesturePhoto(RobotCtrl& rm) {
    rm.vision_ctrl.enable_detection(rm_define::vision_detection_pose);
    for (int shots = 0; shots < 3; ++shots) {
        std::printf("make the capture gesture...\n");
        rm.vision_ctrl.cond_wait_pose(rm_define::pose_capture);
        rm.media_ctrl.play_sound(rm_define::media_sound_count_down);
        rm.media_ctrl.capture();
        RobotCtrl::sleep(1);
    }
}

// 16. Motion control. A computer has no phone gyroscope, so the gimbal follows
//     yaw/pitch angles typed on stdin instead; any other input source works too.
void motionControl(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(90);
    std::printf("type \"<yaw> <pitch>\" in degrees, empty line to finish\n");
    std::string line;
    while (std::getline(std::cin, line) && !line.empty()) {
        std::istringstream in(line);
        double yaw = 0, pitch = 0;
        if (!(in >> yaw)) continue;
        in >> pitch;
        rm.gimbal_ctrl.angle_ctrl(yaw, pitch);
    }
    rm.gimbal_ctrl.recenter();
}

// 17. Play the piano: a C major scale and back.
void piano(RobotCtrl& rm) {
    for (int i = 0; i < 7; ++i) {
        rm.media_ctrl.play_sound(SCALE[i]);
        RobotCtrl::sleep(0.4);
    }
    for (int i = 5; i >= 0; --i) {
        rm.media_ctrl.play_sound(SCALE[i]);
        RobotCtrl::sleep(0.4);
    }
}

// 18. Arithmetic: use 1 + 1 as the repeat count.
void arithmetic(RobotCtrl& rm) {
    int repeat = 1 + 1;
    for (int i = 0; i < repeat; ++i) {
        rm.led_ctrl.set_top_led(rm_define::armor_all, 0, 0, 255, rm_define::effect_flash);
        rm.media_ctrl.play_sound(rm_define::media_sound_recognize_success);
        RobotCtrl::sleep(1);
    }
    rm.led_ctrl.set_top_led(rm_define::armor_all, 0, 255, 0, rm_define::effect_always_on);
}

// 19. Turn left: rotate the chassis 90 degrees anticlockwise.
void turnLeft(RobotCtrl& rm) {
    rm.chassis_ctrl.set_rotate_speed(60);
    rm.chassis_ctrl.rotate_with_degree(rm_define::anticlockwise, 90);
}

// 21. Parade: march with the head turned, then look ahead again.
void parade(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(90);
    rm.gimbal_ctrl.yaw_ctrl(-90);
    rm.chassis_ctrl.set_trans_speed(0.3);
    rm.chassis_ctrl.move_with_time(0, 3);
    rm.gimbal_ctrl.yaw_ctrl(0);
}

// 22. Follow the view: keep aiming at the target marker for 30 s.
void followTarget(RobotCtrl& rm) {
    rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
    double end = now() + 30;
    while (now() < end) {
        if (!rm.vision_ctrl.detect_marker_and_aim(rm_define::marker_trans_target)) RobotCtrl::sleep(0.1);
    }
}

// 23. Ten shots in a row.
void tenShots(RobotCtrl& rm) {
    for (int i = 0; i < 10; ++i) {
        rm.media_ctrl.capture();
        RobotCtrl::sleep(1);
    }
}

// 24. Eye test: after seeing the left arrow, turn the gimbal left.
void eyeTest(RobotCtrl& rm) {
    rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
    std::printf("show the left arrow marker...\n");
    rm.vision_ctrl.cond_wait_marker(rm_define::marker_trans_left);
    rm.gimbal_ctrl.set_rotate_speed(60);
    rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_left, 45);
}

// 25. S-shaped reverse: a quarter circle, half circle the other way, quarter circle.
void sReverse(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.3);
    rm.chassis_ctrl.set_rotate_speed(150);
    rm.chassis_ctrl.move_and_rotate(180, rm_define::clockwise);
    RobotCtrl::sleep(0.5);
    rm.chassis_ctrl.move_and_rotate(180, rm_define::anticlockwise);
    RobotCtrl::sleep(1);
    rm.chassis_ctrl.move_and_rotate(180, rm_define::clockwise);
    RobotCtrl::sleep(0.5);
    rm.chassis_ctrl.stop();
}

// 27. Roll the dice: after seeing the dice marker, play a random note 1-6.
void rollDice(RobotCtrl& rm) {
    rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
    std::printf("show the dice marker...\n");
    rm.vision_ctrl.cond_wait_marker(rm_define::marker_trans_dice);
    int face = 1 + std::rand() % 6;
    std::printf("dice: %d\n", face);
    rm.media_ctrl.play_sound(SCALE[face - 1]);
}

// 28. Marker concert: number 1 plays Do and number 2 plays Re, for 30 s.
void markerConcert(RobotCtrl& rm) {
    rm.vision_ctrl.enable_detection(rm_define::vision_detection_marker);
    double end = now() + 30;
    while (now() < end) {
        if (rm.vision_ctrl.check_marker(rm_define::marker_number_zero + 1)) {
            rm.media_ctrl.play_sound(SCALE[0]);
        } else if (rm.vision_ctrl.check_marker(rm_define::marker_number_zero + 2)) {
            rm.media_ctrl.play_sound(SCALE[1]);
        }
        RobotCtrl::sleep(0.5);
    }
}

// 29. Speed up: five steps, adding 0.1 m/s to the 0.5 m/s default each time.
void speedUp(RobotCtrl& rm) {
    double speed = 0.5;
    for (int i = 0; i < 5; ++i) {
        speed += 0.1;
        rm.chassis_ctrl.set_trans_speed(speed);
        rm.chassis_ctrl.move_with_time(0, 1);
    }
}

struct Card {
    int number;
    const char* title;
    void (*run)(RobotCtrl&);
};

const Card CARDS[] = {
    {1, "走个正方形", square},       {2, "十字走位", cross},
    {4, "能歌善舞", singAndDance},   {6, "瞄准红心", aimRedHeart},
    {9, "回马枪", backStrike},       {10, "环绕扫射", sweepFire},
    {11, "摄像师 S1", cameraOperator}, {13, "\"8\" 字形运动", figureEight},
    {14, "云台传感器", gimbalSensor}, {15, "手势拍照", gesturePhoto},
    {16, "体感控制", motionControl}, {17, "弹钢琴", piano},
    {18, "算数学题", arithmetic},    {19, "S1 向左转", turnLeft},
    {21, "阅兵", parade},            {22, "视野跟随", followTarget},
    {23, "十连拍", tenShots},        {24, "视力测试", eyeTest},
    {25, "S 形倒退", sReverse},      {27, "摇骰子", rollDice},
    {28, "标签演奏会", markerConcert}, {29, "加速前进", speedUp},
};
const std::size_t CARD_COUNT = sizeof(CARDS) / sizeof(CARDS[0]);

void list() {
    for (std::size_t i = 0; i < CARD_COUNT; ++i) std::printf("%2d  %s\n", CARDS[i].number, CARDS[i].title);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--list") {
        list();
        return 0;
    }
    if (argc != 4) {
        std::fprintf(stderr, "usage: %s <robot-ip> <appid> <card>\n       %s --list\n", argv[0], argv[0]);
        return 2;
    }
    const Card* card = 0;
    for (std::size_t i = 0; i < CARD_COUNT; ++i) {
        if (CARDS[i].number == std::atoi(argv[3])) card = &CARDS[i];
    }
    if (!card) {
        std::fprintf(stderr, "card %s is not available; see --list\n", argv[3]);
        return 2;
    }
    std::srand(static_cast<unsigned>(std::time(0)));
    try {
        hanppie::Robot robot(argv[1], argv[2]);
        if (!robot.initialize()) {
            std::fprintf(stderr, "error: robot did not answer\n");
            return 1;
        }
        std::printf("card %d: %s\n", card->number, card->title);
        {
            // Leaving this scope stops all motion, disables vision and exits control mode.
            RobotCtrl rm(robot);
            card->run(rm);
        }
        robot.close();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
