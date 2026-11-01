// The App's bundled Lab presets ported to the Lab-style API. Each function
// follows shared/src/commonMain/composeResources/files/presets/<name>/script.py
// step for step; only the differences listed below are deliberate.
//
// - Lab robot_ctrl.set_mode(robot_mode_free) has no App protocol equivalent
//   yet; the presets run in the App control mode.
// - The marquee light effect is not available yet; friday_disco flashes instead.
// - music_sprinkler emulates gimbal_ctrl.compound_motion_ctrl on the host.
// - friday_disco plays its soundtrack only when given the preset's
//   audio/0_friday_night.opus file (Git LFS). Lab ships it with the script;
//   here it goes through the speaker upload, which takes at most 64 KB, so the
//   clip plays as consecutive segments with a short gap between them.
// - log_ctrl.print_msg prints to the terminal.
//
// Usage: hanppie_presets <robot-ip> <appid> <preset> [audio.opus]
//        hanppie_presets --list
#include <atomic>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hanppie/hanppie.hpp"

using namespace hanppie::ctrl;

namespace {

const double PI = 3.14159265358979323846;
const double BEAT = 0.28;
// Lab NOTES tuple shared by most presets: Do Re Mi Fa Sol La Si and high Do.
const std::uint32_t NOTES[8] = {
    rm_define::media_sound_solmization_1C, rm_define::media_sound_solmization_1D,
    rm_define::media_sound_solmization_1E, rm_define::media_sound_solmization_1F,
    rm_define::media_sound_solmization_1G, rm_define::media_sound_solmization_1A,
    rm_define::media_sound_solmization_1B, rm_define::media_sound_solmization_2C,
};

std::string soundtrack;  // friday_disco audio file, if given

void print_msg(const std::string& message) {
    std::printf("%s\n", message.c_str());
    std::fflush(stdout);
}

void sleep(double seconds) { RobotCtrl::sleep(seconds); }

void setAll(RobotCtrl& rm, int r, int g, int b, int effect) {
    rm.led_ctrl.set_led(rm_define::armor_all, r, g, b, effect);
}

// Plays each note under a steady light, one beat apart.
void chime(RobotCtrl& rm, const int* notes, int count, int r, int g, int b) {
    for (int i = 0; i < count; ++i) {
        setAll(rm, r, g, b, rm_define::effect_always_on);
        rm.media_ctrl.play_sound(NOTES[notes[i]]);
        sleep(BEAT);
    }
}

// 拆弹倒计时
void bombSquad(RobotCtrl& rm) {
    print_msg("演习开始：发现倒计时装置");
    for (int phase = 0; phase < 3; ++phase) {
        print_msg("正在排查第 " + std::to_string(phase + 1) + " 组线路");
        double interval = 0.7 - phase * 0.2;
        for (int tick = 0; tick < 5; ++tick) {
            setAll(rm, 255, 20, 10, rm_define::effect_always_on);
            rm.media_ctrl.play_sound(NOTES[0]);
            sleep(interval / 2);
            setAll(rm, 20, 0, 0, rm_define::effect_always_on);
            sleep(interval / 2);
        }
    }
    print_msg("最后一根线……");
    setAll(rm, 0, 0, 0, rm_define::effect_always_off);
    sleep(1.5);
    print_msg("解除成功！演习结束");
    const int finale[4] = {0, 2, 4, 7};
    chime(rm, finale, 4, 20, 220, 90);
    sleep(1);
}

// 好奇哨兵
void curiousSentry(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(60);
    setAll(rm, 40, 120, 255, rm_define::effect_breath);
    for (int round = 1; round <= 3; ++round) {
        std::string prefix = "Sentry scan " + std::to_string(round) + "/3: ";
        print_msg(prefix + "right");
        rm.media_ctrl.play_sound(rm_define::media_sound_scanning);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_right, 35);
        print_msg(prefix + "left");
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_left, 70);
        print_msg(prefix + "center");
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_right, 35);
    }
    // Lab waits for the sound to finish; play_sound returns at once here.
    rm.media_ctrl.play_sound(rm_define::media_sound_recognize_success);
    sleep(1);
}

// 萤火虫花园
void fireflyGarden(RobotCtrl& rm) {
    print_msg("黄昏，花园渐渐安静");
    for (int level = 8; level > 0; --level) {
        setAll(rm, level * 20, level * 8, level * 3, rm_define::effect_always_on);
        sleep(0.2);
    }
    const int notes[2] = {4, 7};
    const int tints[2][3] = {{150, 230, 30}, {30, 200, 140}};
    const int levels[10] = {1, 2, 4, 6, 8, 6, 4, 2, 1, 0};
    for (int meeting = 1; meeting <= 3; ++meeting) {
        print_msg("萤火虫的问答 " + std::to_string(meeting));
        for (int i = 0; i < 2; ++i) {
            rm.media_ctrl.play_sound(NOTES[notes[i]]);
            for (int j = 0; j < 10; ++j) {
                setAll(rm, tints[i][0] * levels[j] / 8, tints[i][1] * levels[j] / 8,
                       tints[i][2] * levels[j] / 8, rm_define::effect_always_on);
                sleep(0.12);
            }
        }
    }
    print_msg("晚安，明天见");
    sleep(1);
}

// 招财猫营业中
void luckyCat(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(30);
    rm.gimbal_ctrl.recenter();
    print_msg("小店开门，欢迎光临");
    const int opening[3] = {0, 2, 4};
    chime(rm, opening, 3, 255, 185, 40);
    for (int visitor = 1; visitor <= 3; ++visitor) {
        print_msg("招手迎接第 " + std::to_string(visitor) + " 位客人");
        setAll(rm, 255, 185, 40, rm_define::effect_breath);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_right, 20);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_left, 40);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_right, 20);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_down, 8);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_up, 8);
        const int greeting[2] = {4, 7};
        chime(rm, greeting, 2, 255, 220, 130);
    }
    print_msg("谢谢惠顾");
}

// 月光华尔兹
void moonlightWaltz(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.15);
    rm.chassis_ctrl.set_rotate_speed(30);
    print_msg("月光亮起，舞曲开始");
    const double directions[2] = {90, -90};
    const int steps[2][3] = {{0, 2, 4}, {1, 3, 5}};
    for (int dance = 0; dance < 2; ++dance) {
        for (int i = 0; i < 2; ++i) {
            setAll(rm, 100, 70, 240, rm_define::effect_breath);
            for (int j = 0; j < 3; ++j) {
                rm.media_ctrl.play_sound(NOTES[steps[i][j]]);
                rm.chassis_ctrl.move_with_time(directions[i], BEAT);
            }
        }
        print_msg("转身，向舞伴致意");
        rm.chassis_ctrl.rotate_with_degree(rm_define::clockwise, 25);
        const int bow[3] = {7, 5, 4};
        chime(rm, bow, 3, 180, 130, 255);
        rm.chassis_ctrl.rotate_with_degree(rm_define::anticlockwise, 25);
    }
    print_msg("舞曲终了");
    const int ending[3] = {2, 1, 0};
    chime(rm, ending, 3, 70, 120, 210);
}

// 点球守门员
void penaltyKeeper(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.15);
    rm.chassis_ctrl.set_rotate_speed(30);
    print_msg("点球大战，门将就位；来球由剧情模拟");
    const int whistle[3] = {0, 0, 4};
    chime(rm, whistle, 3, 60, 150, 255);
    for (int shot = 0; shot < 2; ++shot) {
        print_msg("盯住来球……");
        setAll(rm, 255, 170, 30, rm_define::effect_breath);
        sleep(1);
        if (shot == 0) {
            print_msg("判断错了！赶快回扑");
            rm.chassis_ctrl.move_with_time(90, 0.4);
            rm.chassis_ctrl.move_with_time(-90, 0.8);
            rm.chassis_ctrl.move_with_time(90, 0.4);
        } else {
            print_msg("扑向左侧，稳稳接住");
            rm.chassis_ctrl.move_with_time(-90, 0.6);
            const int save[3] = {0, 4, 7};
            chime(rm, save, 3, 30, 230, 90);
            rm.chassis_ctrl.move_with_time(90, 0.6);
        }
    }
    print_msg("比赛结束，守门成功");
    const int ending[3] = {7, 4, 7};
    chime(rm, ending, 3, 30, 230, 90);
}

// 太空发射中心
void spaceLaunch(RobotCtrl& rm) {
    for (int remaining = 5; remaining > 0; --remaining) {
        print_msg("发射倒计时 " + std::to_string(remaining));
        setAll(rm, 255, 35, 30, rm_define::effect_always_on);
        rm.media_ctrl.play_sound(NOTES[0]);
        sleep(0.6);
    }
    print_msg("点火，升空！");
    const int scale[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    for (int launch = 0; launch < 2; ++launch) chime(rm, scale, 8, 255, 100, 15);
    print_msg("进入轨道，太阳能板展开");
    setAll(rm, 20, 90, 255, rm_define::effect_breath);
    sleep(2);
    const int orbit[4] = {7, 4, 2, 7};
    chime(rm, orbit, 4, 180, 210, 255);
}

// 环游小火车
void stationTrain(RobotCtrl& rm) {
    rm.chassis_ctrl.set_trans_speed(0.15);
    rm.chassis_ctrl.set_rotate_speed(30);
    print_msg("各位乘客请坐好，小火车出发");
    const int departure[3] = {0, 4, 7};
    chime(rm, departure, 3, 255, 190, 40);
    const char* stations[4] = {"森林站", "海边站", "星空站", "家乡站"};
    const int colors[4][3] = {{50, 200, 80}, {30, 150, 255}, {150, 80, 255}, {255, 150, 50}};
    for (int i = 0; i < 4; ++i) {
        const int* color = colors[i];
        print_msg(std::string("下一站：") + stations[i]);
        for (int sleeper = 0; sleeper < 3; ++sleeper) {
            setAll(rm, color[0], color[1], color[2], rm_define::effect_always_on);
            rm.media_ctrl.play_sound(NOTES[sleeper % 2]);
            rm.chassis_ctrl.move_with_time(0, BEAT);
        }
        print_msg(std::string("到站停靠：") + stations[i]);
        const int arrival[2] = {4, 2};
        chime(rm, arrival, 2, color[0], color[1], color[2]);
        sleep(0.5);
        rm.chassis_ctrl.rotate_with_degree(rm_define::clockwise, 90);
    }
    print_msg("环游结束，请带好随身物品");
}

// 深夜铁板烧
void streetChef(RobotCtrl& rm) {
    print_msg("深夜小摊，热锅开张");
    for (int heat = 1; heat <= 8; ++heat) {
        setAll(rm, heat * 28, heat * 8, 0, rm_define::effect_always_on);
        sleep(0.2);
    }
    for (int order = 0; order < 2; ++order) {
        print_msg("新订单：切菜备料");
        for (int chop = 0; chop < 6; ++chop) {
            rm.media_ctrl.play_sound(NOTES[chop % 2 == 0 ? 0 : 2]);
            sleep(0.18);
        }
        print_msg("大火翻炒，准备出锅");
        for (int toss = 0; toss < 5; ++toss) {
            setAll(rm, 255, 70, 10, rm_define::effect_always_on);
            sleep(0.15);
            setAll(rm, 80, 15, 0, rm_define::effect_always_on);
            sleep(0.15);
        }
        print_msg("出餐，请慢用");
        const int served[2] = {4, 7};
        chime(rm, served, 2, 255, 210, 100);
        sleep(0.8);
    }
    print_msg("最后一位客人离开，收摊");
}

// 路口协管员
void trafficOfficer(RobotCtrl& rm) {
    rm.gimbal_ctrl.set_rotate_speed(30);
    rm.gimbal_ctrl.recenter();
    for (int crossing = 0; crossing < 2; ++crossing) {
        print_msg("红灯：请在停止线后等待");
        setAll(rm, 255, 30, 20, rm_define::effect_always_on);
        sleep(1);
        print_msg("左右观察，确认模拟路口无车");
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_left, 30);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_right, 60);
        rm.gimbal_ctrl.rotate_with_degree(rm_define::gimbal_left, 30);
        print_msg("绿灯：行人通行");
        for (int pedestrian = 0; pedestrian < 4; ++pedestrian) {
            setAll(rm, 30, 220, 80, rm_define::effect_always_on);
            rm.media_ctrl.play_sound(NOTES[4]);
            sleep(0.5);
        }
        print_msg("黄灯：本轮通行结束");
        for (int warning = 0; warning < 3; ++warning) {
            setAll(rm, 255, 160, 20, rm_define::effect_always_on);
            sleep(0.25);
            setAll(rm, 0, 0, 0, rm_define::effect_always_off);
            sleep(0.25);
        }
    }
    print_msg("执勤结束");
}

// Host-side stand-in for Lab gimbal_ctrl.compound_motion_ctrl on the yaw axis:
// a sine sweep of the given amplitude around the current yaw, driven by speed
// feed-forward plus the SDK's closed-loop gain on gimbal telemetry.
class YawSweep {
public:
    YawSweep(RobotCtrl& rm, double period, double amplitude)
        : rm_(rm), period_(period), amplitude_(amplitude),
          center_(rm.gimbal_ctrl.get_axis_angle(rm_define::gimbal_axis_yaw)), stopping_(false),
          thread_(&YawSweep::run, this) {}
    ~YawSweep() { stop(); }

    void stop() {
        if (!thread_.joinable()) return;
        stopping_ = true;
        thread_.join();
        rm_.gimbal_ctrl.stop();
    }

private:
    void run() {
        const double omega = 2 * PI / period_;
        const double start = hanppie::monotonicSeconds();
        try {
            while (!stopping_) {
                double phase = omega * (hanppie::monotonicSeconds() - start);
                double target = center_ + amplitude_ * std::sin(phase);
                double error = angleDifference(target, rm_.gimbal_ctrl.get_axis_angle(rm_define::gimbal_axis_yaw));
                rm_.gimbal_ctrl.rotate_with_speed(amplitude_ * omega * std::cos(phase) + approachRate(error, 120), 0);
                RobotCtrl::sleep(0.05);
            }
        } catch (const std::exception& error) {
            std::fprintf(stderr, "gimbal sweep stopped: %s\n", error.what());
            rm_.gimbal_ctrl.stop();
        }
    }

    RobotCtrl& rm_;
    double period_;
    double amplitude_;
    double center_;
    std::atomic<bool> stopping_;
    std::thread thread_;
};

// Plays a length-prefixed Opus clip of any size through the speaker upload,
// whose start command carries 16-bit sizes: one segment of whole 20 ms packets
// at a time, the next uploaded once the previous has played out.
class Soundtrack {
public:
    Soundtrack(hanppie::Robot& robot, const hanppie::Bytes& encoded)
        : robot_(robot), segments_(split(encoded)), stopping_(false), thread_(&Soundtrack::run, this) {}
    ~Soundtrack() {
        stopping_ = true;
        thread_.join();
    }

private:
    struct Segment {
        hanppie::Bytes encoded;
        double seconds;
    };

    static std::vector<Segment> split(const hanppie::Bytes& encoded) {
        std::vector<Segment> segments;
        Segment current = {hanppie::Bytes(), 0};
        std::size_t offset = 0;
        while (offset + 2 <= encoded.size()) {
            std::size_t size = 2 + (encoded[offset] | (encoded[offset + 1] << 8));
            if (offset + size > encoded.size()) throw std::runtime_error("truncated Opus packet");
            if (current.encoded.size() + size > 0xFFFF) {
                segments.push_back(current);
                current = Segment{hanppie::Bytes(), 0};
            }
            current.encoded.insert(current.encoded.end(), encoded.begin() + offset, encoded.begin() + offset + size);
            current.seconds += 0.02;
            offset += size;
        }
        if (!current.encoded.empty()) segments.push_back(current);
        return segments;
    }

    void run() {
        try {
            for (std::size_t i = 0; i < segments_.size() && !stopping_; ++i) {
                robot_.audio.playOpus(segments_[i].encoded);
                double until = hanppie::monotonicSeconds() + segments_[i].seconds;
                while (!stopping_ && hanppie::monotonicSeconds() < until) RobotCtrl::sleep(0.05);
            }
        } catch (const std::exception& error) {
            std::fprintf(stderr, "soundtrack stopped: %s\n", error.what());
        }
    }

    hanppie::Robot& robot_;
    std::vector<Segment> segments_;
    std::atomic<bool> stopping_;
    std::thread thread_;
};

struct Note {
    std::uint32_t sound;
    double beats;
};

struct Phrase {
    const char* description;
    bool cruising;
    Note notes[12];
    int count;
};

// 音乐洒水车：《兰花草》四个乐句，每句 8 拍。
void musicSprinkler(RobotCtrl& rm) {
    using namespace rm_define;
    const double beatSeconds = 0.55;
    const double transSpeed = 0.12;
    const Phrase phrases[4] = {
        {"乐句 1/4：我从山中来，带着兰花草 —— 慢速巡航出车，水炮向斜上方扇形扫洒", true,
         {{media_sound_solmization_1A, 0.5}, {media_sound_solmization_2E, 0.5},
          {media_sound_solmization_2E, 0.5}, {media_sound_solmization_2E, 0.5},
          {media_sound_solmization_2E, 1.5}, {media_sound_solmization_2D, 0.5},
          {media_sound_solmization_2C, 0.75}, {media_sound_solmization_2D, 0.25},
          {media_sound_solmization_2C, 0.5}, {media_sound_solmization_1B, 0.5},
          {media_sound_solmization_1A, 2.0}},
         11},
        {"乐句 2/4：种在小园中，希望花开早 —— 扇形水幕向两侧均匀挥洒", true,
         {{media_sound_solmization_1A, 0.5}, {media_sound_solmization_1A, 0.5},
          {media_sound_solmization_1A, 0.5}, {media_sound_solmization_1A, 0.5},
          {media_sound_solmization_1A, 1.5}, {media_sound_solmization_1G, 0.5},
          {media_sound_solmization_1E, 0.5}, {media_sound_solmization_1G, 0.5},
          {media_sound_solmization_1G, 0.5}, {media_sound_solmization_1FSharp, 0.5},
          {media_sound_solmization_1E, 2.0}},
         11},
        {"乐句 3/4：一日看三回，看得花时过 —— 到位停稳，水炮继续大角度漫洒", false,
         {{media_sound_solmization_2E, 0.5}, {media_sound_solmization_2A, 0.5},
          {media_sound_solmization_2A, 0.5}, {media_sound_solmization_2G, 0.5},
          {media_sound_solmization_2E, 1.5}, {media_sound_solmization_2D, 0.5},
          {media_sound_solmization_2C, 0.5}, {media_sound_solmization_2D, 0.5},
          {media_sound_solmization_2C, 0.5}, {media_sound_solmization_1B, 0.5},
          {media_sound_solmization_1A, 1.0}, {media_sound_solmization_1E, 1.0}},
         12},
        {"乐句 4/4：兰花却依然，苞也无一个 —— 水幕挥洒，准备关水收工", false,
         {{media_sound_solmization_1E, 0.5}, {media_sound_solmization_2C, 0.5},
          {media_sound_solmization_2C, 0.5}, {media_sound_solmization_1B, 0.5},
          {media_sound_solmization_1A, 1.5}, {media_sound_solmization_1E, 0.5},
          {media_sound_solmization_2D, 0.75}, {media_sound_solmization_2C, 0.25},
          {media_sound_solmization_1B, 0.5}, {media_sound_solmization_1G, 0.5},
          {media_sound_solmization_1A, 2.0}},
         11},
    };

    rm.chassis_ctrl.set_trans_speed(transSpeed);
    rm.gimbal_ctrl.set_rotate_speed(30);
    rm.gimbal_ctrl.recenter();
    rm.gimbal_ctrl.rotate_with_degree(gimbal_up, 20);
    print_msg("早班出车：水炮斜向上就位，音乐响起，请避让");
    rm.led_ctrl.set_top_led(armor_top_all, 255, 160, 20, effect_breath);
    rm.led_ctrl.set_bottom_led(armor_bottom_all, 0, 160, 255, effect_always_on);
    sleep(1);
    {
        // 周期 4.4 秒、左右各 35 度，对应 compound_motion_ctrl(enable, yaw, 4.4, 35)。
        YawSweep sweep(rm, 4.4, 35);
        for (int i = 0; i < 4; ++i) {
            print_msg(phrases[i].description);
            if (phrases[i].cruising) {
                rm.chassis_ctrl.move(0);
            } else {
                // Lab stops the cruise with set_trans_speed(0).
                rm.chassis_ctrl.stop();
            }
            for (int j = 0; j < phrases[i].count; ++j) {
                rm.media_ctrl.play_sound(phrases[i].notes[j].sound);
                sleep(phrases[i].notes[j].beats * beatSeconds);
            }
        }
        print_msg("到站关水：停车收工，水炮归位");
        rm.led_ctrl.set_bottom_led(armor_bottom_all, 0, 0, 0, effect_always_off);
        rm.led_ctrl.set_top_led(armor_top_all, 30, 220, 160, effect_breath);
        sleep(1);
    }
    rm.chassis_ctrl.stop();
}

// 周五狂欢舞. Lab's effect_marquee is replaced by effect_flash.
void fridayDisco(RobotCtrl& rm) {
    using namespace rm_define;
    ChassisCtrl& chassis = rm.chassis_ctrl;
    GimbalCtrl& gimbal = rm.gimbal_ctrl;
    const int marquee = effect_flash;

    chassis.set_trans_speed(0.40);
    chassis.set_rotate_speed(360);
    gimbal.set_rotate_speed(180);
    gimbal.recenter();

    // The 88.16 s soundtrack plays alongside the dance.
    std::unique_ptr<Soundtrack> music;
    if (!soundtrack.empty()) {
        std::ifstream file(soundtrack.c_str(), std::ios::binary);
        hanppie::Bytes audio((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        if (audio.empty()) throw std::runtime_error("cannot read " + soundtrack);
        music.reset(new Soundtrack(rm.robot, audio));
    }

    // 0.00s - 14.04s: 开场站桩 + 自带干冰喷雾 + 主歌律动
    print_msg("Beck Martin: Ready!");
    setAll(rm, 255, 255, 255, effect_always_on);
    gimbal.rotate_with_degree(gimbal_up, 15);
    sleep(0.79);

    print_msg("启动自带干冰烟雾机: 噗—— (Dry ice smoke blast!)");
    setAll(rm, 255, 255, 255, effect_flash);
    for (int i = 0; i < 5; ++i) {
        chassis.move_with_time(0, 0.08);
        chassis.move_with_time(180, 0.08);
    }
    sleep(0.90);

    print_msg(">>> Techno Beat Drop! Club X Factor 营业！<<<");
    setAll(rm, 255, 0, 150, marquee);
    for (int i = 0; i < 3; ++i) {
        gimbal.rotate_with_degree(gimbal_down, 8);
        gimbal.rotate_with_degree(gimbal_up, 8);
        sleep(0.20);
    }
    sleep(0.63);

    print_msg("Beck: I was working all week...");
    setAll(rm, 0, 200, 255, effect_breath);
    for (int i = 0; i < 2; ++i) {
        chassis.move_with_time(-90, 0.35);
        gimbal.rotate_with_degree(gimbal_down, 6);
        gimbal.rotate_with_degree(gimbal_up, 6);
    }
    chassis.move_with_time(-90, 0.35);
    sleep(1.05);

    print_msg("Beck: ...nothing exciting");
    gimbal.rotate_with_degree(gimbal_left, 18);
    gimbal.rotate_with_degree(gimbal_right, 36);
    gimbal.rotate_with_degree(gimbal_left, 18);
    sleep(1.26);

    print_msg("Beck: I was waiting all week...");
    setAll(rm, 0, 200, 255, effect_breath);
    for (int i = 0; i < 2; ++i) {
        chassis.move_with_time(90, 0.35);
        gimbal.rotate_with_degree(gimbal_down, 6);
        gimbal.rotate_with_degree(gimbal_up, 6);
    }
    chassis.move_with_time(90, 0.35);
    sleep(0.95);

    print_msg("Beck: ...so boring");
    setAll(rm, 50, 100, 255, effect_breath);
    gimbal.rotate_with_degree(gimbal_down, 14);
    sleep(2.08);

    // 14.04s - 39.08s: 周五降临 + 健身教练律动滑步
    print_msg("Beck: But tonight I will go out, because it is Friday night!");
    setAll(rm, 255, 50, 200, marquee);
    chassis.move_with_time(0, 0.35);
    gimbal.rotate_with_degree(gimbal_up, 14);
    for (int i = 0; i < 5; ++i) {
        chassis.move_with_time(-90, 0.28);
        gimbal.rotate_with_degree(gimbal_right, 16);
        chassis.move_with_time(90, 0.28);
        gimbal.rotate_with_degree(gimbal_left, 16);
    }
    chassis.move_with_time(180, 0.35);
    sleep(0.24);

    print_msg("Beck: I wanna keep moving all night!");
    setAll(rm, 255, 140, 0, marquee);
    for (int i = 0; i < 5; ++i) {
        chassis.move_with_time(-90, 0.24);
        gimbal.rotate_with_degree(gimbal_up, 7);
        chassis.move_with_time(90, 0.24);
        gimbal.rotate_with_degree(gimbal_down, 7);
    }
    sleep(0.18);

    print_msg("Beck: I wanna dance all night, all night!");
    for (int i = 0; i < 5; ++i) {
        gimbal.rotate_with_degree(gimbal_up, 9);
        chassis.move_with_time(0, 0.10);
        gimbal.rotate_with_degree(gimbal_down, 9);
        chassis.move_with_time(180, 0.10);
    }
    sleep(1.12);

    print_msg("Beck: And I wanna drink all night...");
    setAll(rm, 255, 215, 0, effect_breath);
    chassis.move_with_time(-90, 0.35);
    gimbal.rotate_with_degree(gimbal_up, 8);
    chassis.move_with_time(-90, 0.35);
    gimbal.rotate_with_degree(gimbal_down, 8);
    chassis.move_with_time(90, 0.25);
    sleep(0.81);

    print_msg("Beck: ...And I wanna party all night!");
    chassis.move_with_time(90, 0.35);
    gimbal.rotate_with_degree(gimbal_up, 8);
    chassis.move_with_time(90, 0.35);
    gimbal.rotate_with_degree(gimbal_down, 8);
    chassis.move_with_time(-90, 0.25);
    sleep(0.97);

    print_msg("Beck: ...And I wanna enjoy this night!");
    for (int i = 0; i < 3; ++i) {
        chassis.move_with_time(-90, 0.20);
        gimbal.rotate_with_degree(gimbal_up, 6);
        chassis.move_with_time(90, 0.20);
        gimbal.rotate_with_degree(gimbal_down, 6);
    }
    sleep(0.08);

    // 39.08s - 64.12s: 周五高潮 + Everybody Hands Up!
    print_msg("Beck: Because it is Friday... Friday night!");
    setAll(rm, 255, 0, 100, effect_flash);
    chassis.move_with_time(0, 0.35);
    gimbal.rotate_with_degree(gimbal_up, 15);
    for (int i = 0; i < 3; ++i) {
        chassis.move_with_time(-90, 0.26);
        gimbal.rotate_with_degree(gimbal_down, 8);
        chassis.move_with_time(90, 0.26);
        gimbal.rotate_with_degree(gimbal_up, 8);
    }
    chassis.move_with_time(180, 0.35);
    sleep(0.97);

    print_msg("Beck: Everybody hands up!!");
    setAll(rm, 255, 215, 0, effect_always_on);
    gimbal.rotate_with_degree(gimbal_up, 20);
    for (int i = 0; i < 5; ++i) {
        chassis.move_with_time(-90, 0.20);
        chassis.move_with_time(90, 0.20);
        gimbal.rotate_with_degree(gimbal_down, 6);
        gimbal.rotate_with_degree(gimbal_up, 6);
    }
    sleep(0.67);

    print_msg("Beck: It's all right, keep moving closer!");
    setAll(rm, 255, 0, 255, marquee);
    chassis.move_with_time(0, 0.35);
    gimbal.rotate_with_degree(gimbal_left, 18);
    gimbal.rotate_with_degree(gimbal_right, 36);
    gimbal.rotate_with_degree(gimbal_left, 18);
    for (int i = 0; i < 3; ++i) {
        chassis.move_with_time(-90, 0.24);
        chassis.move_with_time(90, 0.24);
    }
    chassis.move_with_time(180, 0.35);
    sleep(1.65);

    print_msg("Beck: Party till the morning, buy you shots and drinks!");
    setAll(rm, 0, 255, 200, marquee);
    for (int i = 0; i < 10; ++i) {
        chassis.move_with_time(-90, 0.26);
        gimbal.rotate_with_degree(gimbal_up, 7);
        chassis.move_with_time(90, 0.26);
        gimbal.rotate_with_degree(gimbal_down, 7);
    }
    sleep(1.52);

    // 64.12s - 88.16s: 狂欢大回旋 + Nicole 尖叫 Dry Ice + 晋级谢幕
    print_msg("Beck: Dancing all night long, kissing nonstop!");
    setAll(rm, 255, 100, 0, marquee);
    for (int i = 0; i < 5; ++i) {
        chassis.move_with_time(-90, 0.25);
        gimbal.rotate_with_degree(gimbal_up, 8);
        gimbal.rotate_with_degree(gimbal_down, 8);
        chassis.move_with_time(90, 0.25);
        gimbal.rotate_with_degree(gimbal_up, 8);
        gimbal.rotate_with_degree(gimbal_down, 8);
    }
    sleep(1.01);

    print_msg("Beck: Because it's Friday night!");
    setAll(rm, 255, 0, 255, effect_flash);
    gimbal.rotate_with_degree(gimbal_up, 18);
    chassis.move_with_time(0, 0.30);
    for (int i = 0; i < 4; ++i) {
        chassis.move_with_time(-90, 0.25);
        gimbal.rotate_with_degree(gimbal_down, 7);
        chassis.move_with_time(90, 0.25);
        gimbal.rotate_with_degree(gimbal_up, 7);
    }
    chassis.move_with_time(180, 0.30);
    sleep(1.17);

    print_msg("Nicole 站上评委席热舞！Nicole 尖叫: Woo! Dry ice!! Simon 破天荒起立摇摆！");
    setAll(rm, 255, 255, 255, effect_flash);
    for (int i = 0; i < 2; ++i) chassis.rotate_with_degree(clockwise, 360);
    setAll(rm, 255, 0, 255, marquee);
    gimbal.rotate_with_degree(gimbal_up, 18);
    for (int i = 0; i < 4; ++i) {
        chassis.move_with_time(-90, 0.22);
        chassis.move_with_time(90, 0.22);
        gimbal.rotate_with_degree(gimbal_down, 7);
        gimbal.rotate_with_degree(gimbal_up, 7);
    }
    sleep(2.24);

    print_msg("Beck: Friday night climax!");
    for (int i = 0; i < 4; ++i) {
        chassis.move_with_time(0, 0.12);
        gimbal.rotate_with_degree(gimbal_up, 8);
        chassis.move_with_time(180, 0.12);
        gimbal.rotate_with_degree(gimbal_down, 8);
    }
    sleep(0.75);

    print_msg("全场起立欢呼！Beck Martin 晋级训练营 (Boot Camp)！鞠躬致谢！");
    setAll(rm, 255, 255, 255, effect_always_on);
    gimbal.rotate_with_degree(gimbal_down, 20);
    sleep(0.70);
}

struct Preset {
    const char* name;  // preset folder name
    const char* title;
    void (*run)(RobotCtrl&);
    bool recenter;  // the Lab script recenters the gimbal when it ends
};

const Preset PRESETS[] = {
    {"bomb_squad", "拆弹倒计时", bombSquad, false},
    {"curious_sentry", "好奇哨兵", curiousSentry, true},
    {"firefly_garden", "萤火虫花园", fireflyGarden, false},
    {"friday_disco", "周五狂欢舞", fridayDisco, true},
    {"lucky_cat", "招财猫营业中", luckyCat, false},
    {"moonlight_waltz", "月光华尔兹", moonlightWaltz, false},
    {"music_sprinkler", "音乐洒水车", musicSprinkler, true},
    {"penalty_keeper", "点球守门员", penaltyKeeper, false},
    {"space_launch", "太空发射中心", spaceLaunch, false},
    {"station_train", "环游小火车", stationTrain, false},
    {"street_chef", "深夜铁板烧", streetChef, false},
    {"traffic_officer", "路口协管员", trafficOfficer, false},
};
const std::size_t PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

// The Lab scripts' finally blocks: lights off and, for some, gimbal recentered.
// Motion is stopped by RobotCtrl's destructor.
void finish(RobotCtrl& rm, const Preset& preset) {
    rm.led_ctrl.turn_off(rm_define::armor_all);
    if (preset.recenter) rm.gimbal_ctrl.recenter();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--list") {
        for (std::size_t i = 0; i < PRESET_COUNT; ++i) std::printf("%-16s %s\n", PRESETS[i].name, PRESETS[i].title);
        return 0;
    }
    if (argc != 4 && argc != 5) {
        std::fprintf(stderr, "usage: %s <robot-ip> <appid> <preset> [audio.opus]\n       %s --list\n", argv[0],
                     argv[0]);
        return 2;
    }
    const Preset* preset = 0;
    for (std::size_t i = 0; i < PRESET_COUNT; ++i) {
        if (std::string(PRESETS[i].name) == argv[3]) preset = &PRESETS[i];
    }
    if (!preset) {
        std::fprintf(stderr, "unknown preset %s; see --list\n", argv[3]);
        return 2;
    }
    if (argc == 5) soundtrack = argv[4];
    try {
        hanppie::Robot robot(argv[1], argv[2]);
        if (!robot.initialize()) {
            std::fprintf(stderr, "error: robot did not answer\n");
            return 1;
        }
        std::printf("preset %s: %s\n", preset->name, preset->title);
        {
            // Leaving this scope stops all motion and exits control mode.
            RobotCtrl rm(robot);
            try {
                preset->run(rm);
            } catch (...) {
                try {
                    finish(rm, *preset);
                } catch (...) {
                }
                throw;
            }
            finish(rm, *preset);
        }
        robot.close();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
