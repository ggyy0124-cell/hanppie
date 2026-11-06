#include "hanppie/edu.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>

#include "hanppie/connection.hpp"
#include "hanppie/ctrl.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace hanppie {
namespace edu {

namespace {

// Messages are ASCII: Dev-C++ consoles use the system code page, so UTF-8
// Chinese text from this header would print garbled.
void report(const char* message) { std::printf("[robot] %s\n", message); }

void report(const char* what, const std::exception& error) {
    std::printf("[robot] %s: %s\n", what, error.what());
}

// One robot per program; the destructor stops it when the program exits.
class Session {
public:
    ~Session() { close(); }

    bool open() {
        std::lock_guard<std::mutex> guard(lock_);
        if (ctrl_) return true;
        report("looking for the robot...");
        try {
            ctrl_.reset(new ctrl::RobotCtrl());
            ctrl_->chassis_ctrl.set_trans_speed(MOVE_SPEED);
            ctrl_->chassis_ctrl.set_rotate_speed(TURN_SPEED);
        } catch (const TimeoutError&) {
            ctrl_.reset();
            report("no robot found. Turn it on, join its network, close other control apps, "
                   "then try again.");
            return false;
        } catch (const std::exception& error) {
            ctrl_.reset();
            report("cannot connect", error);
            return false;
        }
        report("connected");
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> guard(lock_);
        ctrl_.reset();  // stops, disarms, leaves control mode and closes the session
    }

    bool connected() {
        std::lock_guard<std::mutex> guard(lock_);
        return ctrl_ && ctrl_->robot.connected();
    }

    // Best effort when the console window closes or Ctrl+C is pressed: zero
    // the motion and give the 50 Hz loop time to send it before the process
    // dies. Runs on a system thread while an action may hold lock_, so it only
    // reads the controller pointer, which stays valid until close().
    void emergencyStop() {
        ctrl::RobotCtrl* rm = ctrl_.get();
        if (rm == nullptr) return;
        try {
            rm->robot.disarm();
        } catch (...) {
        }
        sleepSeconds(0.2);
    }

    template <typename Action>
    bool run(const char* name, Action action) {
        std::lock_guard<std::mutex> guard(lock_);
        if (!ctrl_) {
            std::printf("[robot] %s: not connected, call connect() first\n", name);
            return false;
        }
        try {
            action(*ctrl_);
            return true;
        } catch (const std::exception& error) {
            report(name, error);
        }
        try {
            ctrl_->chassis_ctrl.stop();
        } catch (...) {
        }
        return false;
    }

private:
    std::mutex lock_;
    std::unique_ptr<ctrl::RobotCtrl> ctrl_;
};

Session& session() {
    static Session instance;
    return instance;
}

#ifdef _WIN32
BOOL WINAPI consoleClosing(DWORD) {
    session().emergencyStop();
    return FALSE;  // let Windows end the process as usual
}

void watchConsole() {
    static bool installed = false;
    if (!installed) installed = ::SetConsoleCtrlHandler(consoleClosing, TRUE) != 0;
}
#else
void watchConsole() {}
#endif

bool checkRange(const char* name, double value, double limit) {
    if (value >= 0 && value <= limit) return true;
    std::printf("[robot] %s: use a value from 0 to %g\n", name, limit);
    return false;
}

bool drive(const char* name, double seconds, double direction) {
    if (!checkRange(name, seconds, MAX_SECONDS)) return false;
    return session().run(name, [&](ctrl::RobotCtrl& rm) {
        rm.chassis_ctrl.move_with_time(direction, seconds);
    });
}

bool turn(const char* name, double degrees, int direction) {
    if (!checkRange(name, degrees, MAX_DEGREES)) return false;
    return session().run(name, [&](ctrl::RobotCtrl& rm) {
        rm.chassis_ctrl.rotate_with_degree(direction, degrees);
    });
}

}  // namespace

bool connect() {
    watchConsole();
    return session().open();
}

void disconnect() { session().close(); }

bool connected() { return session().connected(); }

bool forward(double seconds) { return drive("forward", seconds, 0); }
bool backward(double seconds) { return drive("backward", seconds, 180); }
bool left(double seconds) { return drive("left", seconds, -90); }
bool right(double seconds) { return drive("right", seconds, 90); }

bool turnLeft(double degrees) { return turn("turnLeft", degrees, ctrl::rm_define::anticlockwise); }
bool turnRight(double degrees) { return turn("turnRight", degrees, ctrl::rm_define::clockwise); }

bool stop() {
    return session().run("stop", [](ctrl::RobotCtrl& rm) { rm.chassis_ctrl.stop(); });
}

bool beep() {
    return session().run("beep", [](ctrl::RobotCtrl& rm) {
        rm.media_ctrl.play_sound(ctrl::rm_define::media_sound_solmization_1A);
    });
}

void wait(double seconds) {
    if (checkRange("wait", seconds, MAX_SECONDS)) sleepSeconds(seconds);
}

void wait(int seconds) { wait(static_cast<double>(seconds)); }

}  // namespace edu
}  // namespace hanppie
