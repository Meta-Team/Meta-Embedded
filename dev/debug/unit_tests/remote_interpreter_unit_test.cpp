//
// Created by liuzikai on 2018/8/6.
//

// [遥控器拨轮的数据解析【RoboMaster论坛-科技宅天堂】](https://bbs.robomaster.com/thread-8123-1-1.html)

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "remote_interpreter.h"

using namespace chibios_rt;

class RemoteFeedbackThread : public chibios_rt::BaseStaticThread <512> {
    void main() final {
        setName("remote_fb");
        while(!shouldTerminate()) {
            Shell::printf("ch0 ch1 ch2 ch3 s1 s2 mouse_x mouse_y mouse_z L R" SHELL_NEWLINE_STR);
            Shell::printf("%3d %3d %3d %3d %2d %2d %7d %7d %7d %1d %1d" SHELL_NEWLINE_STR,
                          (int) (Remote::rc.ch0 * 100), (int) (Remote::rc.ch1 * 100),
                          (int) (Remote::rc.ch2 * 100), (int) (Remote::rc.ch3 * 100),
                          Remote::rc.s1, Remote::rc.s2,
                          Remote::mouse.x, Remote::mouse.y, Remote::mouse.z,
                          Remote::mouse.press_left, Remote::mouse.press_right);
            Shell::printf(SHELL_NEWLINE_STR);
            Shell::printf("W S A D SHIFT CTRL Q E R F G Z X C V B" SHELL_NEWLINE_STR);
            Shell::printf("%d %d %d %d %5d %4d %d %d %d %d %d %d %d %d %d %d" SHELL_NEWLINE_STR,
                          Remote::key.w, Remote::key.s, Remote::key.a, Remote::key.d, Remote::key.shift, Remote::key.ctrl,
                          Remote::key.q, Remote::key.e, Remote::key.r, Remote::key.f, Remote::key.g, Remote::key.z,
                          Remote::key.x, Remote::key.c, Remote::key.v, Remote::key.b);
            Shell::printf(SHELL_NEWLINE_STR SHELL_NEWLINE_STR);
            sleep(TIME_MS2I(500));
        }
    }
} remoteFeedbackThread;

int main() {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    Remote::start_receive();

    remoteFeedbackThread.start(NORMALPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
        // enter an infinite loop, so we set the priority to lowest
        // before quitting, to let other threads run normally
        BaseThread::setPriority(1);
#endif
    return 0;
}
