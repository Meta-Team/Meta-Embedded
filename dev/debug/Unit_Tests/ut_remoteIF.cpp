//
// Created by liuzikai on 2018/8/6.
//

// [遥控器拨轮的数据解析【RoboMaster论坛-科技宅天堂】](https://bbs.robomaster.com/thread-8123-1-1.html)

#include <scheduler/buzzer_scheduler.h>
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "remote_interpreter.h"

using namespace chibios_rt;

class RemoteFeedbackThread : public chibios_rt::BaseStaticThread <512> {
    void main() final {
        setName("remote_fb");
        while(!shouldTerminate()) {
            Shell::printf("ch0 ch1 ch2 ch3 s1 s2 sw mouse_x mouse_y mouse_z L R" SHELL_NEWLINE_STR);
            Shell::printf("%3d %3d %3d %3d %2d %2d %3d %7d %7d %7d %1d %1d" SHELL_NEWLINE_STR,
                          (int) (Remote::rc.ch0 * 100), (int) (Remote::rc.ch1 * 100),
                          (int) (Remote::rc.ch2 * 100), (int) (Remote::rc.ch3 * 100),
                          Remote::rc.s1, Remote::rc.s2,
                          (int) (Remote::rc.wheel * 100),
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

#if REMOTE_USE_EVENTS

class RemoteEventThread : public chibios_rt::BaseStaticThread<512> {

public:

    event_listener_t s_change_listener;
    static constexpr eventmask_t S_CHANGE_EVENTMASK = (1U << 0U);

    event_listener_t mouse_press_listener;
    static constexpr eventmask_t MOUSE_PRESS_EVENTMASK = (1U << 1U);

    event_listener_t mouse_release_listener;
    static constexpr eventmask_t MOUSE_RELEASE_EVENTMASK = (1U << 2U);

    event_listener_t key_press_listener;
    static constexpr eventmask_t KEY_PRESS_EVENTMASK = (1U << 3U);

    event_listener_t key_release_listener;
    static constexpr eventmask_t KEY_RELEASE_EVENTMASK = (1U << 4U);

    eventmask_t LISTEN_EVENTS = KEY_PRESS_EVENTMASK | KEY_RELEASE_EVENTMASK;

    void main() final {
        setName("remote_evt");

        chEvtRegisterMask(&Remote::s_change_event, &s_change_listener, S_CHANGE_EVENTMASK);
        chEvtRegisterMask(&Remote::mouse_press_event, &mouse_press_listener, MOUSE_PRESS_EVENTMASK);
        chEvtRegisterMask(&Remote::mouse_release_event, &mouse_release_listener, MOUSE_RELEASE_EVENTMASK);
        chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, KEY_PRESS_EVENTMASK);
        chEvtRegisterMask(&Remote::key_release_event, &key_release_listener, KEY_RELEASE_EVENTMASK);

        while (!shouldTerminate()) {
            
            eventmask_t events = chEvtWaitAny(LISTEN_EVENTS);
            
            if (events & S_CHANGE_EVENTMASK) {
                chEvtGetAndClearEvents(S_CHANGE_EVENTMASK);
                LOG("EVENT s change!");
            } 
            
            if (events & MOUSE_PRESS_EVENTMASK) {
                eventflags_t mouse_press_flags = chEvtGetAndClearFlags(&mouse_press_listener);
                LOG("EVENT mouse_press %x !", mouse_press_flags);
            }

            if (events & MOUSE_RELEASE_EVENTMASK) {
                eventflags_t mouse_release_flags = chEvtGetAndClearFlags(&mouse_release_listener);
                LOG("EVENT mouse_release %x !", mouse_release_flags);
            }

            if (events & KEY_PRESS_EVENTMASK) {
                eventflags_t key_press_flags = chEvtGetAndClearFlags(&key_press_listener);
                LOG("EVENT key_press %x !", key_press_flags);
            }

            if (events & KEY_RELEASE_EVENTMASK) {
                eventflags_t key_release_flags = chEvtGetAndClearFlags(&key_release_listener);
                LOG("EVENT key_release %x !", key_release_flags);
            }

        }
    }
} remoteEventThread;

#endif

int main() {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    Remote::start();

    remoteFeedbackThread.start(NORMALPRIO);
    remoteEventThread.start(NORMALPRIO - 1);

    BuzzerSKD::init(LOWPRIO);

    BuzzerSKD::play_sound(BuzzerSKD::sound_startup);

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
