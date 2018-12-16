//
// Created by liuzikai on 2018/8/6.
//

#include "ch.hpp"
#include "hal.h"

#include "led.hpp"
#include "serial_shell.hpp"
#include "unit_test_common.h"
#include "remote_interpreter.h"

using namespace chibios_rt;

// Thread of blinking leds common for unit tests.
UnitTestBlinkLEDThread blinkLEDThread;

RemoteInterpreter *remote = NULL;

// Print the remote info.
static void cmd_remote_print(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "p");
        return;
    }
    chprintf(chp, "ch0 ch1 ch2 ch3 s1 s2 mouse_x mouse_y mouse_z L R" SHELL_NEWLINE_STR);
    chprintf(chp, "%3d %3d %3d %3d %2d %2d %7d %7d %7d %1d %1d" SHELL_NEWLINE_STR,
           (int) (remote->rc.ch0 * 100), (int) (remote->rc.ch1 * 100),
           (int) (remote->rc.ch2 * 100), (int) (remote->rc.ch3 * 100),
           remote->rc.s1, remote->rc.s2,
           remote->mouse.x, remote->mouse.y, remote->mouse.z,
           remote->mouse.press_left, remote->mouse.press_right);
    chprintf(chp, SHELL_NEWLINE_STR);
    chprintf(chp, "W S A D SHIFT CTRL Q E R F G Z X C V B" SHELL_NEWLINE_STR);
    chprintf(chp, "%d %d %d %d %5d %4d %d %d %d %d %d %d %d %d %d %d" SHELL_NEWLINE_STR,
           remote->key.w, remote->key.s, remote->key.a, remote->key.d, remote->key.shift, remote->key.ctrl,
           remote->key.q, remote->key.e, remote->key.r, remote->key.f, remote->key.g, remote->key.z,
           remote->key.x, remote->key.c, remote->key.v, remote->key.b);
    chprintf(chp, SHELL_NEWLINE_STR SHELL_NEWLINE_STR);
}


// Shell commands to pause and resume the echos.
ShellCommand remoteShellCommands[] = {
        {"p", cmd_remote_print},
        {NULL, NULL}
};

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    serialShell.start(HIGHPRIO);

    // Start led blink thread common for unit test
    blinkLEDThread.start(HIGHPRIO - 1);

    remote = remoteInit();

    shellAddCommands(remoteShellCommands);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
        // enter an infinite loop, so we set the priority to lowest
        // before quitting, to let other threads run normally
        BaseThread::setPriority(1);
#endif
    return 0;
}
