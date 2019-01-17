//
// Created by liuzikai on 2019-01-17.
//


#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "judge_system.h"

using namespace chibios_rt;

void uart_received_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLock();

    rc.ch0 = (((rx_buf[0] | rx_buf[1] << 8) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch1 = (((rx_buf[1] >> 3 | rx_buf[2] << 5) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch2 = (((rx_buf[2] >> 6 | rx_buf[3] << 2 | rx_buf[4] << 10) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch3 = (((rx_buf[4] >> 1 | rx_buf[5] << 7) & 0x07FF) - 1024.0f) / 660.0f;

    rc.s1 = (rc_status_t) ((rx_buf[5] >> 6) & 0x0003);
    rc.s2 = (rc_status_t) ((rx_buf[5] >> 4) & 0x0003);

    mouse.x = (int16_t) (rx_buf[6] | rx_buf[7] << 8);
    mouse.y = (int16_t) (rx_buf[8] | rx_buf[9] << 8);
    mouse.z = (int16_t) (rx_buf[10] | rx_buf[11] << 8);

    mouse.press_left = (bool) rx_buf[12];
    mouse.press_right = (bool) rx_buf[13];

    key._key_code = static_cast<uint16_t>(rx_buf[14] | rx_buf[15] << 8);

    chSysUnlock();

    uartStartReceive(uartp, REMOTE_DATA_BUF_SIZE, rx_buf);
}

static constexpr UARTConfig remoteUartConfig = {
        nullptr,
        nullptr,
        uart_received_callback, // callback function when the buffer is filled
        nullptr,
        nullptr,
        921600, // speed
        USART_CR1_PCE,
        0,
        0,
};

// Print the remote info
static void cmd_remote_print(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "p");
        return;
    }
    chprintf(chp, "ch0 ch1 ch2 ch3 s1 s2 mouse_x mouse_y mouse_z L R" SHELL_NEWLINE_STR);
    chprintf(chp, "%3d %3d %3d %3d %2d %2d %7d %7d %7d %1d %1d" SHELL_NEWLINE_STR,
             (int) (Remote::rc.ch0 * 100), (int) (Remote::rc.ch1 * 100),
             (int) (Remote::rc.ch2 * 100), (int) (Remote::rc.ch3 * 100),
             Remote::rc.s1, Remote::rc.s2,
             Remote::mouse.x, Remote::mouse.y, Remote::mouse.z,
             Remote::mouse.press_left, Remote::mouse.press_right);
    chprintf(chp, SHELL_NEWLINE_STR);
    chprintf(chp, "W S A D SHIFT CTRL Q E R F G Z X C V B" SHELL_NEWLINE_STR);
    chprintf(chp, "%d %d %d %d %5d %4d %d %d %d %d %d %d %d %d %d %d" SHELL_NEWLINE_STR,
             Remote::key.w, Remote::key.s, Remote::key.a, Remote::key.d, Remote::key.shift, Remote::key.ctrl,
             Remote::key.q, Remote::key.e, Remote::key.r, Remote::key.f, Remote::key.g, Remote::key.z,
             Remote::key.x, Remote::key.c, Remote::key.v, Remote::key.b);
    chprintf(chp, SHELL_NEWLINE_STR SHELL_NEWLINE_STR);
}

ShellCommand remoteShellCommands[] = {
        {"p", cmd_remote_print},
        {nullptr, nullptr}
};

int main() {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    Remote::start_receive();

    Shell::addCommands(remoteShellCommands);

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
