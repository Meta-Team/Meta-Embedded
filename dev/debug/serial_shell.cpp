#include "serial_shell.h"

using namespace chibios_rt;

/**
 * Config of the serial port used by the shell.
 * First parameter is the bitrate.
 * Other three are for parity, stop bits, etc which we don't care.
 */
static SerialConfig shellSerialConfig = {115200, 0, 0, 0};

/**
 * Config of the shell.
 * First parameter is the Serial port.
 * Second parameter is the list of commands, provided in serial_shell_commands.hpp
 */
static const ShellConfig shellConfig = {
        (BaseSequentialStream *) &SD6,
        shellCommands
#if (SHELL_USE_HISTORY == TRUE)
,new char[64],
64
#endif
};

/**
 * Main function of SerialShellThread which will do nothing.
 * This function will never be called, it's here or G++ will be angry.
 * SerialShellThread calls the thread provided by shell instead.
 */
void SerialShellThread::main(void) {
}

/**
 * Function to start the thread.
 * Instead of calling main(), as BaseStaticThread does,
 * it calls the thread provided by ChibiOS shell.
 */
chibios_rt::ThreadReference SerialShellThread::start(tprio_t prio) {
    // Remember to change GPIO pins here
    palSetPadMode(GPIOG, 14, PAL_MODE_ALTERNATE(8));
    palSetPadMode(GPIOG, 9, PAL_MODE_ALTERNATE(8));
    sdStart(&SD6, &shellSerialConfig);
    // Call init provided by shell
    shellInit();

    // Create a thread and name it
    thread_t *shellThreadRef = chThdCreateStatic(
            wa, sizeof(wa), prio,
            shellThread, (void *) &shellConfig);
    chRegSetThreadNameX(shellThreadRef, "shell");

    // Return reference to thread
    return chibios_rt::ThreadReference(shellThreadRef);
}

int sprintf(const char *fmt, ...) {
    va_list ap;
    int formatted_bytes;

    va_start(ap, fmt);
    formatted_bytes = chvprintf((BaseSequentialStream *) &SD6, fmt, ap);
    va_end(ap);

    return formatted_bytes;
}