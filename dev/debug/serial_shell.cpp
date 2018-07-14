#include "serial_shell.hpp"

using namespace chibios_rt;

static SerialConfig shellSerialConfig = {115200, 0, 0, 0};

static const ShellConfig shellConfig = {
    (BaseSequentialStream*) &SD1,
    shellCommands
};

void SerialShellThread::main(void) {
    // Do nothing, as this function will never be called.
    // This function is here, otherwise G++ will be angry.
    // SerialShellThread calls the thread provided by shell instead.
}

chibios_rt::ThreadReference SerialShellThread::start(tprio_t prio) {
    // Remember to change GPIO pins here
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, &shellSerialConfig);
    // Call init provided by shell
    shellInit();
    
    // Create a thread and name it
    thread_t *shellThreadRef = chThdCreateStatic(
        wa, sizeof(wa), prio,
        shellThread, (void*) &shellConfig);
    chRegSetThreadNameX(shellThreadRef, "shell");

    // Return reference to thread
    return chibios_rt::ThreadReference(shellThreadRef);
}