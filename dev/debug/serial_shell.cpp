#include "serial_shell.hpp"

using namespace chibios_rt;

static SerialConfig shellSerialConfig = {115200, 0, 0, 0};

static const ShellConfig shellConfig = {
    (BaseSequentialStream*) &SD1,
    shellCommands
};

static THD_WORKING_AREA(waShell, 1024);

void shellStart(void) {
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, &shellSerialConfig);
    shellInit();
    
    thread_t *shellThreadRef = chThdCreateStatic(
        waShell, sizeof(waShell), NORMALPRIO,
        shellThread, (void*) &shellConfig);
    chRegSetThreadNameX(shellThreadRef, "shell");
}