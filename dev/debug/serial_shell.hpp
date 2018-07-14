#ifndef _SERIAL_SHELL_HPP_
#define _SERIAL_SHELL_HPP_

#include "ch.hpp"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "serial_shell_commands.hpp"

class SerialShellThread : public chibios_rt::BaseThread {
protected:
    THD_WORKING_AREA(wa, 1024);

    SerialConfig shellSerialConfig = {115200, 0, 0, 0};

    const ShellConfig shellConfig = {
        (BaseSequentialStream*) &SD1,
        shellCommands
    };

    void main(void) override {}
public:
    chibios_rt::ThreadReference start(tprio_t prio) override {
        palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
        palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
        sdStart(&SD1, &shellSerialConfig);
        shellInit();
        
        thread_t *shellThreadRef = chThdCreateStatic(
            wa, sizeof(wa), prio,
            shellThread, (void*) &shellConfig);
        chRegSetThreadNameX(shellThreadRef, "shell");

        return chibios_rt::ThreadReference(shellThreadRef);
    }
};

static SerialShellThread serialShell;

#endif