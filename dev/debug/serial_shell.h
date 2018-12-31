#ifndef _SERIAL_SHELL_HPP_
#define _SERIAL_SHELL_HPP_

#include "ch.hpp"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "shell_debug_commands.h"

/**
 * SerialShellThread: as its name suggests.
 * Extends BaseThread to call the shell thread.
 */
class SerialShellThread : public chibios_rt::BaseThread {
protected:
    THD_WORKING_AREA(wa, 2048);

    void main(void) override;

public:
    chibios_rt::ThreadReference start(tprio_t prio) override;
};

// Create the shell thread.
static SerialShellThread serialShell;

int sprintf(const char *fmt, ...);

#endif