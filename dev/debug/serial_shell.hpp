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

    void main(void) override;
public:
    chibios_rt::ThreadReference start(tprio_t prio) override;
};

static SerialShellThread serialShell;

#endif