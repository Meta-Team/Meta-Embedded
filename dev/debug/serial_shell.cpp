#include "serial_shell.h"

using namespace chibios_rt;

/**
 * Declaration for class variables
 */
ShellCommand Shell::shellCommands[Shell::maxCommandCount + 1] = {nullptr, nullptr};
ShellConfig Shell::shellConfig;
SerialConfig Shell::shellSerialConfig = {115200, 0, 0, 0};
bool Shell::enabled = false;

/**
 * The working area for the shell thread
 */
THD_WORKING_AREA(wa, 4096);

/**
 * Function to start the thread.
 * Instead of calling main(), as BaseStaticThread does,
 * it calls the thread provided by ChibiOS shell.
 */
bool Shell::start(tprio_t prio) {

    if (enabled) return false;

    addCommands(shell_debug_commands); // add common debug commands

    // Configure shell
    shellConfig = {
            (BaseSequentialStream *) &SD6,
            shellCommands
#if (SHELL_USE_HISTORY == TRUE)
    ,new char[64],
64
#endif
    };

    sdStart(&SD6, &shellSerialConfig);
    // Call init provided by shell
    shellInit();

    // Create a thread and name it
    thread_t *shellThreadRef = chThdCreateStatic(
            wa, sizeof(wa), prio,
            shellThread, (void *) &shellConfig);
    chRegSetThreadNameX(shellThreadRef, "shell");

    enabled = true;
    return true;
}


bool Shell::addCommands(ShellCommand *commandList) {
    int i = 0;
    while (i < maxCommandCount && shellCommands[i].sc_name != nullptr) i++;
    while (i < maxCommandCount && commandList->sc_name != nullptr) {
        shellCommands[i].sc_name = commandList->sc_name;
        shellCommands[i].sc_function = commandList->sc_function;
        i++;
        commandList++;
    }
    shellCommands[i] = {nullptr, nullptr};
    return (commandList->sc_name == nullptr);
}


int Shell::printf(const char *fmt, ...) {
    va_list ap;
    int formatted_bytes;

    va_start(ap, fmt);
    formatted_bytes = chvprintf((BaseSequentialStream *) &SD6, fmt, ap);
    va_end(ap);

    return formatted_bytes;
}