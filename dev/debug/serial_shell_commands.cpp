#include "serial_shell_commands.hpp"

/**
 * cmd_hello: hello world again.
 * It's a demonstration of how to write a function for the shell.
 */
static void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: hello\r\n");
        return;
    }
    chprintf(chp, "Hello World from ChibiOS!\r\n");
}

/**
 * List of available shell commands.
 * Must be at the bottom of file, otherwise it won't find the functions.
 * {NULL, NULL} is a marker of end of list and mustn't be removed.
 */
const ShellCommand shellCommands[] = {
    {"hello", cmd_hello},
    {NULL, NULL}
};
