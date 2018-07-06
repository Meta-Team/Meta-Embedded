#include "serial_shell_commands.hpp"

static void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: hello\r\n");
        return;
    }
    chprintf(chp, "Hello World from ChibiOS!\r\n");
}

const ShellCommand shellCommands[] = {
    {"hello", cmd_hello},
    {NULL, NULL}
};
