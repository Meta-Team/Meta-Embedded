#ifndef _SERIAL_SHELL_COMMANDS_HPP_
#define _SERIAL_SHELL_COMMANDS_HPP_

#include "ch.hpp"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#define MAX_COMMAND_COUNT 10

// List of available shell commands, needed by serial_shell.hpp
extern ShellCommand shellCommands[MAX_COMMAND_COUNT + 1];

void shellAddCommands(ShellCommand *commandList);

#endif