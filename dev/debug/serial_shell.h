#ifndef _SERIAL_SHELL_HPP_
#define _SERIAL_SHELL_HPP_

#include "ch.hpp"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "shell_debug_commands.h"

/**
 * A serial shell interface.
 * Using SD6 (UART6) as serial port.
 */
class Shell {

private:

    /**
     * Container for the shell commands
     */
    static constexpr int maxCommandCount = 20;
    static ShellCommand shellCommands[];

    /**
     * Config of the serial port used by the shell.
     * First parameter is the bitrate.
     * Other three are for parity, stop bits, etc which we don't care.
     */
    static SerialConfig shellSerialConfig;

    /**
     * Config of the shell.
     * First parameter is the Serial port.
     * Second parameter is the list of commands, provided in serial_shell_commands.hpp
     */
    static ShellConfig shellConfig;

public:

    static bool enabled;  // whether the shell thread has started

    /**
     * @brief start the shell thread
     * @param prio priority
     * @return if the shell thread has already started return false
     */
    static bool start(tprio_t prio);

    /**
     * @brief add shell commands
     * @param command_list NULL-terminate command lsit
     * @return if one or more commands can't be added because the maximum command count has been reached, return false
     */
    static bool addCommands(ShellCommand *command_list);

    /**
     * @brief printf through shell
     * @param fmt
     * @param ...
     * @return the number of bytes that has been printed
     */
    static int printf(const char *fmt, ...);

    /**
     * @brief convert string to signed integer
     * @param s        the string to be converted
     * @return the integer
     * @note no error check
     */
    static inline int atoi (char* s) {
        int ret = 0;
        char* p = s;
        if (*p == '-') p++;
        while (*p) {
            ret = ret * 10 + (*p - '0');
            p++;
        }
        if (s[0] == '-') ret = -ret;
        return ret;
    }
};


#endif