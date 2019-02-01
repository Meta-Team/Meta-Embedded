#ifndef META_INFANTRY_SERIAL_SHELL_H
#define META_INFANTRY_SERIAL_SHELL_H

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
     * @note NO ERROR CHECK
     */
    static inline int atoi (const char* s) {
        int ret = 0;
        const char* p = s;
        if (*p == '-') p++;
        while (*p) {
            ret = ret * 10 + (*p - '0');
            p++;
        }
        if (s[0] == '-') ret = -ret;
        return ret;
    }

    /**
     * @brief convert string to float
     * @param s        the string to be converted
     * @return the float
     * @note NO ERROR CHECK
     */
    static inline float atof(const char* s){
        int rez = 0;
        float fact = 1;
        if (*s == '-'){
            s++;
            fact = -1;
        }
        for (int point_seen = 0; *s; s++){
            if (*s == '.'){
                point_seen = 1;
                continue;
            };
            if (point_seen) fact /= 10.0f;
            rez = rez * 10 + (*s - '0');
        };
        return (float) rez * fact;
    };
};

#define SHELL_PRINTF(...) Shell::printf(__VA_ARGS__);
//#define SHELL_PRINTF(...)

#endif