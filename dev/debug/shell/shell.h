//
// Created by Lan Tian on 2019-02-24.
//

/**
 * @file    shell.h
 * @brief   Serial shell interface and debug macros.
 *
 * @addtogroup shell
 * @{
 */


#ifndef META_INFANTRY_SHELL_H
#define META_INFANTRY_SHELL_H

#include "ch.hpp"
#include "hal.h"

#include "shell_base.h"
#include "printf.h"

#include "shell_dbg_cmd.h"

#include "common_macro.h"

#if defined(BOARD_RM_2018_A)
// USART6_TX - PG14, USART6_RX - PG9
#elif defined(BOARD_RM_2017)
// USART6_TX - PG14, USART6_RX - PG9
#else
#error "Shell has not been defined for selected board"
#endif

#define SHELL_ENABLE_ISR_FIFO FALSE
#define SHELL_MAX_COMMAND_COUNT 20
#define SHELL_RX_WORKAREA_SIZE 2048
#define SHELL_ISR_TX_WORKAREA_SIZE 512
#define SHELL_ISR_TX_BUF_SIZE 512

/*** Debug Macro ***/

#define VA_ARGS(...) , ##__VA_ARGS__

#define LOG(fmt, ...) Shell::printf("[%u] " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_USER(fmt, ...) Shell::printf("[%u] USER " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_ERR(fmt, ...) Shell::printf("[%u] ERR " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_WARN(fmt, ...) Shell::printf("[%u] WARN " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define DBPRINTF(fmt, ...) Shell::printf("%s:%d:%s(): " fmt SHELL_NEWLINE_STR, __FILE__, __LINE__, __func__ VA_ARGS(__VA_ARGS__))


/**
 * @name    Shell
 * @brief   A serial shell interface using SD6 (UART6) as serial port.
 * @pre     Pins are configured properly in board.h, and driver are configured properly in halconf.h
 * @usage   1. Call start() with given thread priority
 *          2. Call addCommands() to add custom command to shell
 *             Call printf() and other helper function
 */
class Shell {

public:

    static bool enabled;  // whether the shell thread has started

    /**
     * Start the shell thread
     * @param prio   Priority
     * @return If the shell thread has already started, return false
     */
    static bool start(tprio_t prio);

    /**
     * Add shell commands
     * @param commandList   NULL-terminate command lsit
     * @return If one or more commands can't be added because the maximum command count has been reached, return false
     */
    static bool addCommands(const ShellCommand *commandList);

    /**
     * Print with format through shell
     * @param fmt
     * @param ...
     * @return The number of bytes that has been printed
     * @note API, can only be called from normal thread state
     */
    static int printf(const char *fmt, ...);


#if SHELL_ENABLE_ISR_FIFO
    /**
     * Print with format printf through shell
     * @param fmt
     * @param ...
     * @return the number of bytes that has been printed
     * @playing_note I-Class function, can only be called from I-Lock state (ISR critical section)
     */
    static int printfI(const char *fmt, ...);
#endif

    /**
     * Convert string to signed integer
     * @param s   The string to be converted
     * @return The integer
     * @warning NO ERROR CHECK
     */
    static int atoi(const char *s);

    /**
     * Convert string to float
     * @param s        The string to be converted
     * @return The float
     * @warning NO ERROR CHECK
     */
    static float atof(const char *s);

private:

    static ShellCommand shellCommands_[]; // Container for the shell commands

    /**
     * Config of the shell.
     * First parameter is the Serial port.
     * Second parameter is the list of commands, provided in serial_shell_commands.hpp
     * @note DO NOT put it as a temporary variable in the start() function
     */
    static ShellConfig shellConfig;

    /**
     * Config of the serial port used by the shell.
     * First parameter is the bitrate.
     * Other three are for parity, stop bits, etc which we don't care.
     */
    static constexpr SerialConfig SHELL_SERIAL_CONFIG = {115200,
                                                         0,
                                                         USART_CR2_STOP1_BITS,
                                                         0};

    static char complection_[SHELL_MAX_COMMAND_COUNT][SHELL_MAX_LINE_LENGTH];

#if SHELL_ENABLE_ISR_FIFO

    static uint8_t isrTxBuf_[SHELL_ISR_TX_BUF_SIZE];
    static input_queue_t isrTxQueue_;

    class ShellISRTxThread : public chibios_rt::BaseStaticThread<SHELL_ISR_TX_WORKAREA_SIZE> {
        void main() final;
    };

    static ShellISRTxThread isrTxThread;

#endif

};

#endif

/** @} */