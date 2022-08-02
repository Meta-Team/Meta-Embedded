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

#include "shellconf.h"
#include "usb_serial_interface.h"
#include "hardware_conf.h"

#include "common_macro.h"

#if defined(BOARD_RM_2018_A)
// USART6_TX - PG14, USART6_RX - PG9
#elif defined(BOARD_RM_2017)
// USART6_TX - PG14, USART6_RX - PG9
#else
#error "Shell has not been defined for selected board"
#endif
#if ENABLE_USB_SHELL == FALSE
#define SerialDriver SD6
#else
#define SerialDriver USBSerialIF::SDU
#endif

#define SHELL_RX_WORK_AREA_SIZE 2048

/*** Debug Macro ***/

#define ENDL SHELL_NEWLINE_STR

#define VA_ARGS(...) , ##__VA_ARGS__

// Should be called from NORMAL state
#define LOG(fmt, ...) Shell::printf("[%u] " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_USER(fmt, ...) Shell::printf("[%u] USER " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_ERR(fmt, ...) Shell::printf("[%u] ERR " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_WARN(fmt, ...) Shell::printf("[%u] WARN " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_LOC(fmt, ...) Shell::printf("%s:%d:%s(): " fmt SHELL_NEWLINE_STR, __FILE__, __LINE__, __func__ VA_ARGS(__VA_ARGS__))

// Should be called from I-Locked state
#define LOG_I(fmt, ...) Shell::printfI("[%u] " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_ERR_I(fmt, ...) Shell::printfI("[%u] ERR " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))
#define LOG_WARN_I(fmt, ...) Shell::printfI("[%u] WARN " fmt SHELL_NEWLINE_STR, SYSTIME VA_ARGS(__VA_ARGS__))

#define DECL_SHELL_CMD(name)      bool name(BaseSequentialStream *, int argc, char *argv[])

#define DEF_SHELL_CMD_START(name)                                   \
    bool name(BaseSequentialStream *chp, int argc, char *argv[]) {  \
        (void) chp;                                                 \
        if (argc == 1 && argv[0][0] == '?') return false;

#define DEF_SHELL_CMD_END                                           \
    }

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

    /**
     * Start the shell thread
     * @param prio   Thread priority
     * @return If the shell thread has already started, return false
     */
    static bool start(tprio_t prio);

    /**
     * Callback function pointer for Shell command
     * @note same as the revised shellcmd_t in shell_base.h
     */
    using CommandCallback = bool (*)(BaseSequentialStream *chp, int argc, char *argv[]);

    /**
     * Shell command defintion
     * @note same as the revised revised ShellCommand in shell_base.h
     */
    struct Command {
        const char *name;
        const char *arguments;
        CommandCallback callback;
        void *callbackArg;
    };

    /**
     * Add shell commands
     * @param commandList   NULL-terminate command list
     */
    static void addCommands(const Command *commandList);

    /**
     * Callback function to echo feedback
     */
    using FeedbackCallbackFunction = void (*)(void *);

    /**
     * Add feedback callback
     * @param callback     Feedback function callback
     * @param callbackArg  Callback argument
     */
    static void addFeedbackCallback(const FeedbackCallbackFunction &callback, void *callbackArg = nullptr);

    /**
     * Print with format through shell
     * @param fmt
     * @param ...
     * @return The number of bytes that has been printed
     * @note API, can only be called from NORMAL thread state
     */
    static int printf(const char *fmt, ...);

    /**
     * Print with format through shell inside I-Lock state
     * @param fmt
     * @param ...
     * @return The number of bytes that has been printed
     * @note I-Class API, can only be called from I-Lock State
     */
    static int printfI(const char *fmt, ...);

    /**
     * Format to string
     * @param str   pointer to a buffer
     * @param size  maximum size of the buffer
     * @param fmt   formatting string
     * @param ...
     * @return The number of characters (excluding the terminating NUL byte) that would have been stored in @p str
     *         if there was room.
     */
    static int snprintf(char *str, size_t size, const char *fmt, ...);

    /**
     * Print command usage
     * @param message
     * @note API, can only be called from NORMAL thread state
     */
    static void usage(const char *message);

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

    /**
     * Parse an ID and a bool value from arguments.
     * @param argc
     * @param argv
     * @param idCount
     * @param id    [Out] -1 for all
     * @param val
     * @return      Input valid or not
     * @note        id and val may change even when parse fails
     */
    static bool parseIDAndBool(int argc, char *argv[], int idCount, int &id, bool &val);

private:

    static bool enabled;  // whether the shell thread has started

    static mutex_t printfMutex;

    static Command shellCommands[]; // container for the shell commands

#if (SHELL_USE_COMPLETION == TRUE)
    static char completion[SHELL_MAX_COMMAND_COUNT][SHELL_MAX_LINE_LENGTH];
#endif

    class FeedbackThread : public chibios_rt::BaseStaticThread<512> {
    public:
        struct FeedbackCallback {
            FeedbackCallbackFunction callback;
            void *callbackArg;
        } feedbacks[SHELL_MAX_COMMAND_COUNT + 1] = {{nullptr, nullptr}};
    private:
        void main() override;
        static constexpr unsigned FEEDBACK_INTERVAL = 20;  // [ms]
    };

    static FeedbackThread feedbackThread;

};

#endif

/** @} */