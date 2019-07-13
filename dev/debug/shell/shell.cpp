/**
 * @file    shell.h
 * @brief   Serial shell interface and debug macros.
 *
 * @addtogroup shell
 * @{
 */

#include "shell.h"

using namespace chibios_rt;

/**
 * Declaration for class variables
 */
ShellCommand Shell::shellCommands_[SHELL_MAX_COMMAND_COUNT + 1] = {{nullptr, nullptr}};
ShellConfig Shell::shellConfig;
constexpr SerialConfig Shell::SHELL_SERIAL_CONFIG;
bool Shell::enabled = false;
char Shell::complection_[SHELL_MAX_COMMAND_COUNT][SHELL_MAX_LINE_LENGTH] = {{0}};
#if SHELL_ENABLE_ISR_FIFO
uint8_t Shell::isrTxBuf_[SHELL_ISR_TX_BUF_SIZE];
output_queue_t Shell::isrTxQueue_;
Shell::ShellISRTxThread Shell::isrTxThread;
#endif


THD_WORKING_AREA(wa, SHELL_RX_WORKAREA_SIZE);  // the working area for the shell rx thread


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
            shellCommands_
#if (SHELL_USE_HISTORY == TRUE)
            , new char[64],
            64
#endif
#if (SHELL_USE_COMPLETION == TRUE)
            , (char **) complection_
#endif
    };

    sdStart(&SD6, &SHELL_SERIAL_CONFIG);
    // Call init provided by shell
    shellInit();

    // Create a thread and name it
    thread_t *shellThreadRef = chThdCreateStatic(
            wa, sizeof(wa), prio,
            shellThread, (void *) &shellConfig);
    chRegSetThreadNameX(shellThreadRef, "shell_rx");

#if SHELL_ENABLE_ISR_FIFO
    iqObjectInit(&isrTxQueue_, isrTxBuf_, SHELL_ISR_TX_BUF_SIZE, nullptr, nullptr);
    isrTxThread.start(prio - 1);
#endif

    enabled = true;
    return true;
}

bool Shell::addCommands(const ShellCommand *commandList) {
    int i = 0;
    while (i < SHELL_MAX_COMMAND_COUNT && shellCommands_[i].sc_name != nullptr) i++;
    while (i < SHELL_MAX_COMMAND_COUNT && commandList->sc_name != nullptr) {
        shellCommands_[i].sc_name = commandList->sc_name;
        shellCommands_[i].sc_function = commandList->sc_function;
        i++;
        commandList++;
    }
    shellCommands_[i] = {nullptr, nullptr};
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

#if SHELL_ENABLE_ISR_FIFO
int Shell::printfI(const char *fmt, ...) {
    va_list ap;
    int formatted_bytes;

    va_start(ap, fmt);
    formatted_bytes = chsnprintf((char *) isrTxBuf_, SHELL_ISR_TX_BUF_SIZE, fmt, ap);
    for (int i = 0; i < formatted_bytes; i++) {
        iqPutI(&isrTxQueue_, isrTxBuf_[i]);
    }
    va_end(ap);

    return formatted_bytes;
}

void Shell::ShellISRTxThread::main() {
    setName("shell_isr");
    msg_t c;
    while (!shouldTerminate()) {
        c = iqGet(&isrTxQueue_);
        if (c != MSG_TIMEOUT) {
            streamPut((BaseSequentialStream *) &SD6, (uint8_t) c);
        } else sleep(TIME_MS2I(1));
    }
}

#endif

int Shell::atoi(const char *s) {
    int ret = 0;
    const char *p = s;
    if (*p == '-') p++;
    while (*p) {
        ret = ret * 10 + (*p - '0');
        p++;
    }
    if (s[0] == '-') ret = -ret;
    return ret;
}

float Shell::atof(const char *s) {
    float rez = 0.0f;
    float fact = 1.0f;
    float sign = 1.0f;
    if (*s == '-') {
        s++;
        sign = -1.0f;
    }
    for (int point_seen = 0; *s; s++) {
        if (*s == '.') {
            point_seen = 1;
            continue;
        };
        if (point_seen) {
            fact /= 10.0f;
            rez = rez + (float) (*s - '0') * fact;
        } else {
            rez = rez * 10.0f + (float) (*s - '0');
        }
    };
    return rez * sign;
}

/** @} */