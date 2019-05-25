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
#if SHELL_USE_FIFO
uint8_t Shell::tx_buf_[SHELL_TX_PRINTF_BUF_SIZE];
output_queue_t Shell::tx_queue_;
Shell::ShellTXThread Shell::txThread;
#endif


/**
 * The working area for the shell rx thread
 */
THD_WORKING_AREA(wa, SHELL_RX_WORKAREA_SIZE);


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

#if SHELL_USE_FIFO
    oqObjectInit(&tx_queue_, tx_buf_, SERIAL_BUFFERS_SIZE, tx_queue_callback_, nullptr);
    txThread.start(prio - 1);
#endif

    enabled = true;
    return true;
}

void Shell::tx_queue_callback_(io_queue_t *qp) {

}

bool Shell::addCommands(ShellCommand *commandList) {
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
#if SHELL_USE_FIFO
    formatted_bytes = chsnprintf((char *) tx_buf_, SHELL_TX_PRINTF_BUF_SIZE, fmt, ap);
    oqWriteTimeout(&tx_queue_, tx_buf_, formatted_bytes, TIME_INFINITE);
#else
    formatted_bytes = chvprintf((BaseSequentialStream *) &SD6, fmt, ap);
#endif

    va_end(ap);

    return formatted_bytes;
}

#if SHELL_USE_FIFO

void Shell::ShellTXThread::main() {
    setName("shell_tx");
    msg_t c;
    while (!shouldTerminate()) {
        c = oqGetI(&tx_queue_);
        if (c != MSG_TIMEOUT) {
            streamPut((BaseSequentialStream *) &SD6, (uint8_t) c);
        } else sleep(TIME_US2I(100));
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
};