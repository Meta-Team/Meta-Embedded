/**
 * @file    shell.h
 * @brief   Serial shell interface and debug macros.
 *
 * @addtogroup shell
 * @{
 */

#include "shell.h"
#include "shell_base.h"
#include "printf.h"
#include "shell_dbg_cmd.h"
#include <memstreams.h>

using namespace chibios_rt;

/**
 * Declaration for class variables
 */
Shell::Command Shell::shellCommands[SHELL_MAX_COMMAND_COUNT + 1] = {{nullptr, nullptr, nullptr, nullptr}};
bool Shell::enabled = false;
MUTEX_DECL(Shell::printfMutex);
#if (SHELL_USE_COMPLETION == TRUE)
char Shell::completion[SHELL_MAX_COMMAND_COUNT][SHELL_MAX_LINE_LENGTH] = {{0}};
#endif
Shell::FeedbackThread Shell::feedbackThread;

THD_WORKING_AREA(wa, SHELL_RX_WORK_AREA_SIZE);  // the working area for the shell rx thread

static ShellConfig shellConfig;
static constexpr SerialConfig SHELL_SERIAL_CONFIG = {115200,
                                                     0,
                                                     USART_CR2_STOP1_BITS,
                                                     0};

/**
 * Function to start the thread.
 * Instead of calling main(), as BaseStaticThread does,
 * it calls the thread provided by ChibiOS shell.
 */
bool Shell::start(tprio_t prio) {

    if (enabled) return false;

    addCommands(shell_debug_commands); // add common debug commands

    // Configure shell
    shellConfig = ShellConfig{
            (BaseSequentialStream *) &SD6,
            (ShellCommand *) shellCommands,
            &printfMutex,
#if (SHELL_USE_HISTORY == TRUE)
            , new char[64],
            64
#endif
#if (SHELL_USE_COMPLETION == TRUE)
            , (char **) completion
#endif
    };

    sdStart(&SD6, &SHELL_SERIAL_CONFIG);
    // Call init provided by shell
    shellInit();

    // Create a thread and name it
    thread_t *shellThreadRef = chThdCreateStatic(
            wa, sizeof(wa), prio,
            shellThread, (void *) &shellConfig);
    chRegSetThreadNameX(shellThreadRef, "Shell_RX");

    feedbackThread.start(prio);

    enabled = true;
    return true;
}

void Shell::addCommands(const Shell::Command *commandList) {
    int i = 0;
    while (i < SHELL_MAX_COMMAND_COUNT && shellCommands[i].name != nullptr) i++;
    while (i < SHELL_MAX_COMMAND_COUNT && commandList->name != nullptr) {
        shellCommands[i].name = commandList->name;
        shellCommands[i].arguments = commandList->arguments;
        shellCommands[i].callback = commandList->callback;
        shellCommands[i].callbackArg = commandList->callbackArg;
        i++;
        commandList++;
    }
    shellCommands[i] = {nullptr, nullptr, nullptr, nullptr};
}

void Shell::addFeedbackCallback(const FeedbackCallbackFunction &callback, void *callbackArg) {
    int i = 0;
    while (i < SHELL_MAX_COMMAND_COUNT && feedbackThread.feedbacks[i].callback != nullptr) i++;
    if (i < SHELL_MAX_COMMAND_COUNT) {
        feedbackThread.feedbacks[i] = {callback, callbackArg};
        feedbackThread.feedbacks[i + 1] = {nullptr, nullptr};
    }
}


int Shell::printf(const char *fmt, ...) {
    va_list ap;
    int formatted_bytes;

    va_start(ap, fmt);
    chMtxLock(&printfMutex);
    {
        formatted_bytes = chvprintf((BaseSequentialStream *) &SD6, fmt, ap);
    }
    chMtxUnlock(&printfMutex);
    va_end(ap);

    return formatted_bytes;
}

int Shell::printfI(const char *fmt, ...) {
    va_list ap;
    int formatted_bytes;

    va_start(ap, fmt);
    formatted_bytes = chvprintfI((BaseSequentialStream *) &SD6, fmt, ap);
    va_end(ap);

    return formatted_bytes;
}

int Shell::snprintf(char *str, size_t size, const char *fmt, ...) {
    // Copy and paste from ChibiOS, as ... argument cannot be passed as another ..., but va_list

    va_list ap;
    MemoryStream ms;
    BaseSequentialStream *chp;
    size_t size_wo_nul;
    int retval;

    if (size > 0)
        size_wo_nul = size - 1;
    else
        size_wo_nul = 0;

    /* Memory stream object to be used as a string writer, reserving one
       byte for the final zero.*/
    msObjectInit(&ms, (uint8_t *) str, size_wo_nul, 0);

    /* Performing the print operation using the common code.*/
    chp = (BaseSequentialStream *) (void *) &ms;
    va_start(ap, fmt);
    retval = chvprintf(chp, fmt, ap);
    va_end(ap);

    /* Terminate with a zero, unless size==0.*/
    if (ms.eos < size)
        str[ms.eos] = 0;

    /* Return number of bytes that would have been written.*/
    return retval;
}

void Shell::usage(const char *message) {
    Shell::printf("Usage: %s" ENDL, message);
}

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

bool Shell::parseIDAndBool(int argc, char **argv, int idCount, int &id, bool &val) {
    if (argc != 2) return false;

    if (argv[0][0] == 'A' && argv[0][1] == '\0') {
        id = -1;
    } else {
        id = Shell::atoi(argv[0]);
        if (id >= idCount) return false;
    }

    unsigned v = Shell::atoi(argv[1]);
    if (v > 1) return false;
    val = v;

    return true;
}

void Shell::FeedbackThread::main() {
    setName("Feedback");
    while (!shouldTerminate()) {
        for (auto &f : feedbacks) {
            if (f.callback == nullptr) break;
            f.callback(f.callbackArg);
        }
        sleep(TIME_MS2I(FEEDBACK_INTERVAL));
    }
}

/** @} */