#include "serial_shell_commands.hpp"
using namespace chibios_rt;

/**
 * cmd_hello: hello world again.
 * It's a demonstration of how to write a function for the shell.
 */
static void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "hello");
        return;
    }
    chprintf(chp, "Hello World from ChibiOS!" SHELL_NEWLINE_STR);
}

/**
 * Echo the CPU usage data of each threads.
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_show_thread_stats(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "stats");
        return;
    }

    chSysLock();

    Registry reg; // a class managing registered threads
    ThreadReference thd_ref = reg.firstThread();
    thread_t *thd;

    // Iterate threads and sum up cumulative time.
    uint64_t sum = 0;
    while (!thd_ref.isNull()) {
        sum += thd_ref.getInner()->stats.cumulative;
        thd_ref = reg.nextThread(thd_ref);
    }

    // Print the header
    chprintf(chp, "     thread name       best    average      worst    count   all" SHELL_NEWLINE_STR);

    // Iterate threads again and show data.
    thd_ref = reg.firstThread();
    while (!thd_ref.isNull()) {
        thd = thd_ref.getInner();
        chprintf(chp, "%16s %10lu %10lu %10lu %8lu  %3u%%" SHELL_NEWLINE_STR,
                 thd->name, thd->stats.best, (unsigned long)(thd->stats.cumulative / thd->stats.n),
                 thd->stats.worst, thd->stats.n, (unsigned int)(100 * thd->stats.cumulative / sum));
        thd_ref = reg.nextThread(thd_ref);
    }

    chSysUnlock();
}

/**
 * List of available shell commands.
 * Must be at the bottom of file, otherwise it won't find the functions.
 * {NULL, NULL} is a marker of end of list and mustn't be removed.
 */
const ShellCommand shellCommands[] = {
    {"hello", cmd_hello},
    {"stats", cmd_show_thread_stats},
    {NULL, NULL}
};
