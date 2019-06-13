//
// Created by 钱晨 on 2019-05-17.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "scheduler/shoot_scheduler.h"

#include "common_macro.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);

float bullet_per_second_ = 0.0f;
float angle_per_bullet_loader_ = 72.0f;
float angle_per_bullet_plate_ = 36.0f;

time_msecs_t test_end_time = 0;

unsigned const SHOOT_FEEDBACK_INTERVAL = 25; // [ms]
unsigned const SHOOT_THREAD_INTERVAL = 2; // [ms]

/**
 * @brief set shoot controller target
 * @param chp
 * @param argc
 * @param argv
 */
 static  void cmd_shoot_set_target(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if(argc != 1) {
        shellUsage(chp, "s_set_target bullet_per_second n(num/s)");
        return;
    }

    bullet_per_second_ = Shell::atof(argv[0]);
 }

 /**
  * @brief set shoot common PID params
  * @param chp
  * @param argc
  * @param argv
  */
static void cmd_shoot_set_plate_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
     (void) argv;
     if (argc != 3) {
         shellUsage(chp, "c_set_p_params ki kp kd");
         chprintf(chp, "!cpe" SHELL_NEWLINE_STR); //echo shoot parameters error
         return;
     }

     Shoot::v2i_pid[1].change_parameters({Shell::atof(argv[0]),
                                 Shell::atof(argv[1]),
                                 Shell::atof(argv[2]),
                                 500.0f,
                                 5000.0f});
     Shoot::v2i_pid[1].clear_i_out();
     chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}

/**
 * @brief set shoot common PID params
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_set_loader_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "c_set_l_params ki kp kd");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR); //echo shoot parameters error
        return;
    }

    Shoot::v2i_pid[0].change_parameters({Shell::atof(argv[0]),
                              Shell::atof(argv[1]),
                              Shell::atof(argv[2]),
                              0.0f,
                              2000.0f});
    Shoot::v2i_pid[0].clear_i_out();
    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}

// Shell commands to control the shoot
ShellCommand shootCommands[] = {
        {"s_set_target", cmd_shoot_set_target},
        {"s_set_p_params", cmd_shoot_set_plate_parameters},
        {"s_set_l_params", cmd_shoot_set_loader_parameters},
        {nullptr,          nullptr}
};

class ShootFeedbackThread : public BaseStaticThread<1024> {
private:
    void main() final {
        setName("shoot");
        while (!shouldTerminate()) {
            Shell::printf("!cv, %u,%.2f,%.2f" SHELL_NEWLINE_STR,
                    TIME_I2MS(chibios_rt::System::getTime()),
                    Shoot::feedback[2].actual_velocity,
                    Shoot::feedback[3].actual_velocity);
            sleep(TIME_I2MS(SHOOT_FEEDBACK_INTERVAL));
        }
    }
} shootFeedbackThread;

class ShootThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("shoot");
        while (!shouldTerminate()) {
            Shoot::calc(bullet_per_second_);
            Shoot::send_gimbal_currents();

            sleep(TIME_I2MS((SHOOT_THREAD_INTERVAL)));
        }
    }
} shootThread;

int main(void) {
    halInit();
    System::init();
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(shootCommands);
    can1.start(HIGHPRIO - 1);

    // TODO: find a better way to handle this initialization
    GimbalIF::init(&can1, 0, 0);
    /** NOTICE: minus sign has been added here */
    Shoot::init(-72.0f, -36.0f);

    shootFeedbackThread.start(NORMALPRIO -1);
    shootThread.start(NORMALPRIO);
    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
    // enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}