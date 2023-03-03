//
// Created by Tianyi Han on 2/20/2023.
//

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "can_motor_controller.h"
#include "hardware_conf.h"

using namespace chibios_rt;
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class MotorControl :public BaseStaticThread<512> {
private:
    int shell_print_cnt = 0;

    void main() final{
        setName("feedback");
        while(!shouldTerminate()){
            CANMotorController::set_target_angle(CANMotorCFG::MOTOR1, 180);
            sleep(TIME_MS2I(1000));
            CANMotorController::set_target_angle(CANMotorCFG::MOTOR1, -180);
            sleep(TIME_MS2I(1000));

            shell_print_cnt++;
            if (shell_print_cnt == 1000){
                Shell::printf("%.2f" ENDL, CANMotorIF::motor_feedback[CANMotorCFG::MOTOR1].actual_angle);
                shell_print_cnt = 0;
            }
        }
    }
} ControlThread;

int main(void) {
    halInit();
    System::init();

    // Start shell at high priority
    Shell::start(HIGHPRIO);
    chThdSleepMilliseconds(500);
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);

    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);
    chThdSleepMilliseconds(500);

    ControlThread.start(NORMALPRIO+2);
    // green LED on, indicating thread running
    LED::green_on();
    LED::red_off();
    LED::blue_off();

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}