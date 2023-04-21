//
// Created by Tianyi Han on 4/21/2023.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "can_motor_controller.h"
#include "hardware_conf.h"

#include "ahrs_c_interface.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard_C ahrs;

static constexpr Matrix33 MPU_ROTATION_MATRIX = {{-1.0f, 0.0f, 0.0f},
                                                 {0.0f, -1.0f, 0.0f},
                                                 {0.0f, 0.0f, 1.0f}};

static constexpr Matrix33 ANGLE_INSTALLATION_MATRIX = {{1.0f, 0.0f, 0.0f},
                                                       {0.0f, 1.0f, 0.0f},
                                                       {0.0f, 0.0f, 1.0f}};

class BalancingControlThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("BalancingControl");
        while (!shouldTerminate()) {
            Vector3D angle = ANGLE_INSTALLATION_MATRIX * ahrs.get_angle();
            Shell::printf("!angle\t%.2f\t%.2f\t%.2f" ENDL,
                          angle.x,
                          angle.y,
                          angle.z);
            sleep(TIME_MS2I(10));
            CANMotorController::set_target_current(CANMotorCFG::MOTOR1, 10000);
        }
    }
} ControlThread;

int main(void) {
    halInit();
    System::init();

    Shell::start(HIGHPRIO);
    LED::all_off();
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);

    ahrs.start(MPU_ROTATION_MATRIX, NORMALPRIO + 4);

    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);
    chThdSleepMilliseconds(500);

    // Indicating thread running
    LED::green_on();
    BuzzerSKD::init(LOWPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_dji);

    // Start balancing control thread
    ControlThread.start(NORMALPRIO+5);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}