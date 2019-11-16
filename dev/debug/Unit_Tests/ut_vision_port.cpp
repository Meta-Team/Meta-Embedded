//
// Created by Kerui Zhu on 7/4/2019.
//

#include "ch.hpp"
#include "hal.h"
#include "buzzer_scheduler.h"

#include "led.h"
#include "shell.h"
#include "vision_port.h"

class VisionPortEchoThread: public chibios_rt::BaseStaticThread<512>{
    void main() final {
        setName("vision_port");
        while (!shouldTerminate()){
            LOG("YAW: %.2f  PIT: %.2f  dist: %.2f",
                    VisionPort::enemy_info.yaw_angle, VisionPort::enemy_info.pitch_angle, VisionPort::enemy_info.distance);
            VisionPort::send_gimbal(1000, 1000);
            sleep(TIME_MS2I(1000));
        }
    }
} visionPortEchoThread;

int main(void) {

    /*** --------------------------- Period 1. Basic Setup --------------------------- ***/

    /** Basic Initializations **/
    halInit();
    chibios_rt::System::init();

    LED::green_off();
    LED::red_off();

    /** Debug Setup **/
    Shell::start(HIGHPRIO);

    VisionPort::init();

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    visionPortEchoThread.start(NORMALPRIO);
    chThdSleepMilliseconds(500);

    /** Play the Startup Sound **/
//    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel, LOWPRIO);


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}