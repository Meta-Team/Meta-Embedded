//
// Created by Kerui Zhu on 7/22/2019.
//

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "shell.h"
#include "buzzer_scheduler.h"
#include "engineer_gimbal.h"

#include "remote_interpreter.h"

class EngineerGimbalThread: public chibios_rt::BaseStaticThread<256>{
    void main()final {
        setName("engineer_gimbal");
        while (!shouldTerminate()){
            if (Remote::rc.s1 == Remote::S_UP){
                EngineerGimbalIF::set_target_angle(0,0);
            } else if (Remote::rc.s1 == Remote::S_MIDDLE){
                EngineerGimbalIF::set_target_angle((Remote::rc.ch0 / 2 + 0.5) * EngineerGimbalIF::MAX_ANGLE,
                        (Remote::rc.ch1 / 2 + 0.5) * EngineerGimbalIF::MAX_ANGLE);
            } else{
                EngineerGimbalIF::set_target_angle(EngineerGimbalIF::MAX_ANGLE / 2, EngineerGimbalIF::MAX_ANGLE / 2);
            }
            sleep(TIME_MS2I(5));
        }
    }
}engineerGimbalThread;

static void g_set_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set_a yaw pitch");
        return;
    }
    EngineerGimbalIF::set_target_angle(Shell::atof(argv[0]), Shell::atof(argv[1]));
}

static void g_echo_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "a");
        return;
    }
    LOG("%d", (int)(EngineerGimbalIF::get_target_angle(EngineerGimbalIF::YAW) / EngineerGimbalIF::MAX_ANGLE * 1000 + 250));
}

ShellCommand engineerGimbalCommand[]{
        {"set_a",   g_set_angle},
        {"a",       g_echo_angle},
        {nullptr, nullptr}
};

int main(void) {

    /*** --------------------------- Period 1. Basic Setup --------------------------- ***/

    /** Basic Initializations **/
    halInit();
    chibios_rt::System::init();

    LED::green_off();
    LED::red_off();

    /** Debug Setup **/
    Shell::start(HIGHPRIO);
    Shell::addCommands(engineerGimbalCommand);

    /** Basic IO Setup **/
    Remote::start();


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /*** Parameters Set up***/
    EngineerGimbalIF::init();

    LED::green_on();
    /** Start Logic Control Thread **/
    chThdSleepMilliseconds(500);
    engineerGimbalThread.start(HIGHPRIO - 1);
    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
    BuzzerSKD::init(LOWPRIO);
    /** Play the Startup Sound **/
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);


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