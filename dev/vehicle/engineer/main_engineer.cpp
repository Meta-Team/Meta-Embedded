//
// Created by Kerui Zhu on 7/10/2019.
// Modified by LaiXinyi on 7/19/2019.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs.h"
#include "remote_interpreter.h"
#include "sd_card_interface.h"

#include "chassis_interface.h"
#include "engineer_elevator_interface.h"
#include "robotic_arm_interface.h"

#include "engineer_chassis_skd.h"
#include "engineer_elevator_skd.h"
//#include "robotic_arm_skd.h"

#include "engineer_elevator_logic.h"

#include "inspector_engineer.h"
#include "user_engineer.h"

#include "settings_engineer.h"

/// Vehicle Specific Configurations
#if defined(ENGINEER)
#include "vehicle_engineer.h"
#else
#error "File main_engineer.cpp should only be used for Engineer."
#endif

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);
    palClearPad(GPIOH, GPIOH_POWER4_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    InspectorE::init(&can1, &can2);
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    Shell::addCommands(mainProgramCommands);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);

    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }

    LED::led_on(DEV_BOARD_LED_SYSTEM_INIT);  // LED 1 on now

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_PRIO);
    can2.start(THREAD_CAN2_PRIO);
    chThdSleepMilliseconds(5);
    InspectorE::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Setup Remote
    Remote::start();
    InspectorE::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 3 on now

    /// Setup GimbalIF
    EngineerGimbalIF::init();

    /// Setup RoboticArmIF
    RoboticArmIF::init(&can2);
    chThdSleepMicroseconds(10);
    InspectorE::startup_check_robotic_arm_feedback();
    LED::led_on(DEV_BOARD_LED_ROBOTIC_ARM);  // LED 4 on now

    /// Setup ElevatorIF
    float init_angle;
    if (SDCard::get_data(ELEVATOR_ANGLE_DATA_ID, &init_angle, sizeof(init_angle)) == SDCard::OK) {
        EngineerElevatorIF::init(&can2, init_angle);
        LOG("using init angle in SDCard, init angle: %.2f", init_angle);
    } else {
        EngineerElevatorIF::init(&can2, 0);
        LOG("present angle is set as 0");
    }
    chThdSleepMilliseconds(10);
    // TODO: re-enable Inspector
//    InspectorE::startup_check_elevator_feedback();  // check elevator motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_ELEVATOR);  // LED 5 on now
    EngineerElevatorSKD::set_target_height(ELEVATOR_ORIGIN_HEIGHT);

    /// Setup ChassisIF
    ChassisIF::init(&can1);
    chThdSleepMilliseconds(10);
    InspectorE::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_CHASSIS);  // LED 6 on now

    /// Setup Referee
    Referee::init(THREAD_REFEREE_SENDING_PRIO);

    /// Complete Period 1
    LED::green_on();  // LED Green on now


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Start SKDs
    EngineerChassisSKD::start(CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, THREAD_CHASSIS_SKD_PRIO);
    EngineerChassisSKD::load_pid_params(CHASSIS_PID_V2I_PARAMS);
    EngineerElevatorSKD::start(THREAD_ELEVATOR_SKD_PRIO);
    EngineerElevatorSKD::load_pid_params(EngineerElevatorSKD::ELEVATOR_A2V, ELEVATOR_PID_A2V_PARAMS);
    EngineerElevatorSKD::load_pid_params(EngineerElevatorSKD::ELEVATOR_V2I, ELEVATOR_PID_V2I_PARAMS);
    EngineerElevatorSKD::load_pid_params(EngineerElevatorSKD::AIDED_WHEEL_V2I, AIDED_MOTOR_PID_V2I_PARAMS);
    EngineerElevatorSKD::load_pid_params(EngineerElevatorSKD::BALANCE_PID, {0, 0, 0, 0, 0});
    RoboticArmSKD::start(THREAD_ROBOTIC_ARM_SKD_PRIO);

    /// Start LGs
    EngineerElevatorLG::init(THREAD_ELEVATOR_LG_PRIO);

    /// Start Inspector and User Threads
    InspectorE::start_inspection(THREAD_INSPECTOR_PRIO, THREAD_INSPECTOR_REFEREE_PRIO);
    UserE::start(THREAD_USER_PRIO, THREAD_USER_ACTION_PRIO, THREAD_USER_CLIENT_DATA_SEND_PRIO);

    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);  // Now play the startup sound


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
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