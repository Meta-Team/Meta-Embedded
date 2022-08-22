//
// Created by zhukerui on 2019/5/18.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "can_motor_controller.h"
#include "ahrs_ext.h"
#include "remote_interpreter.h"
#include "sd_card_interface.h"
#include "vision_interface.h"

#include "gimbal_scheduler.h"
#include "gimbal_logic.h"
#include "shoot_scheduler.h"
#include "shoot_logic.h"

#include "sentry_chassis_scheduler.h"
#include "sentry_chassis_logic.h"

#include "inspector_sentry.h"
#include "user_sentry.h"
#include "thread_priorities.h"

#include "settings_sentry.h"

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
// TODO: Select hardware and write corresponding software.
//AHRSExt ahrsExt;

/// Local Constants
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1

    InspectorS::init(&can1, &can2
#if ENABLE_AHRS
            ,&ahrsExt);
#else
    );
#endif

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

    /// Setup CAN1
    can1.start(THREAD_CAN1_RX_PRIO);
    can2.start(THREAD_CAN2_RX_PRIO);
    chThdSleepMilliseconds(5);
    CANMotorController::start(THREAD_MOTOR_SKD_PRIO, THREAD_FEEDBACK_SKD_PRIO, &can1, &can2);
    chThdSleepMilliseconds(5);
    InspectorS::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Setup AHRS_EXT
#if ENABLE_AHRS
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrsExt.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrsExt.load_calibration_data(EXT_AHRS_STORED_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrsExt.start(&can2);
    chThdSleepMilliseconds(5);
    InspectorS::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    InspectorS::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
#endif
    LED::led_on(DEV_BOARD_LED_AHRS);  // LED 3 on now

    /// Setup Remote
    Remote::start();
    InspectorS::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 4 on now

    chThdSleepMilliseconds(2000);
    // FIXME: re-enable startup check
    InspectorS::startup_check_gimbal_feedback(); // check gimbal motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_GIMBAL);  // LED 5 on now

    chThdSleepMilliseconds(10);
    InspectorS::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_CHASSIS);  // LED 6 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser

    /// Setup Referee
#if ENABLE_REFEREE
    Referee::init(THREAD_REFEREE_SENDING_PRIO);
#endif
#if ENABLE_VISION
    /// Setup VisionPort
    Vision::init();
#endif

    /// Complete Period 1
    LED::green_on();  // LED Green on now


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        CANMotorIF::motor_feedback[CANMotorCFG::YAW].last_rotor_angle_raw,
        CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle(),
        CANMotorIF::motor_feedback[CANMotorCFG::PITCH].last_rotor_angle_raw,
        CANMotorIF::motor_feedback[CANMotorCFG::PITCH].accumulate_angle());

    /// Start SKDs
    GimbalSKD::start(THREAD_GIMBAL_SKD_PRIO);
    ShootSKD::start(THREAD_SHOOT_SKD_PRIO);
    SChassisSKD::start(SChassisSKD::POSITIVE, SChassisSKD::POSITIVE, THREAD_CHASSIS_SKD_PRIO);

    /// Start LGs
    GimbalLG::init(THREAD_GIMBAL_LG_VISION_PRIO, THREAD_GIMBAL_BALLISTIC_PRIO);
    ShootLG::init(45.0f, false, THREAD_SHOOT_BULLET_COUNTER_PRIO,
                  THREAD_SHOOT_BULLET_COUNTER_PRIO, THREAD_SHOOT_LG_VISION_PRIO);
    SChassisLG::init(THREAD_CHASSIS_MTN_CTL_PRIO);

    /// Start Inspector and User Threads
    // FIXME: re-enable inspector
    InspectorS::start_inspection(THREAD_INSPECTOR_PRIO);
    UserS::start(THREAD_USER_PRIO);

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