//
// Created by Kerui Zhu on 7/19/2019.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs_ext.h"
#include "remote_interpreter.h"
#include "sd_card_interface.h"
#include "vision_port.h"

#include "gimbal_interface.h"
#include "gimbal_scheduler.h"
#include "gimbal_logic.h"
#include "shoot_scheduler.h"
#include "shoot_logic.h"

#include "inspector_aerial.h"
#include "user_aerial.h"
#include "settings_aerial.h"
#include "vehicle_aerial.h"

/// Board Guard
#if defined(BOARD_RM_2018_A)
#else
#error "Infantry supports only RM Board 2018 A currently"
#endif

/// Instances
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSExt ahrsExt;

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
    InspectorA::init(&can1, &can2, &ahrsExt);
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
    can1.start(THREAD_CAN1_PRIO);
    can2.start(THREAD_CAN2_PRIO);
    chThdSleepMilliseconds(5);
    InspectorA::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Setup AHRS_EXT
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
    InspectorA::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    InspectorA::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_AHRS);  // LED 3 on now

    /// Setup Remote
    Remote::start();
    InspectorA::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 4 on now


    /// Setup GimbalIF (for Gimbal and Shoot)
    GimbalIF::init(&can1, &can2, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW,
                   GIMBAL_YAW_MOTOR_TYPE, GIMBAL_PITCH_MOTOR_TYPE, SHOOT_BULLET_MOTOR_TYPE, SHOOT_PLATE_MOTOR_TYPE,
                   GIMBAL_YAW_CAN_CHANNEL, GIMBAL_PITCH_CAN_CHANNEL, GIMBAL_BULLET_CAN_CHANNEL, GIMBAL_PLATE_CAN_CHANNEL);
    chThdSleepMilliseconds(1000); // wait for friction wheel to reset
    InspectorA::startup_check_gimbal_feedback(); // check gimbal motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_GIMBAL);  // LED 5 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser

    /// Setup Referee
    Referee::init(THREAD_REFEREE_SENDING_PRIO);

    /// Setup VisionPort
    VisionPort::init();

    /// Complete Period 1
    LED::green_on();  // LED Green on now


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW].last_angle_raw, GimbalIF::feedback[GimbalIF::YAW].actual_angle,
        GimbalIF::feedback[GimbalIF::PITCH].last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH].actual_angle);

    /// Start SKDs
    GimbalSKD::start(&ahrsExt, GIMBAL_ANGLE_INSTALLATION_MATRIX_, GIMBAL_GYRO_INSTALLATION_MATRIX_,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO,
                     GIMBAL_YAW_DECELERATION_RATIO, GIMBAL_PITCH_DECELERATION_RATIO);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS);

    ShootSKD::start(SHOOT_BULLET_INSTALL_DIRECTION, ShootSKD::POSITIVE /* of no use */, THREAD_SHOOT_SKD_PRIO);
    ShootSKD::load_pid_params(SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                              {0, 0, 0, 0, 0} /* of no use */, {0, 0, 0, 0, 0} /* of no use */,
                              SHOOT_PID_FW_LEFT_V2I_PARAMS, SHOOT_PID_FW_RIGHT_V2I_PARAMS);

    /// Start LGs
    GimbalLG::init();
    ShootLG::init(SHOOT_DEGREE_PER_BULLET, THREAD_SHOOT_LG_STUCK_DETECT_PRIO, THREAD_SHOOT_BULLET_COUNTER_PRIO);


    /// Start Inspector and User Threads
    // FIXME: Re-enable InspectorA
//    InspectorA::start_inspection(THREAD_INSPECTOR_PRIO);
    InspectorA::start_referee_inspection(THREAD_INSPECTOR_REFEREE_PRIO);
    UserA::start(THREAD_USER_PRIO, THREAD_USER_ACTION_PRIO, THREAD_USER_CLIENT_DATA_SEND_PRIO);

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
    chibios_rt::BaseThread::setPriority(THREAD_IDEAL_PRIO);
#endif
    return 0;
}