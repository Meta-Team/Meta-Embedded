//
// Created by liuzikai on 7/29/21.
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

#include "gimbal_interface.h"
#include "gimbal_scheduler.h"

/// Vehicle Specific Configurations

#include "vehicle_hero.h"
#include "thread_priorities.h"

/// Board Guard
#if defined(BOARD_RM_2018_A)
#else
#error "Hero supports only RM Board 2018 A currently"
#endif

static constexpr float MAX_VELOCITY[3] = {30, 10, 10};
static constexpr float MAX_ANGLE[3] = {90, 30, 0};
static constexpr float MIN_ANGLE[3] = {-90, -10, -30};

class PAUserGimbalThread : public chibios_rt::BaseStaticThread<512> {
    void main() {
        setName("PAUserGimbal");
        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_UP) {
                // Safety mode
                GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
                for (bool &e : GimbalSKD::motor_enable) e = false;
            } else {
                GimbalSKD::set_mode(GimbalSKD::ENABLED_MODE);

                int id = (int) Remote::rc.s2 - 1;
                for (int i = 0; i < 3; i++) GimbalSKD::motor_enable[i] = (id == i);

                if (Remote::rc.s1 == Remote::S_MIDDLE) {
                    // Adjust V2I
                    GimbalSKD::a2v_pid_enabled = false;
                    GimbalSKD::target_velocity[id] = Remote::rc.ch1 * MAX_VELOCITY[id];
                } else {
                    // Adjust A2V
                    GimbalSKD::a2v_pid_enabled = true;
                    GimbalSKD::target_angle[id] = Remote::rc.ch1 * (Remote::rc.ch1 >= 0 ? MAX_ANGLE[id] : -MIN_ANGLE[id]);
                }
            }

            sleep(TIME_MS2I(10));
        }
    }
} user_thread;

/// Instances
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

/// Local Constants
static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

static GimbalIF::motor_can_config_t GIMBAL_MOTOR_CONFIG_[GimbalIF::MOTOR_COUNT] = GIMBAL_MOTOR_CONFIG;

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    // Enable power of ultraviolet lights
    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);

    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }

    LED::led_on(DEV_BOARD_LED_SYSTEM_INIT);  // LED 1 on now

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_RX_PRIO, THREAD_CAN1_TX_PRIO);
    can2.start(THREAD_CAN2_RX_PRIO, THREAD_CAN2_TX_PRIO);
    chThdSleepMilliseconds(5);
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Complete Period 1
    LED::green_on();  // LED Green on now

    /// Setup On-Board AHRS
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrs.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_AHRS_PRIO);
    while(!ahrs.ready()) {
        chThdSleepMilliseconds(5);
    }
    Shell::addCommands(ahrs.shellCommands);
    Shell::addFeedbackCallback(AHRSOnBoard::cmdFeedback, &ahrs);
    LED::led_on(DEV_BOARD_LED_AHRS);  // LED 3 on now

    /// Setup Remote
    Remote::start();
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 4 on now


    /// Setup GimbalIF (for Gimbal and Shoot)
    GimbalIF::init(&can1, &can2, GIMBAL_MOTOR_CONFIG_, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW, GIMBAL_SUB_PITCH_FRONT_ANGLE_RAW, MotorIFBase::can_channel_1);
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset
    LED::led_on(DEV_BOARD_LED_GIMBAL);  // LED 5 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f, Sub Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW]->last_angle_raw, GimbalIF::feedback[GimbalIF::YAW]->accumulated_angle(),
        GimbalIF::feedback[GimbalIF::PITCH]->last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH]->accumulated_angle(),
        GimbalIF::feedback[GimbalIF::SUB_PITCH]->last_angle_raw, GimbalIF::feedback[GimbalIF::SUB_PITCH]->accumulated_angle());

    /// Start SKDs
    GimbalSKD::start(&ahrs, GIMBAL_ANGLE_INSTALLATION_MATRIX_, GIMBAL_GYRO_INSTALLATION_MATRIX_,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, GIMBAL_SUB_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO, GimbalSKD::RELATIVE_ANGLE_MODE);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS,
                               {0, 0, 0, 0, 0}/* Not used */, {0, 0, 0, 0, 0}/* Not used */);
    Shell::addCommands(GimbalSKD::shellCommands);
    Shell::addFeedbackCallback(GimbalSKD::cmdFeedback);

    user_thread.start(THREAD_USER_PRIO);


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

