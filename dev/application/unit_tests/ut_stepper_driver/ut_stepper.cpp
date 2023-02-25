//
// Created by kerui on 2021/7/14.
//

// TODO: Fix the bugs in this file

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"

#include "interface/can/can_interface.h"
#include "ahrs.h"
#include "interface/sd_card/sd_card_interface.h"
#include "remote_interpreter.h"

#include "buzzer_scheduler.h"
#include "can_motor_controller.h"

#include "vehicle_pa.h"
#include "thread_priorities.h"

using namespace chibios_rt;

const char *motor_name[10] = {
        "yaw",
        "pitch",
        "sub_pitch",
        "bullet",
        "fw_left",
        "fw_right",
        "ahrs",
        "gyro",
        "accel",
        "magnet"};

// Calculation interval for gimbal thread
unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]
static float encoder_angle = 0.0f;

int const MAX_CURRENT = 30000;  // [mA]

bool feedback_enable[13];

// AHRS PARAMETERS
// Depends on the installation direction of the board
#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

static icucnt_t width, period;
static float target_angle, dir;
static float pos_vel, neg_vel;

static constexpr PWMConfig pwm_config = {
        1000000,
        1000000, // Default playing_note: 1Hz
        nullptr,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // it's all CH1 for current support boards
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr},
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr},
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}
        },
        0,
        0
};

static void step_dropcb(ICUDriver *icup) {
    width = icuGetWidthX(icup);
}

static void step_risecb(ICUDriver *icup) {
    width = icuGetWidthX(icup);
    period = icuGetPeriodX(icup);

    if (period - width > 4) {
        if(palReadPad(GPIOF, GPIOF_PIN1)) {
            dir = 1.0f;
        } else {
            dir = -1.0f;
        }
        target_angle += (dir / 200.0f / 16.0f * 360.0f);
    }

}

static ICUConfig step_cfg = {
        ICU_INPUT_ACTIVE_HIGH,
        240000,            /* 240kHz ICU clock frequency. Based on Nyquist sampling theorem,
                                     * we need at least 60kHz to sample a 30kHz signal*/
        step_dropcb,
        step_risecb,
        NULL,
        ICU_CHANNEL_2,                             /* Based on board schematic, it use PB7 connect to TIM4_CH2 */
        0
};

static const DACConfig dac_config = {
        .init         = 2047u,
        .datamode     = DAC_DHRM_12BIT_RIGHT,
        .cr           = 0
};

DEF_SHELL_CMD_START(cmd_set_enable)
    (void) argv;
    if (argc != 1) {
        Shell::printf("set_enable motor_id");
        return true;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    CANMotorCFG::enable_a2v[(CANMotorCFG::motor_id_t) motor_id] = false;
    CANMotorCFG::enable_v2i[(CANMotorCFG::motor_id_t) motor_id] = true;
}

DEF_SHELL_CMD_START(cmd_set_disable)
    (void) argv;
    if (argc != 1) {
        Shell::printf("set_disable motor_id" SHELL_NEWLINE_STR);
        return true;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    CANMotorCFG::enable_v2i[(CANMotorCFG::motor_id_t) motor_id] = false;
}

DEF_SHELL_CMD_START(cmd_enable_feedback)
    if (argc != 1) {
        Shell::printf("fb_enable feedback_id(0-9)" SHELL_NEWLINE_STR);
        return true;
    }
    unsigned feedback_id = Shell::atoi(argv[0]);
//    if (feedback_id > 9) {
//        Shell::printf("Invalid feedback ID %d" SHELL_NEWLINE_STR, feedback_id);
//        return true;
//    }
    Shell::printf("%s feedback enabled" SHELL_NEWLINE_STR, motor_name[feedback_id]);
    CANMotorController::switch_feedback_motor((CANMotorCFG::motor_id_t) feedback_id);
}

DEF_SHELL_CMD_START(cmd_disable_feedback)
    if (argc != 1) {
        Shell::printf("fb_disable feedback_id(0-9)");
        return true;
    }
    unsigned feedback_id = Shell::atoi(argv[0]);
    if (feedback_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid feedback ID %d" SHELL_NEWLINE_STR, feedback_id);
        return true;
    }
    feedback_enable[feedback_id] = false;
    Shell::printf("%s feedback disabled" SHELL_NEWLINE_STR, motor_name[feedback_id]);
}

DEF_SHELL_CMD_START(cmd_set_vel)
    (void) argv;
    if (argc != 2) {

        return true;
    }
    pos_vel = Shell::atof(argv[0]);
    neg_vel = Shell::atof(argv[1]);
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_set_param)
    (void) argv;

    if (argc != 7) {
        Shell::printf("set_pid motor_id pid_id(0: angle_to_v, 1: v_to_i) ki kp kd i_limit out_limit" SHELL_NEWLINE_STR);
        return true;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    unsigned pid_id = Shell::atoi(argv[1]);
    if (pid_id > 1) {
        Shell::printf("Invalid pid ID %d" SHELL_NEWLINE_STR, pid_id);
        return true;
    }
    PIDController::pid_params_t pid_param = {Shell::atof(argv[2]),
                                             Shell::atof(argv[3]),
                                             Shell::atof(argv[4]),
                                             Shell::atof(argv[5]),
                                             Shell::atof(argv[6])};

    if(pid_id == 1){
        CANMotorController::load_PID_params((CANMotorCFG::motor_id_t) motor_id, false, pid_param);
    } else {
        CANMotorController::load_PID_params((CANMotorCFG::motor_id_t) motor_id, true, pid_param);
    }
    Shell::printf("ps!" SHELL_NEWLINE_STR);
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_echo_param)
    (void) argv;
    if (argc != 2) {
        Shell::printf("echo_pid motor_id pid_id(0: angle_to_v, 1: v_to_i)" SHELL_NEWLINE_STR);
        return true;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    unsigned pid_id = Shell::atoi(argv[1]);
    if (pid_id > 1) {
        Shell::printf("Invalid pid ID %d" SHELL_NEWLINE_STR, pid_id);
        return true;
    }
    PIDController::pid_params_t pid_param = {0,0,0,0,0};
    pid_param = CANMotorCFG::v2iParams[(CANMotorCFG::motor_id_t) motor_id];
    Shell::printf("ki: %.2f, kp: %.2f, kd: %.2f, i_limit: %.2f, out_limit: %.2f" SHELL_NEWLINE_STR,
                 pid_param.ki, pid_param.kp, pid_param.kd, pid_param.i_limit, pid_param.out_limit);
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_set_target_angle)
    (void) argv;
    if (argc != 2) {
        Shell::printf("set_target_angle motor_id target_angle setfail" SHELL_NEWLINE_STR);
        return true;
    }
    int motor_id = Shell::atoi(argv[0]);
    float target_angle = Shell::atof(argv[1]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    CANMotorController::set_target_angle((CANMotorCFG::motor_id_t)motor_id, target_angle);
    Shell::printf("ps!" SHELL_NEWLINE_STR);
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_set_target_vel)
    (void) argv;
    if (argc != 2) {
        Shell::printf("set_target_vel motor_id target_angle setfail" SHELL_NEWLINE_STR);
        return true;
    }
    int motor_id = Shell::atoi(argv[0]);
    float target_speed = Shell::atof(argv[1]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    CANMotorController::set_target_vel((CANMotorCFG::motor_id_t)motor_id, target_speed);
    Shell::printf("ps!" SHELL_NEWLINE_STR);
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_echo_icu_freq)
    Shell::printf("Period: %d Width: %d" SHELL_NEWLINE_STR, period, width);
DEF_SHELL_CMD_END

// Command lists for gimbal controller test and adjustments
Shell::Command mainProgramCommands[] = {
        {"_a",              nullptr,   cmd_set_enable,      nullptr},
        {"set_disable",     "motor_id",   cmd_set_disable,     nullptr},
        {"fb_enable",       "feedback_id",   cmd_enable_feedback, nullptr},
        {"fb_disable",      "feedback_id",   cmd_disable_feedback,nullptr},
        {"set_pid",         "motor_id pid_id(0: angle_to_v, 1: v_to_i) ki kp kd i_limit out_limit",   cmd_set_param,       nullptr},
        {"echo_pid",        "motor_id pid_id(0: angle_to_v, 1: v_to_i)",   cmd_echo_param,      nullptr},
        {"set_target_angle","motor_id target_angle",   cmd_set_target_angle,nullptr},
        {"set_vel", "pos_vel neg_vel", cmd_set_vel, nullptr},
        {nullptr,           nullptr,  nullptr,              nullptr}
};

class controlThread : public BaseStaticThread<512> {
    void main() final;
};

class encoderThread : public BaseStaticThread<512> {
    void main() final;
};

/**
 * @brief thread for calculate encoder angle.
 */

void encoderThread::main() {
    setName("encoder");
    bool A[2]={0,0};
    bool B[2]={0,0};
    while(!shouldTerminate()) {
        // Update.
        A[0] = A[1];
        B[0] = B[1];
        A[1] = palReadPad(GPIOB,GPIOB_PIN0);
        B[1] = palReadPad(GPIOB,GPIOB_PIN1);

        if(!A[0] && A[1]){ // A detect rise
            if(!B[1]) { // Rotate CW
                encoder_angle += (1.0f/200.0f/4.0f*360.0f);
            } else { // Rotate CCW
                encoder_angle -= (1.0f/200.0f/4.0f*360.0f);
            }
        }
        if(!B[0] && B[1]){ // B detect rise
            if(!A[1]) { // Rotate CCW
                encoder_angle -= (1.0f/200.0f/4.0f*360.0f);
            } else { // Rotate CW
                encoder_angle += (1.0f/200.0f/4.0f*360.0f);
            }
        }
        if(A[0] && !A[1]){ // A detect drop
            if(B[1]) { // Rotate CW
                encoder_angle += (1.0f/200.0f/4.0f*360.0f);
            } else { // Rotate CCW
                encoder_angle -= (1.0f/200.0f/4.0f*360.0f);
            }
        }
        if(B[0] && !B[1]){ // B detect drop
            if(A[1]) { // Rotate CCW
                encoder_angle -= (1.0f/200.0f/4.0f*360.0f);
            } else { // Rotate CW
                encoder_angle += (1.0f/200.0f/4.0f*360.0f);
            }
        }
        CANMotorController::fuck_target_V = encoder_angle;
        sleep(TIME_MS2I(1));
    }
}

encoderThread enc_thd;

void controlThread::main() {
    setName("controllerThread");
    CANMotorCFG::enable_v2i[0] = true;
    auto prev_time = 0;
    float prev_angle = 0.0f;
    float printer_ratio = 1.0f*93.0f/3200.0f*360.0f;
    // We assume the target would only vary 0.5mm/s, i.e. 1.5mm/s 2.0mm/s 2.5mm/s;
    // 93 was the step size in simulink. (Varies from printer)
    // 3200 is the original full step (200 steps) * (16 micro step) / round.
    // 360 is 360.0 degree/round.

    while(!shouldTerminate()){
        CANMotorCFG::enable_v2i[0] = true;
        int outputvalue = (int) (2048.0f+(float)CANMotorIF::motor_feedback[0].torque_current()/10000.0f*2048.0f);
        dacPutChannelX(&DACD1, 0, outputvalue);
        auto target_speed =(float)((target_angle-prev_angle)/(float)(TIME_I2US(chVTGetSystemTimeX()) - prev_time))*1000000.0f;
        target_speed = (float)lround((float)target_speed/printer_ratio)*printer_ratio;
        float vellll;
        if(target_speed > 0) {
            vellll = pos_vel * printer_ratio;
        } else if(target_speed < 0) {
            vellll = neg_vel * printer_ratio;
        } else {
            vellll = 0;
        }
        CANMotorController::set_target_vel(CANMotorCFG::BULLET_LOADER,vellll);//tg*10.4625f*2.0f);
        prev_angle = target_angle;
        prev_time = TIME_I2US(chVTGetSystemTimeX());
        sleep(TIME_MS2I(40));
    }
}

static controlThread control_thread;

int main(void) {
    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    Shell::addCommands(mainProgramCommands);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    icuStart(&ICUD4, &step_cfg);
    palSetPadMode(GPIOB, GPIOB_USART1_RX, PAL_MODE_ALTERNATE(2));
    icuStartCapture(&ICUD4);
    icuEnableNotifications(&ICUD4);

    dacStart(&DACD1, &dac_config);

    pwmStart(&PWMD8, &pwm_config);
    pwmChangePeriod(&PWMD8, 1000000 / (unsigned long) 275);
    pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);
    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }
    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_RX_PRIO);
    can2.start(THREAD_CAN2_RX_PRIO);
    chThdSleepMilliseconds(5);

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
    chThdSleepMilliseconds(5);

    /// Setup Remote
    Remote::start();


    /// Setup GimbalIF (for Gimbal and Shoot)
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Start SKDs
    CANMotorController::start(THREAD_MOTOR_SKD_PRIO, THREAD_FEEDBACK_SKD_PRIO, &can1, &can2);
    enc_thd.start(THREAD_CHASSIS_SKD_PRIO);
    control_thread.start(HIGHPRIO-10);
    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup);  // Now play the startup sound


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