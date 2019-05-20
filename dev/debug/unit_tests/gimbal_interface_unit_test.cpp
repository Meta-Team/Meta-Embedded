//
// Created by liuzikai on 2018/8/6.
//

/**
 * This file contain GimbalInterface Unit Test.
 */


#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "can_interface.h"
#include "mpu6500.h"

#include "gimbal_interface.h"
#include "shoot.h"

using namespace chibios_rt;

// Duplicate of motor_id_t in GimbalInterface to reduce code
unsigned const YAW = GimbalInterface::YAW;
unsigned const PITCH = GimbalInterface::PITCH;
unsigned const BULLET = GimbalInterface::BULLET;
unsigned const PLATE = GimbalInterface::PLATE;

unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]
unsigned bullet_per_second = 0;

// Raw angle of yaw and pitch when GimbalInterface points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 620
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 5684

// Depends on the install direction of the board
#define GIMBAL_YAW_ACTUAL_VELOCITY (MPU6500::angle_speed.z)
#define GIMBAL_PITCH_ACTUAL_VELOCITY (MPU6500::angle_speed.x)

CANInterface can1(&CAND1);

class GimbalFeedbackThread : public chibios_rt::BaseStaticThread<1024> {

public:

    bool enable_yaw_feedback = false;
    bool enable_pitch_feedback = false;
    bool enable_loader_feedback = false;
    bool enable_plate_feedback = false;

private:

    void main() final {

        setName("gimbal_fb");

        while (!shouldTerminate()) {

            if (enable_yaw_feedback) {
                Shell::printf("[Yaw] angle: %.2f, v:%.2f, a: %d" SHELL_NEWLINE_STR,
                              GimbalInterface::feedback[YAW].actual_angle,
                              GIMBAL_YAW_ACTUAL_VELOCITY,
                              GimbalInterface::feedback[YAW].actual_current);
            }
            if (enable_pitch_feedback) {
                Shell::printf("[PIT] angle: %.2f, v:%.2f, a: %d" SHELL_NEWLINE_STR,
                              GimbalInterface::feedback[PITCH].actual_angle,
                              GIMBAL_PITCH_ACTUAL_VELOCITY,
                              GimbalInterface::feedback[PITCH].actual_current);
            }
            if (enable_loader_feedback){
                Shell::printf("[PIT] angle: %.2f, v: %.2f" SHELL_NEWLINE_STR,
                            GimbalInterface::feedback[BULLET].actual_angle,
                            GimbalInterface::feedback[BULLET].actual_velocity);
            }
            if (enable_plate_feedback){
                Shell::printf("[PIT] angle: %.2f, v: %.2f" SHELL_NEWLINE_STR,
                            GimbalInterface::feedback[PLATE].actual_angle,
                            GimbalInterface::feedback[PLATE].actual_velocity);
            }

            sleep(TIME_MS2I(500));
        }
    }

} gimbalFeedbackThread;

class GimbalShootThread : public chibios_rt::BaseStaticThread<1024> {
public:
    bool GimbalShootEnable = false;
protected:
    void  main() final {
        setName("shoot");
        Shoot::init(72.0f,36.0f);
        Shoot::change_pid_params({ 20.0f, 0.0f, 0.0f, 0.0f, 2000.0f});
        Shoot::change_plate_params({26.0f, 0.1f, 0.02f, 500.0f, 5000.0f});
        Shoot::calc_bullet_loader(0);
        while(!shouldTerminate()) {
            if (GimbalShootEnable) {
                Shoot::calc_bullet_loader(bullet_per_second);
            } else {
                GimbalInterface::target_current[BULLET] = GimbalInterface::target_current[PLATE] = 0;
            }
            sleep(TIME_MS2I((GIMBAL_THREAD_INTERVAL)));
        }
    }
} gimbalShootThread;
/**
 * @brief set enabled state of friction wheels
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable_fw(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (*argv[0] == '1') {
        GimbalInterface::fw_duty_cycle = 0.1;
    }
    else if (*argv[0] == '2'){
        GimbalInterface::fw_duty_cycle = 0.2;
    }
    else if (*argv[0] == '3'){
        GimbalInterface::fw_duty_cycle = 0.3;
    }
    else if (*argv[0] == '4'){
        GimbalInterface::fw_duty_cycle = 0.4;
    }
    else if (*argv[0] == '5'){
        GimbalInterface::fw_duty_cycle = 0.5;
    }
    else if (*argv[0] == '6'){
        GimbalInterface::fw_duty_cycle = 0.6;
    }
    else if (*argv[0] == '7'){
        GimbalInterface::fw_duty_cycle = 0.7;
    }
    else if (*argv[0] == '8'){
        GimbalInterface::fw_duty_cycle = 0.8;
    }
    else if (*argv[0] == '9'){
        GimbalInterface::fw_duty_cycle = 0.9;
    } else {
        GimbalInterface::fw_duty_cycle = 0;
    }
    GimbalInterface::send_gimbal_currents();
}
//static void cmd_gimbal_loader(BaseSequentialStream *chp, int argc, char *argv[]){
//    (void) argv;
//    if(argc != 3 || (float) *argv[0] < 0 || (float) *argv[1] < 0 || (float) *argv[2] < 0 ){
//        shellUsage(chp, "g_enable_loader loader_angle>0 plate_angle>0 bullet_persecond>0");
//        return;
//    }
//    else {
//        Shoot::degree_per_bullet_ = (float) *argv[0];
//      Shoot::degree_per_bullet_plate = (float) *argv[1];
//        Shoot::calc_bullet_loader((float) *argv[3]);
//    }
//    GimbalInterface::send_gimbal_currents();
//}

static void cmd_gimbal_loader (BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_enable_loader (0/n)");
        return;
    }
    bullet_per_second = Shell::atof(argv[0]);
    gimbalShootThread.GimbalShootEnable = true;
}
/**
 * @brief set feedback enable states
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1') || (*argv[2] != '0' && *argv[2] != '1') || (*argv[3] != '0' && *argv[3] != '1')) {
        shellUsage(chp, "g_enable_fb yaw(0/1) pitch(0/1) bullet(0/1) plate(0/1)");
        return;
    }
    gimbalFeedbackThread.enable_yaw_feedback = *argv[0] - '0';
    gimbalFeedbackThread.enable_pitch_feedback = *argv[1] - '0';
    gimbalFeedbackThread.enable_loader_feedback = *argv[2] - '0';
    gimbalFeedbackThread.enable_plate_feedback = *argv[3] - '0';
}


/**
 * @brief set front_angle_raw with current actual angle
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_fix_front_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_fix");
        return;
    }
    GimbalInterface::feedback[YAW].reset_front_angle();
    GimbalInterface::feedback[PITCH].reset_front_angle();
}

/**
 * @brief set target currents of yaw and pitch
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "g_set yaw_current pitch_current bullet_loader_current bullet_plate_current");
        return;
    }

    GimbalInterface::target_current[YAW] = Shell::atoi(argv[0]);
    GimbalInterface::target_current[PITCH] = Shell::atoi(argv[1]);
    GimbalInterface::target_current[BULLET] = Shell::atoi(argv[2]);
    GimbalInterface::target_current[PLATE] = Shell::atoi(argv[3]);
    chprintf(chp, "Gimbal yaw target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::target_current[YAW]);
    chprintf(chp, "Gimbal pitch target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::target_current[PITCH]);
    chprintf(chp, "Gimbal bullet loader_target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::target_current[BULLET]);
    chprintf(chp, "Gimbal plate target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::target_current[PLATE]);
}


// Command lists for GimbalInterface test
ShellCommand gimbalCotrollerCommands[] = {
        {"g_enable_fb",   cmd_gimbal_enable_feedback},
        {"g_fix",         cmd_gimbal_fix_front_angle},
        {"g_enable_fw",   cmd_gimbal_enable_fw},
        {"g_set", cmd_gimbal_set_target_currents},
        {"g_enable_loader",cmd_gimbal_loader},

        {nullptr,         nullptr}
};

class GimbalDebugThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("gimbal");
        while (!shouldTerminate()) {
            GimbalInterface::send_gimbal_currents();
            sleep(TIME_MS2I(GIMBAL_THREAD_INTERVAL));
        }
    }
} gimbalThread;


int main(void) {

    halInit();
    System::init();
    LED::all_off();
    Shell::start(HIGHPRIO);
    Shell::addCommands(gimbalCotrollerCommands);
    can1.start(HIGHPRIO - 1);
    MPU6500::start(HIGHPRIO - 2);
    chThdSleepMilliseconds(10);
    GimbalInterface::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);

    gimbalFeedbackThread.start(NORMALPRIO - 1);
    gimbalThread.start(NORMALPRIO);
    gimbalShootThread.start(NORMALPRIO-2);

    chThdSleepMilliseconds(100);
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalInterface::feedback[YAW].last_angle_raw, GimbalInterface::feedback[YAW].actual_angle,
        GimbalInterface::feedback[PITCH].last_angle_raw, GimbalInterface::feedback[PITCH].actual_angle);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}