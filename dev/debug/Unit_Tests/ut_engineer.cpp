//
// Created by Kerui Zhu on 7/10/2019.
// Modified by LaiXinyi on 7/16/2019.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "can_interface.h"
#include "common_macro.h"

#include "chassis_interface.h"
#include "engineer_elevator_interface.h"
#include "robotic_arm_interface.h"

#include "engineer_chassis_skd.h"
#include "engineer_elevator_skd.h"
#include "robotic_arm_skd.h"

#include "engineer_elevator_logic.h"

float c_kp, c_ki, c_kd, c_i_limit, c_out_limit;
float e_kp, e_ki, e_kd, e_i_limit, e_out_limit;
float a_kp, a_ki, a_kd, a_i_limit, a_out_limit;

adcsample_t data[4];

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class EngineerFeedbackThread: public chibios_rt::BaseStaticThread<512>{
public:
    bool chassis = false, elevator = false, aided_motor = false, dms = false;
    void main()final {
        setName("EngineerFeedback");
        while (!shouldTerminate()){

            DMSInterface::get_raw_sample(data);

            if (chassis){
                LOG("act_v, tgt_v, act_I, tgt_I: %.2f, %.2f, %d, %d", ChassisIF::feedback[0].actual_velocity, EngineerChassisSKD::target_velocity[0],
                    ChassisIF::feedback[0].actual_current_raw, ChassisIF::target_current[0]);
            } else if (elevator){
                LOG("act_h, act_v, tgt_v, act_I, tgt_I: %.2f, %.2f, %.2f, %d, %d", EngineerElevatorIF::get_current_height(), EngineerElevatorIF::elevatorMotor[0].actual_velocity, EngineerElevatorSKD::target_velocity[0],
                    EngineerElevatorIF::elevatorMotor[0].actual_current, EngineerElevatorIF::elevatorMotor[0].target_current);
            } else if (aided_motor){
                LOG("act_v, tgt_v, tgt_I, %.2f, %.2f, %d", EngineerElevatorIF::aidedMotor[0].actual_velocity, EngineerElevatorSKD::target_velocity[2], EngineerElevatorIF::aidedMotor[0].target_current);
            } else if (dms){
                LOG("FR, FL, BL, BR: %u %u %u %u", data[DMSInterface::FR], data[DMSInterface::FL], data[DMSInterface::BL], data[DMSInterface::BR]);
                dms = false;
            }
            sleep(TIME_MS2I(100));
        }
    }
}engineerFeedbackThread;

static void cmd_enable(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "enable 0(chassis)/1(elevator)/2(aided_motor)");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i == 0) EngineerChassisSKD::unlock();
    else if (i == 1) EngineerElevatorSKD::elevator_enable(true);
    else if (i == 2) EngineerElevatorSKD::aided_motor_enable(true);
}

static void cmd_disable(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "disable 0(chassis)/1(elevator)/2(aided_motor)");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i == 0) EngineerChassisSKD::lock();
    else if (i == 1) EngineerElevatorSKD::elevator_enable(false);
    else if (i == 2) EngineerElevatorSKD::aided_motor_enable(false);
}

static void cmd_echo_fb(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "echo_fb 0(chassis)/1(elevator)/2(aided_motor)/3(dms)");
        return;
    }
    engineerFeedbackThread.chassis = engineerFeedbackThread.elevator = engineerFeedbackThread.aided_motor = engineerFeedbackThread.dms = false;
    int i = Shell::atoi(argv[0]);
    if (i == 0) engineerFeedbackThread.chassis = true;
    else if (i == 1) engineerFeedbackThread.elevator = true;
    else if (i == 2) engineerFeedbackThread.aided_motor = true;
    else if (i == 3) engineerFeedbackThread.dms = true;
}

static void cmd_stop_echo_fb(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "stop_fb");
        return;
    }
    engineerFeedbackThread.chassis = engineerFeedbackThread.elevator = engineerFeedbackThread.aided_motor = engineerFeedbackThread.dms = false;
}

static void cmd_set_v2i(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 6) {
        shellUsage(chp, "set_v2i 0(chassis)/1(elevator)/2(aided_motor) kp ki kd i_limit out_limit");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i == 0)
        EngineerChassisSKD::load_pid_params({c_kp = Shell::atof(argv[1]),
                                             c_ki = Shell::atof(argv[2]),
                                             c_kd = Shell::atof(argv[3]),
                                             c_i_limit = Shell::atof(argv[4]),
                                             c_out_limit = Shell::atof(argv[5])});
//    else if (i == 1)
//        EngineerElevatorSKD::change_pid_params(0, {e_kp = Shell::atof(argv[1]),
//                                                   e_ki = Shell::atof(argv[2]),
//                                                   e_kd = Shell::atof(argv[3]),
//                                                   e_i_limit = Shell::atof(argv[4]),
//                                                   e_out_limit = Shell::atof(argv[5])});
    else if (i == 2)
        EngineerElevatorSKD::set_aided_pid_params( {a_kp = Shell::atof(argv[1]),
                                                   a_ki = Shell::atof(argv[2]),
                                                   a_kd = Shell::atof(argv[3]),
                                                   a_i_limit = Shell::atof(argv[4]),
                                                   a_out_limit = Shell::atof(argv[5])});
//    LOG("not available!");
}

static void cmd_echo_v2i(BaseSequentialStream *chp, int argc, char *argv[]){
//    (void) argv;
//    if (argc != 1) {
//        shellUsage(chp, "echo_v2i 0(chassis)/1(elevator)/2(aided_motor)");
//        return;
//    }
//    int i = Shell::atoi(argv[0]);
//    if (i == 0) LOG("%.2f, %.2f, %.2f, %.2f, %.2f", c_kp, c_ki, c_kd, c_i_limit, c_out_limit);
//    else if (i == 1) LOG("%.2f, %.2f, %.2f, %.2f, %.2f", e_kp, e_ki, e_kd, e_i_limit, e_out_limit);
//    else if (i == 2) LOG("%.2f, %.2f, %.2f, %.2f, %.2f", a_kp, a_ki, a_kd, a_i_limit, a_out_limit);
    LOG("not available");
}


static void cmd_elevator_set_free(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "free");
        return;
    }
    // for debugging the elevator separately
    EngineerElevatorLG::set_action_free();
}

static void cmd_chassis_set_velocity(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "c_set_v vx vy w");
        return;
    }
    EngineerChassisSKD::set_velocity(Shell::atof(argv[0]),Shell::atof(argv[1]),Shell::atof(argv[2]));
}

static void cmd_aided_motor_set_velocity(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "a_set_v v (degree/s)");
        return;
    }
    float v = Shell::atof(argv[0]);
    EngineerElevatorSKD::set_aided_motor_velocity(v, v);
}

static void cmd_elevator_set_height(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "e_set_h h (cm)");
        return;
    }
    float new_height = Shell::atof(argv[0]);
    EngineerElevatorSKD::set_target_height(new_height);
}

static void cmd_elevator_clear_height(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "e_clear_h");
        return;
    }
    EngineerElevatorIF::elevatorMotor[0].clear_accumulate_angle();
    EngineerElevatorIF::elevatorMotor[1].clear_accumulate_angle();
}



static void cmd_stop_elev(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s");
        return;
    }
    EngineerElevatorLG::pause_action();
}

static void cmd_elevator_quit_action(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "quit");
        return;
    }
    EngineerElevatorLG::quit_action();
}

static void cmd_elevator_cont_action(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "cont");
        return;
    }
    EngineerElevatorLG::continue_action();
}

static void cmd_reset_elev(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "reset");
        return;
    }
    EngineerElevatorLG::set_action_free();
    EngineerElevatorSKD::aided_motor_enable(false);
    EngineerElevatorSKD::elevator_enable(true);
    EngineerElevatorSKD::set_target_height(ELEVATOR_ORIGIN_HEIGHT);
}



static void cmd_auto_up(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "up");
        return;
    }
    EngineerElevatorLG::going_up();
}

static void cmd_auto_down(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "down");
        return;
    }
    EngineerElevatorLG::going_down();
}

static void cmd_set_sensor_state(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "sensor 0(reach_stage) / 1(back_landed) / 2(back_edged) / 3(front_leave_stage)");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i==0) EngineerElevatorLG::reach_stage = true;
    if (i==1) EngineerElevatorLG::back_landed = true;
    if (i==2) EngineerElevatorLG::back_edged = true;
    if (i==3) EngineerElevatorLG::front_leave_stage = true;
}

static void cmd_chassis_pivot_turn(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "pivot 0(BL) / 1(BR)");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i==0) EngineerChassisSKD::pivot_turn(ChassisBase::BL, -0.05*ENGINEER_CHASSIS_W_MAX);
    if (i==1) EngineerChassisSKD::pivot_turn(ChassisBase::BR, 0.05*ENGINEER_CHASSIS_W_MAX);
}

static void cmd_aided_motor_test(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "ttt 0(forward) / 1(backward) / 2(stop) delay_time[ms] ");
        return;
    }

    EngineerElevatorLG::delay_time = Shell::atoi(argv[1]);
    int i = Shell::atoi(argv[0]);

    if (i==0) {
        EngineerElevatorLG::a_t_forward = true;
        EngineerElevatorLG::a_t_backward = false;
    } else if (i==1) {
       EngineerElevatorLG::a_t_forward = false;
       EngineerElevatorLG::a_t_backward = true;
    } else if (i==2) {
        EngineerElevatorLG::a_t_forward = EngineerElevatorLG::a_t_backward = false;
    }
}

static void cmd_change_door(BaseSequentialStream *chp, int argc, char *argv[]){
    RoboticArmSKD::change_door();
}


ShellCommand chassisCommands[] = {
        {"enable",          cmd_enable},
        {"disable",         cmd_disable},
        {"echo_fb",         cmd_echo_fb},
        {"stop_fb",         cmd_stop_echo_fb},
        {"set_v2i",         cmd_set_v2i},
        {"echo_v2i",        cmd_echo_v2i},

//        {"free",            cmd_elevator_set_free},
//        {"c_set_v",         cmd_chassis_set_velocity},
//        {"a_set_v",         cmd_aided_motor_set_velocity},
//        {"e_set_h",         cmd_elevator_set_height},
//        {"e_clear_h",       cmd_elevator_clear_height},
//
//        {"s",               cmd_stop_elev},
//        {"cont",            cmd_elevator_cont_action},
//        {"quit",            cmd_elevator_quit_action},
//        {"reset",           cmd_reset_elev},

        {"up",              cmd_auto_up},
        {"down",            cmd_auto_down},
        {"sensor",          cmd_set_sensor_state},
        {"pivot",           cmd_chassis_pivot_turn},

        {"ttt",             cmd_aided_motor_test},

        {"door",            cmd_change_door},

        {nullptr,    nullptr}
};

int main(){
    halInit();
    chibios_rt::System::init();
    //LED::green_off();
    LED::red_off();

    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);

    can1.start(HIGHPRIO - 1);
    can2.start(HIGHPRIO - 2);

    DMSInterface::init(4);

    ChassisIF::init(&can1);
    EngineerElevatorIF::init(&can2);
    //RoboticArmIF::init(&can2);

    EngineerChassisSKD::start(CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, THREAD_CHASSIS_SKD_PRIO);
    EngineerChassisSKD::load_pid_params(CHASSIS_PID_V2I_PARAMS);
    EngineerElevatorSKD::start(THREAD_ELEVATOR_SKD_PRIO);
    EngineerElevatorSKD::load_pid_params(ELEVATOR_PID_A2V_PARAMS, ELEVATOR_PID_V2I_PARAMS, AIDED_MOTOR_PID_V2I_PARAMS, {0, 0, 0, 0, 0});

    RoboticArmSKD::roboticArmThread.start(NORMALPRIO - 2);

    EngineerElevatorLG::init(THREAD_ELEVATOR_LG_PRIO);

    engineerFeedbackThread.start(HIGHPRIO - 2);

    Buzzer::play_sound(Buzzer::sound_startup_intel, LOWPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
    // enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}