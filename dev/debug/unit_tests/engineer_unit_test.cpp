//
// Created by Kerui Zhu on 7/10/2019.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"

#include "engineer_chassis_interface.h"
#include "engineer_elevator_interface.h"

#include "engineer_chassis_skd.h"
#include "engineer_elevator_skd.h"

using namespace chibios_rt;
float c_kp, c_ki, c_kd, c_i_limit, c_out_limit;
float e_kp, e_ki, e_kd, e_i_limit, e_out_limit;
float a_kp, a_ki, a_kd, a_i_limit, a_out_limit;

CANInterface can1(&CAND1);

class EngineerFeedbackThread: public BaseStaticThread<256>{
public:
    bool chassis = false, elevator = false, aided_motor = false;
    void main()final {
        setName("EngineerFeedback");
        while (!shouldTerminate()){
            if (chassis){
                LOG("%.2f, %.2f, %d, %d", EngineerChassisIF::motors[0].actual_velocity, EngineerChassisSKD::target_velocity[0],
                    EngineerChassisIF::motors[0].actual_current_raw, EngineerChassisIF::motors[0].target_current);
            } else if (elevator){
                LOG("%.2f, %.2f, %d, %d", EngineerElevatorIF::elevatorMotor[0].actual_velocity, EngineerElevatorSKD::target_velocity[0],
                    EngineerElevatorIF::elevatorMotor[0].actual_current, EngineerElevatorIF::elevatorMotor[0].target_current);
            } else if (aided_motor){
                LOG("%.2f, %.2f, %d", EngineerElevatorIF::aidedMotor[0].actual_velocity, EngineerElevatorSKD::target_velocity[1], EngineerElevatorIF::aidedMotor[0].target_current);
            }
        }
    }
}engineerFeedbackThread;

static void cmd_chassis_echo(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_echo");
        return;
    }
}

static void cmd_chassis_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "c_enable 0/1");
        return;
    }
    if (Shell::atoi(argv[0])) EngineerChassisSKD::unlock();
    else EngineerChassisSKD::lock();
}

static void cmd_set_v2i(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 6) {
        shellUsage(chp, "set_v2i 0(chassis)/1(elevator)/2(aided_motor) kp ki kd i_limit out_limit");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i == 0)
        EngineerChassisSKD::change_pid_params({c_kp = Shell::atof(argv[1]),
                                               c_ki = Shell::atof(argv[2]),
                                               c_kd = Shell::atof(argv[3]),
                                               c_i_limit = Shell::atof(argv[4]),
                                               c_out_limit = Shell::atof(argv[5])});
    else if (i == 1)
        EngineerElevatorSKD::change_pid_params(0, {e_kp = Shell::atof(argv[1]),
                                                   e_ki = Shell::atof(argv[2]),
                                                   e_kd = Shell::atof(argv[3]),
                                                   e_i_limit = Shell::atof(argv[4]),
                                                   e_out_limit = Shell::atof(argv[5])});
    else if (i == 2)
        EngineerElevatorSKD::change_pid_params(1, {a_kp = Shell::atof(argv[1]),
                                                   a_ki = Shell::atof(argv[2]),
                                                   a_kd = Shell::atof(argv[3]),
                                                   a_i_limit = Shell::atof(argv[4]),
                                                   a_out_limit = Shell::atof(argv[5])});
    LOG("pass!");
}

static void cmd_chassis_set_velocity(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "c_set_v vx vy w");
        return;
    }
    EngineerChassisSKD::set_velocity(Shell::atof(argv[0]),Shell::atof(argv[1]),Shell::atof(argv[2]));
}

static void cmd_echo_fb(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "echo_fb 0(chassis)/1(elevator)/2(aided_motor)");
        return;
    }
    engineerFeedbackThread.chassis = engineerFeedbackThread.elevator = engineerFeedbackThread.aided_motor = false;
    int i = Shell::atoi(argv[0]);
    if (i == 0) engineerFeedbackThread.chassis = true;
    else if (i == 1) engineerFeedbackThread.elevator = true;
    else if (i == 2) engineerFeedbackThread.aided_motor = true;
}

static void cmd_echo_v2i(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "echo_v2i 0(chassis)/1(elevator)/2(aided_motor)");
        return;
    }
    int i = Shell::atoi(argv[0]);
    if (i == 0) LOG("%.2f, %.2f, %.2f, %.2f, %.2f", c_kp, c_ki, c_kd, c_i_limit, c_out_limit);
    else if (i == 1) LOG("%.2f, %.2f, %.2f, %.2f, %.2f", e_kp, e_ki, e_kd, e_i_limit, e_out_limit);
    else if (i == 2) LOG("%.2f, %.2f, %.2f, %.2f, %.2f", a_kp, a_ki, a_kd, a_i_limit, a_out_limit);
}



ShellCommand chassisCommands[] = {
        {"c_enable",        cmd_chassis_enable},
        {"set_v2i",         cmd_set_v2i},
        {"c_set_v",         cmd_chassis_set_velocity},
        {"echo_fb",         cmd_echo_fb},
        {"echo_v2i",        cmd_echo_v2i},
        {nullptr,    nullptr}
};

int main(){
    halInit();
    System::init();
    LED::green_off();
    LED::red_off();

    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);

    can1.start(HIGHPRIO - 1);
    EngineerChassisIF::init(&can1);
    EngineerElevatorIF::init(&can1);
    EngineerChassisSKD::engineerChassisThread.start(NORMALPRIO);
    EngineerElevatorSKD::engineerElevatorThread.start(NORMALPRIO - 1);

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
    BaseThread::setPriority(1);
#endif
    return 0;
}