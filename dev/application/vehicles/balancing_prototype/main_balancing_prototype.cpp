//
// Created by Tianyi Han on 4/21/2023.
//
#define PI 3.141592653589793238462643

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "can_motor_controller.h"
#include "hardware_conf.h"

#include "ahrs_c_interface.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard_C ahrs;

// Rotation matrix from the IMU frame to compass frame
static constexpr Matrix33 IMU_COMPASS_ROTATION_MATRIX = {{-1.0f, 0.0f, 0.0f},
                                                         {0.0f, -1.0f, 0.0f},
                                                         {0.0f, 0.0f, 1.0f}};

// Rotation matrix from the compass frame to board frame
static constexpr Matrix33 COMPASS_BOARD_ROTATION_MATRIX = {{0.0f,  0.0f, 1.0f},
                                                           {0.0f,  1.0f,  0.0f},
                                                           {-1.0f, 0.0f,  0.0f}};

class BalancingControlThread : public BaseStaticThread<1024> {
protected:
    // Balancing controller
    const float kp = 250000.0, kd = 10000.0, ki = 0.0;
    float phi_desired = -0.050;
    float phi_current = 0;
    float error_phi;
    float phi_integral = 0;
    float phi_dot, phi_dot_filtered;
    float phi_old = 0;
    float phi_dot_old[2] = {0, 0};
    int tau = 0;

    // Trajectory tracking feedback controller
    const float kp_t = 0.1, kd_t = 0.0, ki_t = 0.0;
    float theta_desired = 0;
    float theta_current;
    int theta_revolution = 0;
    float error_theta;
    float theta_integral = 0;
    float theta_dot, theta_dot_filtered;
    float theta_old = 0;
    float theta_dot_old[4] = {0, 0, 0, 0};
    float phi_compensation = 0;

    void main() final {
        setName("BalancingControl");
        while (!shouldTerminate()) {
            // Fetch angle data
            Vector3D angle = ahrs.get_angle();
            phi_current = angle.y;
            theta_current = CANMotorIF::motor_feedback[CANMotorCFG::MOTOR1].actual_angle;
            theta_current = theta_current / 180.0 * PI + PI;

            /// Trajectory tracking feedback controller
            // Calculate state
            if ((theta_old < (PI * 0.5)) && ((PI * 1.5) < theta_current)){
                theta_revolution--;
                theta_dot = (theta_current - theta_old - 2 * PI) * 1000.0;
            }
            else if ((theta_current < (PI * 0.5)) && ((PI * 1.5) < theta_old)){
                theta_revolution++;
                theta_dot = (theta_current - theta_old + 2 * PI) * 1000.0;
            }
            else {
                theta_dot = (theta_current - theta_old);
            }
            error_theta = theta_desired - (2 * PI * theta_revolution + theta_current);
            theta_dot_filtered = (theta_dot + theta_dot_old[0] + theta_dot_old[1] + theta_dot_old[2] + theta_dot_old[3]) / 5.0;
            theta_integral += (PI * theta_revolution + theta_current) * 0.001;
            theta_integral = fmax(fmin(theta_integral, 1), -1);

            // Calculate body angle compensation
            phi_compensation = fmax(fmin(kp_t * error_theta - kd_t * theta_dot_filtered + ki_t * theta_integral, PI), -PI);
            Shell::printf("%.4f\t%.4f" ENDL, error_theta, phi_compensation);

            // Save state
            theta_dot_old[3] = theta_dot_old[2];
            theta_dot_old[2] = theta_dot_old[1];
            theta_dot_old[1] = theta_dot_old[0];
            theta_dot_old[0] = theta_dot_filtered;
            theta_old = theta_current;

            /// Balancing controller

            // Calculate state
            error_phi = phi_compensation + phi_desired- phi_current;
            phi_dot = (phi_current - phi_old) * 1000.0;
            phi_dot_filtered = (phi_dot + phi_dot_old[0] + phi_dot_old[1]) / 3.0;
            phi_integral += phi_current * 0.001;
            phi_integral = fmax(fmin(phi_integral, 1), -1);

            // Calculate tau (motor target)
            tau = int(fmax(fmin(kp * error_phi - kd * phi_dot_filtered + ki * phi_integral, 20000), -20000));

            // Save state
            phi_dot_old[1] = phi_dot_old[0];
            phi_dot_old[0] = phi_dot_filtered;
            phi_old = phi_current;

            // Assign motor target
            CANMotorController::set_target_current(CANMotorCFG::MOTOR1, tau);

            chThdSleepMicroseconds(1);
        }
    }
} ControlThread;

int main(void) {
    halInit();
    System::init();

    Shell::start(HIGHPRIO);
    LED::all_off();
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);

    ahrs.start(IMU_COMPASS_ROTATION_MATRIX, COMPASS_BOARD_ROTATION_MATRIX, NORMALPRIO + 4);

    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);
    // chThdSleepMilliseconds(500);

    // Start balancing control thread
    ControlThread.start(NORMALPRIO+5);

    // Indicating thread running
    LED::green_on();
    BuzzerSKD::init(LOWPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_dji);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}