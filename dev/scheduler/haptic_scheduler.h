//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_HAPTIC_SCHEDULER_H
#define META_INFANTRY_HAPTIC_SCHEDULER_H

#include "can_motor_interface.h"
#include "pid_controller.hpp"

using namespace chibios_rt;

class haptic_scheduler {
public:

    static void start(tprio_t SKD_PRIO, tprio_t FB_PRIO);

    /**
     * @brief Load PID parameters.
     * @param id        The corresponding motor PID to change.
     * @param is_a2v    Identify whether it changes a2v or v2i parameters
     * @param params    The PID parameters.
     * */
    static void load_PID_params(can_motor_interface::motor_id_t id, bool is_a2v, PIDController::pid_params_t params);

    /**
     * @brief Switch the motor to display.
     * @param id        The corresponding id of the motor that tend to show feedback.
     * */
    static void switch_feedback_motor(can_motor_interface::motor_id_t id);

private:

    class feedbackThread : public BaseStaticThread<512> {
    public:
        can_motor_interface::motor_id_t disp_id = (can_motor_interface::motor_id_t)((int)can_motor_interface::MOTOR_COUNT-1);
    private:
        void main() final;
    };
    static feedbackThread FeedbackThread;

    class skdThread : public BaseStaticThread<512> {
    private:
        float target[can_motor_interface::MOTOR_COUNT];
        float targetV[can_motor_interface::MOTOR_COUNT];
        int   output[can_motor_interface::MOTOR_COUNT];
        void main() final;
        friend feedbackThread;
    };
    static skdThread SKDThread;

    static PIDController a2vController[can_motor_interface::MOTOR_COUNT];
    static PIDController v2iController[can_motor_interface::MOTOR_COUNT];
};


#endif //META_INFANTRY_HAPTIC_SCHEDULER_H
