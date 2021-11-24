//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_CAN_MOTOR_SCHEDULER_H
#define META_INFANTRY_CAN_MOTOR_SCHEDULER_H

#include "can_motor_interface.h"
#include "pid_controller.hpp"
#include "VCP.h"

using namespace chibios_rt;

class can_motor_scheduler {
public:

    static void start(tprio_t SKD_PRIO, tprio_t FB_PRIO);

    /**
     * @brief Load PID parameters.
     * @param id        The corresponding motor PID to change.
     * @param is_a2v    Identify whether it changes a2v or v2i parameters
     * @param params    The PID parameters.
     * */
    static void load_PID_params(CANBUS_MOTOR_CFG::motor_id_t id, bool is_a2v, PIDController::pid_params_t params);

    /**
     * @brief Switch the motor to display.
     * @param id        The corresponding id of the motor that tend to show feedback.
     * */
    static void switch_feedback_motor(CANBUS_MOTOR_CFG::motor_id_t id);

    /**
     * @brief Get torque current from feedback
     * @param id        The corresponding id of the motor that tend to show torque current.
     * @return          Torque current in encoder unit.
     * @details         Range mapping: [-16383~16383] -> [-20A, 20A]
     * */
    static int get_torque_current(CANBUS_MOTOR_CFG::motor_id_t id);

    /**
     * @brief           Get output current calculated by PID.
     * @return          PID current.
     * @details         Range mapping: [-16383~16383] -> [-20A, 20A]
     * */
    static int get_PID_current(CANBUS_MOTOR_CFG::motor_id_t id);

    /**
     * @brief Set the target angle in PID Controller mode
     * @param id        [in] Target motor id.
     * @param target    [in] Target angle of motor.
     * */
    static void set_target_angle(CANBUS_MOTOR_CFG::motor_id_t id, float target);

    /**
     * @brief Set the target velocity in velocity PID Controller mode
     * @param id        [in] Target motor id
     * @param target    [in] Target velocity of motor
     * */
    static void set_target_vel(CANBUS_MOTOR_CFG::motor_id_t id, float target);

    /**
     * @brief Set the target current in Current Mode
     * @param id        [in] Target motor id.
     * @param target    [in] Target current of motor.
     * */
     static void set_target_current(CANBUS_MOTOR_CFG::motor_id_t id, int target);
private:

    class feedbackThread : public BaseStaticThread<512> {
    public:
        CANBUS_MOTOR_CFG::motor_id_t disp_id = (CANBUS_MOTOR_CFG::motor_id_t)((int)can_motor_interface::MOTOR_COUNT-1);
    private:
        void main() final;
    };
    static feedbackThread FeedbackThread;

    class skdThread : public BaseStaticThread<512> {
    private:
        float targetA[can_motor_interface::MOTOR_COUNT];
        float targetV[can_motor_interface::MOTOR_COUNT];
        int   output[can_motor_interface::MOTOR_COUNT];
        float PID_output[can_motor_interface::MOTOR_COUNT];
        void main() final;
        friend feedbackThread;
        friend can_motor_scheduler;
    };
    static skdThread SKDThread;

    static PIDController a2vController[can_motor_interface::MOTOR_COUNT];
    static PIDController v2iController[can_motor_interface::MOTOR_COUNT];
};


#endif //META_INFANTRY_CAN_MOTOR_SCHEDULER_H
