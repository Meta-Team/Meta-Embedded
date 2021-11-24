//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_CAN_MOTOR_SCHEDULER_H
#define META_INFANTRY_CAN_MOTOR_SCHEDULER_H

#include "can_motor_interface.h"
#include "pid_controller.hpp"
#include "VirtualCOMPort.h"

using namespace chibios_rt;

class CANMotorSKD {
public:

    static void start(tprio_t SKD_PRIO, tprio_t FB_PRIO);

    /**
     * @brief Load PID parameters.
     * @param id        The corresponding motor PID to change.
     * @param is_a2v    Identify whether it changes a2v or v2i parameters
     * @param params    The PID parameters.
     * */
    static void load_PID_params(CANMotorCFG::motor_id_t id, bool is_a2v, PIDController::pid_params_t params);

    /**
     * @brief Switch the motor to display.
     * @param id        The corresponding id of the motor that tend to show feedback.
     * */
    static void switch_feedback_motor(CANMotorCFG::motor_id_t id);

    /**
     * @brief Get torque current from feedback
     * @param id        The corresponding id of the motor that tend to show torque current.
     * @return          Torque current in encoder unit.
     * @details         Range mapping: [-16383~16383] -> [-20A, 20A]
     * */
    static int get_torque_current(CANMotorCFG::motor_id_t id);

    /**
     * @brief           Get output current calculated by PID.
     * @return          PID current.
     * @details         Range mapping: [-16383~16383] -> [-20A, 20A]
     * */
    static int get_PID_current(CANMotorCFG::motor_id_t id);

    /**
     * @brief Set the target angle in PID Controller mode
     * @param id        [in] Target motor id.
     * @param target    [in] Target angle of motor.
     * */
    static void set_target_angle(CANMotorCFG::motor_id_t id, float target);

    /**
     * @brief Set the target velocity in velocity PID Controller mode
     * @param id        [in] Target motor id
     * @param target    [in] Target velocity of motor
     * */
    static void set_target_vel(CANMotorCFG::motor_id_t id, float target);

    /**
     * @brief Set the target current in Current Mode
     * @param id        [in] Target motor id.
     * @param target    [in] Target current of motor.
     * */
     static void set_target_current(CANMotorCFG::motor_id_t id, int target);
private:

    class feedbackThread : public BaseStaticThread<512> {
    public:
        CANMotorCFG::motor_id_t disp_id = (CANMotorCFG::motor_id_t)((int)CANMotorInterface::MOTOR_COUNT - 1);
    private:
        void main() final;
    };
    static feedbackThread FeedbackThread;

    class skdThread : public BaseStaticThread<512> {
    private:
        float targetA[CANMotorInterface::MOTOR_COUNT];
        float targetV[CANMotorInterface::MOTOR_COUNT];
        int   output[CANMotorInterface::MOTOR_COUNT];
        float PID_output[CANMotorInterface::MOTOR_COUNT];
        void main() final;
        friend feedbackThread;
        friend CANMotorSKD;
    };
    static skdThread SKDThread;

    static PIDController a2vController[CANMotorInterface::MOTOR_COUNT];
    static PIDController v2iController[CANMotorInterface::MOTOR_COUNT];
};


#endif //META_INFANTRY_CAN_MOTOR_SCHEDULER_H
