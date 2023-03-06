//
// Created by Chen Qian on 10/29/21.
//

/**
 * @file can_motor_scheduler.h
 * @brief Class to drive can motors.
 *
 * @addtogroup CAN Driver
 * @{
 */

#ifndef META_INFANTRY_CAN_MOTOR_SCHEDULER_H
#define META_INFANTRY_CAN_MOTOR_SCHEDULER_H

#include "can_motor_interface.h"
#include "pid_controller.hpp"

using namespace chibios_rt;

class CANMotorController: private CANMotorCFG{
public:

    /**
     * @brief           Start CAN motor scheduler thread.
     * @param SKD_PRIO  [in] Scheduler thread priority, motor controllers calculations.
     * @param FB_PRIO   [in] Feedback thread priority, log motor feedback data on shell.
     * @param can1_     [in/out] CAN interface for CAN channel 1.
     * @param can2_     [in/out] CAN interface for CAN channel 2.
     */
    static void start(tprio_t SKD_PRIO, tprio_t FB_PRIO, CANInterface *can1_, CANInterface *can2_);

    /**
     * @brief           Load PID parameters.
     * @param id        [in] The corresponding motor PID to change.
     * @param is_a2v    [in] Identify whether it changes a2v or v2i parameters
     * @param params    [in] The PID parameters.
     */
    static void load_PID_params(motor_id_t id, bool is_a2v, PIDController::pid_params_t params);

    /**
     * @brief           Switch the motor to display.
     * @param id        [in] The corresponding id of the motor that tend to show feedback.
     */
    static void switch_feedback_motor(motor_id_t id);

    /**
     * @brief           Get torque current from feedback
     * @param id        [in] The corresponding id of the motor that tend to show torque current.
     * @return          Torque current in encoder unit.
     * @details         Range mapping: [-16383~16383] -> [-20A, 20A]
     */
    static int get_torque_current(motor_id_t id);

    /**
     * @brief           Get output current calculated by PID.
     * @return          PID current.
     * @details         Range mapping: [-16383~16383] -> [-20A, 20A]
     */
    static int get_PID_current(motor_id_t id);

    /**
     * @brief Set the target angle in PID Controller mode
     * @param id        [in] Target motor id.
     * @param target    [in] Target angle of motor.
     */
    static void set_target_angle(motor_id_t id, float target);

    /**
     * @brief Set the target velocity in velocity PID Controller mode
     * @param id        [in] Target motor id
     * @param target    [in] Target velocity of motor
     */
    static void set_target_vel(motor_id_t id, float target);

    /**
     * @brief Set the target current in Current Mode
     * @param id        [in] Target motor id.
     * @param target    [in] Target current of motor.
     */
     static void set_target_current(motor_id_t id, int target);

     /**
      * @brief Customized feedback type.
      */
     enum feedback_type_t {
         angle, ///< Feedback type is angle
         velocity, ///< Feedback type is velocity
     };

     /**
      * Register the customize feedback, like AHRS angle for gimbal.
      * @param feedback_addr    [in] The address of input variable (static), SKD will continuously read the address.
      * @param fb_type          [in] Feedback_type_t, angle/velocity.
      * @param motor_id         [in] The logical motor will affect, like YAW for gimbal control.
      */
     static void register_custom_feedback(float *feedback_addr, feedback_type_t fb_type, motor_id_t motor_id);

     /**
      * Unregister the customize feedback, SKD will use build-in motor feedback for calculation.
      * @param fb_type           [in] Feedback_type_t, angle/velocity.
      * @param motor_id          [in] The logical motor will affect, like YAW for gimbal control.
      */
     static void unregister_custom_feedback(feedback_type_t fb_type, motor_id_t motor_id);
private:

    class feedbackThread : public BaseStaticThread<512> {
    public:
        motor_id_t disp_id = (motor_id_t)((int)MOTOR_COUNT - 1);
    private:
        void main() final;
    };
    static feedbackThread FeedbackThread;

    class skdThread : public BaseStaticThread<512> {
    private:
        float targetA[MOTOR_COUNT];
        float targetV[MOTOR_COUNT];
        int   output[MOTOR_COUNT]; // output current (mapped 16bit)
        float PID_output[MOTOR_COUNT];
        void main() final;
        friend feedbackThread;
        friend CANMotorController;
    };
    static skdThread SKDThread;

    static float *feedbackV[MOTOR_COUNT];
    static float *feedbackA[MOTOR_COUNT];

    static PIDController a2vController[MOTOR_COUNT];
    static PIDController v2iController[MOTOR_COUNT];
};


#endif //META_INFANTRY_CAN_MOTOR_SCHEDULER_H

/** @} */