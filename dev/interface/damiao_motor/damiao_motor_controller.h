//
// Created by WuFeiyang on 10/29/21.
//

/**
 * @file can_motor_scheduler.h
 * @brief Class to drive can motors.
 *
 * @addtogroup Damiao Driver
 * @{
 */

#ifndef DAMIAO_MOTOR_CONTROLLER_H
#define DAMIAO_MOTOR_CONTROLLER_H

#include "damiao_motor_interface.h"
#include "damiao_motor_feedback.h"


using namespace chibios_rt;
class DamiaoMotorController{
public:

    /**
     * @brief           Start Damiao Motor scheduler thread.
     * @param SKD_PRIO  [in] Scheduler thread priority, motor controllers calculations.
     * @param FB_PRIO   [in] Feedback thread priority, log motor feedback data on shell.
     * @param can1_     [in/out] CAN interface for CAN channel 1.
     * @param can2_     [in/out] CAN interface for CAN channel 2.
     */
    static void start(tprio_t SKD_PRIO, tprio_t FB_PRIO, CANInterface *can1_, CANInterface *can2_);

    /**
     * @brief          Shell display enable or disable.
     * @param name     [in] The motor name to enable the feedback display.
     * @param enable   [in] Enable the shell display.
     */
    static void shell_display(DamiaoMotorCFG::MotorName name, bool enable);

    /**
     * @brief           Set the target angle of the motor.
     * @param name     [in] The motor name.
     * @param target   [in] target angle of the motor to be controlled
     */
    static void set_target_angle(DamiaoMotorCFG::MotorName name, float target);

    /**
     * @brief           Set the target angle of the motor.
     * @param name     [in] The motor name.
     * @param target   [in] target angle of the motor to be controlled
     */
    static void set_target_vel(DamiaoMotorCFG::MotorName name, float target);

     /**
      * @brief Customized feedback type.
      */
     enum feedback_type_t {
         angle, ///< Feedback type is angle
         velocity, ///< Feedback type is velocity
     };

    /**
     * @brief           Get the torque of the motor.
     * @param name     [in] The motor name.
     */
    static float get_torque(DamiaoMotorCFG::MotorName name);

    /**
     * @brief           Get the error code of the motor.
     * @param name     [in] The motor name.
     * @return integer error code
     *   8--Voltage too high；
     *   9--Voltage too low；
     *   A--Current too high；
     *   B--MOS over heating；
     *   C--Motor Cable over heating；
     *   D--Lost communication；
     *   E--Overload；
     */
    static int get_error(DamiaoMotorCFG::MotorName name);

    /**
     * @brief           Get the position of the motor.
     * @param name     [in] The motor name.
     * @return         [out] Angel value in radius.
     */
    static float get_position_angel(DamiaoMotorCFG::MotorName name);

    /**
     * @brief           Get the MOS temprature of the motor.
     * @param name     [in] The motor name.
     * @return         [out] Temperature of the motor.
     */
    static float get_mos_temprature(DamiaoMotorCFG::MotorName name);

    /**
     * @brief           Get the rotor temprature of the motor.
     * @param name     [in] The motor name.
     * @return         [out] Temperature of the rotor.
     */
    static float get_rotor_temprature(DamiaoMotorCFG::MotorName name);

    /**
     * @brief          Get the ID of the motor.
     * @param name     [in] The motor name.
     * @return         [out] The ID of the motor.
     */
    static int get_ID(DamiaoMotorCFG::MotorName name);

    /**
     * @brief          Enable the Damiao Motor.
     * @param name     [in] The motor name.
     */
    static void motor_enable(DamiaoMotorCFG::MotorName name);

    /**
     * @brief          Disable the Damiao Motor.
     * @param name     [in] The motor name.
     */
    static void motor_disable(DamiaoMotorCFG::MotorName name);

    /**
     * @brief          Set the motor control parameters in MIT mode.
     * @param name     [in] The motor name.
     * @param pos      [in] The target position.
     * @param vel      [in] The target velocity.
     * @param torque   [in] The target torque.
     */
    static void set_target_MIT(DamiaoMotorCFG::MotorName name,float pos,float vel,float torque);

    /**
      * @brief          Set the motor control parameters in position-velocity mode.
      * @param name     [in] The motor name.
      * @param pos      [in] The target position.
      * @param vel      [in] The target velocity.
     */
    static void set_target_POSVEL(DamiaoMotorCFG::MotorName name, float pos,float vel);

    /**
      * @brief          Set the motor control parameters in velocity mode.
      * @param name     [in] The motor name.
      * @param vel      [in] The target velocity.
      */
    static void set_target_VEL(DamiaoMotorCFG::MotorName name, float vel);
private:

    class feedbackThread : public BaseStaticThread<512> {
    public:
        bool enable_feedback[DamiaoMotorCFG::MOTOR_COUNT]={false};
    private:
        DamiaoMotorFeedback feedback[DamiaoMotorCFG::MOTOR_COUNT];
        void main() final;
        const int THREAD_INTERVAL = 20000; // [us]
    };
    static feedbackThread FeedbackThread;

    class skdThread : public BaseStaticThread<512> {
    private:
        void main() final;
        friend feedbackThread;
        friend DamiaoMotorController;
        const int THREAD_INTERVAL = 8000; // [us] The timeout value for sending 1 CAN frame is 1ms. In the worst case, the sending time would be 6 ms if we need to send 6 frame.
    };
    static skdThread SKDThread;

    /**
      * @brief change from degree angle to radius angel.
      * @param angle_degree     [in] Angle in degree.
      */
    static float degree2radius(float angle_degree);
};


#endif //DAMIAO_MOTOR_CONTROLLER_H

/** @} */