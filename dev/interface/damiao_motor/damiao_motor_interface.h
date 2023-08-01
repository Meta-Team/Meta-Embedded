//
// Created by Wu Feiyang on 7/31/23.
//

/**
 * @file can_motor_interface.h
 * @brief Class to handle CAN motor feedback data.
 *
 * @addtogroup CAN Driver
 * @{
 */

#ifndef DAMIAO_MOTOR_INTERFACE_H
#define DAMIAO_MOTOR_INTERFACE_H

#include "can_interface.h"
#include "damiao_motor_feedback.h"

/**
 * @author Wu Feiyang
 * @brief  Interface for Damiao motors.
 */

class DamiaoMotorIF{
public:
    static DamiaoMotorFeedback motor_feedback[DamiaoMotorCFG::MOTOR_COUNT];
    static motor_mode_t motors_mode[DamiaoMotorCFG::MOTOR_COUNT];

private:

    static CANTxFrame motors_can_tx_frame[DamiaoMotorCFG::MotorName::MOTOR_COUNT];

    static CANInterface * can1;

    static CANInterface * can2;

    static constexpr uint64_t start_cmd = 0xfcffffffffffffff;

    static constexpr uint64_t stop_cmd = 0xfdffffffffffffff;

    static constexpr uint64_t save_zero_cmd = 0xfeffffffffffffff;

    static constexpr uint64_t clear_error_cmd = 0xffffffffffffffff;

    static float uint_to_float(int x_int, float x_min, float x_max, int bits){
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

    static int float_to_uint(float x, float x_min, float x_max, int bits){
        /// Converts a float to an unsigned int, given range and number of bits
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

    /**
     * @brief initialize the damiao motor interface.
     * @param can1_ Pointer of Can driver 1.
     * @param can2_ Pointer of Can driver 2.
     */
    static void init(CANInterface *can1_, CANInterface *can2_);

    /**
     * @brief start and enable the damiao motor by sending out the start command.
     * @param motorProfile
     */
    static void start(DamiaoMotorCFG::MotorName motorProfile);

    /**
     * @brief stop and disable the damiao motor by sending out the stop command.
     * @param motorProfile
     */
    static void stop(DamiaoMotorCFG::MotorName motorProfile);

    /**
     * @brief set the mode of the Damiao Motor. Namely, MIT_MODE, POS_VEL_MODE, VEL_MODE.
     * @param motorProfile
     * @param mode
     */
    static void set_mode(DamiaoMotorCFG::MotorName motorProfile, motor_mode_t mode);

    /**
     * @brief Set the target velocity of the motor. Compatible for all modes.
     * @param motorProfile
     * @param vel
     */
    static void set_velocity(DamiaoMotorCFG::MotorName motorProfile, float vel);

    /**
     * @brief Set the target position of the motor. Compatible for all modes.
     * @param motorProfile
     * @param pos
     */
    static void set_position(DamiaoMotorCFG::MotorName motorProfile, float pos);

    /**
     * @brief Set the target torque of the motor. Compatible for MIT mode only.
     * @param motorProfile
     * @param torq
     */
    static void set_torque(DamiaoMotorCFG::MotorName motorProfile, float torq);

    /**
     * @brief set the kp and kd parameters of MIT_MODE.
     * @param motorProfile
     * @param kp
     * @param kd
     */
    static void set_param_MIT(DamiaoMotorCFG::MotorName motorProfile,float kp,float kd);

    /**
     * @brief Post Can TX message.
     * @param motorProfile
     * @return true for success.
     */
    static bool postMsg(DamiaoMotorCFG::MotorName motorProfile);

    /**
     * @brief CAN1 callback function.
     * @param rxmsg
     */
    static void can1_callback_func(CANRxFrame const *rxmsg);

    /**
     * @brief CAN2 callback function.
     * @param rxmsg
     */
    static void can2_callback_func(CANRxFrame const *rxmsg);

    friend class DamiaoMotorController;

};

#endif //DAMIAO_MOTOR_INTERFACE_H

/** @} */