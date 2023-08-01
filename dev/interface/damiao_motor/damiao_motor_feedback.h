//
// Created by Wu Feiyang on 7/14/23.
//

/**
 * @file can_motor_feedback.h
 * @brief Class to store CAN motor feedback data
 *
 * @addtogroup CAN driver
 * @{
 */

#ifndef DAMIAO_MOTOR_FEEDBACK_H
#define DAMIAO_MOTOR_FEEDBACK_H

//#include "damiao_motor_interface.h"
#include "common_macro.h"
#include "damiao_motor_config.h"

/**
 * @author  Wu Feiyang
 * @brief   Damiao Motor interface for single motor.
 * @usage   1. Create a instance. \n
 *          2. Call init() function. \n
 *          3. Pass CANRxFrame to the process_feedback() function.\n
 *          4. Get the available feedback from this class.
 */
class DamiaoMotorFeedback{
public:

    /**
     * @brief Initialize the motorIF. Including its type and initial encoder angle.
     * @param   motor_type_             The corresponding motor's type.
     * @param   initial_encoder_angle   The encoder's reading at zero pose. Works best when reduce ratio is 1.
     */
    void init(DamiaoMotorCFG::MotorName motor_name, float initial_encoder_angle);

    /**
     * @brief   [ms]        Last time data updated. Could be used for check CAN network.
     */
    time_msecs_t last_update_time = SYSTIME;

    /**
     * @brief   [Rad]        Last angle.
     */
    float last_angle;
    /**
     * @brief   [Degree/Sec]Actual output shaft velocity for motors.
     */
    float actual_velocity = 0.0f;

    /**
     * @brief   [Degree]    Actual motor angle vary from -180 to 180.
     */
    float actual_angle = 0.0f;
    /**
     * @brief   [Degree]    Actual motor angle vary from -180 to 180.
     */
    float actual_torque = 0.0f;
    /**
     * @brief   [rounds]    The number of rounds motor rotate.
     */
    int round_count = 0;

    /*===========================================================================*/
    /*                        Integrated Feedback Methods                        */
    /*===========================================================================*/

    /**
     * @brief               Get accumulate angle of the motor.
     * @return  [radius]    Accumulate angle of the motor in radius.
     */
    float accumulate_angle();

    /**
     * @brief               Get the output torque of the motor.
     * @return  [Nm]        Output torque of the motor.
     */
    float torque() const;

    /**
     * @brief Reset the recorded angle of motor.
     */
    void reset_accumulate_angle();

    /*===========================================================================*/
    /*                        Feedback Processing  Method                        */
    /*===========================================================================*/

    /**
     * @brief Process feedback from can received can RX message. It will update the feedback in class. \n
     *        Often called in an overall feedback function (create the link of motor ID and SID). \n
     *        Then registered the feedback function in CANInterface by \n
     *        register_callback_func() method.
     * @param rxmsg CAN Rx Frame.
     */
    void process_feedback(const CANRxFrame *rxmsg);

    /*===========================================================================*/
    /*                          Raw Data for Calculation                         */
    /*===========================================================================*/
    uint8_t ctr_id;

    uint8_t err_code;

    uint16_t pos_raw;

    uint16_t vel_raw;

    uint16_t torque_raw;

    uint8_t mos_avg_tempr_raw;

    uint8_t rotor_avg_tempr_raw;
private:
//    uint16_t last_pos_raw;

    DamiaoMotorCFG::MotorName motor_name_;

    float initial_encoder_angle_;

    float raw2actrual(uint16_t raw,float actual_max,uint8_t bits);

    uint16_t actual2raw(float actual, float actual_max, uint8_t bits);
};


#endif //DAMIAO_MOTOR_FEEDBACK_H

/** @} */