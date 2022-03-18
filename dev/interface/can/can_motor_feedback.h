//
// Created by Chen Qian on 10/28/21.
//

/**
 * @file can_motor_feedback.h
 * @brief Class to store CAN motor feedback data
 *
 * @addtogroup CAN driver
 * @{
 */

#ifndef META_INFANTRY_CAN_MOTOR_FEEDBACK_H
#define META_INFANTRY_CAN_MOTOR_FEEDBACK_H

#include "can_interface.h"

struct CANMotorBase {
    enum can_channel_t{
        can_channel_1,
        can_channel_2
    } can_channel;
    int CAN_SID;
    enum motor_type_t {
        NONE_MOTOR,
        M3508,
        M3508_without_deceleration,
        GM6020,
        M2006
    } motor_type;
    uint16_t initial_encoder_angle;
};

/**
 * @author  Chen Qian
 * @brief   Motor interface for single motor.
 * @usage   1. Create a instance. \n
 *          2. Call init() function. \n
 *          3. Pass CANRxFrame to the process_feedback() function.\n
 *          4. Get the available feedback from this class.
 * */
class CANMotorFeedback : public CANMotorBase {
public:

    /**
     * @brief Initialize the motorIF. Including its type and initial encoder angle.
     * @param   motor_type_             The corresponding motor's type.
     * @param   initial_encoder_angle   The encoder's reading at zero pose. Works best when reduce ratio is 1.
     */
    void init(motor_type_t motor_type_, uint16_t initial_encoder_angle);

    /**
     * @brief   [ms]        Last time data updated. Could be used for check CAN network.
     */
    time_msecs_t last_update_time = SYSTIME;

    /**
     * @brief   [Degree/Sec]Actual output shaft velocity for motors.
     */
    float actual_velocity = 0.0f;

    /**
     * @brief   [Degree]    Actual motor angle vary from -180 to 180.
     */
    float actual_angle = 0.0f;

    /**
     * @brief   [rounds]    The number of rounds motor rotate.
     */
    int round_count = 0;

    /*===========================================================================*/
    /*                        Integrated Feedback Methods                        */
    /*===========================================================================*/

    /**
     * @brief               Get accumulate angle of the motor.
     * @return  [Degree]    Accumulate angle of the motor.
     */
    float accumulate_angle();

    /**
     * @brief               Get the torque constant of the motor.
     * @return  [Nm/A]      Torque constant of the motor.
     */
    float torque_const();

    /**
     * @brief               Get the output torque of the motor.
     * @return  [Nm]        Output torque of the motor.
     */
    float torque();

    /**
     * @brief               Get the torque current of the motor, from feedback
     * @return              Torque current in encoder unit.
     * @details             Range mapping: [-16383~16383] -> [-20A, 20A]
     */
    int torque_current();

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

    /**
     * @brief Motor's type. Motor supported: M3508, M3508 without reducer. M2006, GM6020.
     */
    motor_type_t motor_type = NONE_MOTOR;

    /**
     * @brief Angle from encoder, varies from (0 to 8191)
     */
    uint16_t rotor_angle_raw = 0;

    /**
     * @brief rpm from encoder.
     */
    int16_t rotor_rpm_raw = 0;

    /**
     * @brief Torque current from encoder.
     */
    int16_t torque_current_raw = 0;

    /**
     * @brief Temperature feedback from encoder (If support).
     */
    uint8_t temperature_raw = 0;

    /**
     * @brief Angle from last loop, to calculate the displacement.
     */
    uint16_t last_rotor_angle_raw = 0;

    /**
     * @brief Attached external reducer's reduce ratio.
     */
    float external_reduce_ratio = 1.0;

    /*===========================================================================*/
    /*                   Constant for processing the feedback                    */
    /*===========================================================================*/
private:

    static float constexpr M3508_TORQUE_CONST = 0.3;          // 0.3Nm/A  on the data sheet
    static float constexpr M3508_REDUCE_RATIO = 3591.0/187.0; // 3591/187 on the data sheet
    static float constexpr M2006_TORQUE_CONST = 1.0;          // 1.0 for feedback as the feedback is its torque.
    static float constexpr M2006_REDUCE_RATIO = 36.0;         // 36/1     on the data sheet
    static float constexpr ENCODER_ANGLE_RATIO = 360.0f/8192.0f;
    static float constexpr ENCODER_CURRENT_RATIO =  20.0/16384.0;//       on the data sheet

};


#endif //META_INFANTRY_CAN_MOTOR_FEEDBACK_H

/** @} */