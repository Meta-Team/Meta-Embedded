//
// Created by Chen Qian on 10/29/21.
//

#ifndef META_INFANTRY_CAN_MOTOR_INTERFACE_H
#define META_INFANTRY_CAN_MOTOR_INTERFACE_H

#include "can_interface.h"
#include "can_motor_feedback.h"

#include "CANBUS_MOTOR_CFG.h"

/**
 * @author Chen Qian
 * @brief  Universal Interface for CAN motors.
 * @usage
 * @code
 * 1.   Create and include motor CANMotorCFG class.
 * 2.   Initialized the class by calling init() method.
 * 3.a. Set current to the motor on certain CAN BUS.
 * 3.b. Also can read feedback by accessing motor_feedback.
 * 3.c. After set current, publish the data on CAN-BUS by post_target_current() method.
 * *Notice: Either set motor current or get access to motor feedback should use the
 *          logical index (YAW, PITCH).
 * @endcode
 * @details 1. CANMotorCFG class structure:
 * @code
 * class CANMotorCFG{
 *     enum             motor_id_t {...,..., MOTOR_COUNT};
 *     CANMotorBase     CANMotorProfile[MOTOR_COUNT];}
 * @endcode
 *
 * @details 2. Single CAN motor profile structure:
 * @code
 * CANMotorBase {[CANMotorProfile::can_channel_t] can_channel,
 *               [int] CAN_SID,
 *               [CANMotorBase::motor_type_t] motor_type,
 *               [int] initial_encoder_angle}
 * @endcode
 * */

class CANMotorInterface : public CANMotorCFG {
    /********************************************//**
     *        Public Contribution and Methods
     ***********************************************/
public:

    /**
     * @brief Initialize the haptic arm interface.
     * @param[in] can1_             The can1 channel to use.
     * @param[in] can2_             The can2 channel to use.
     * */
    static void init(CANInterface *can1_, CANInterface *can2_);

    /**
     * @brief Motors that will be liked with logical motor id.
     * */
    static CANMotorFeedback motor_feedback[MOTOR_COUNT];

    /**
     * @brief Set can motor currents.
     * @param motor_id[in]      (YAW/PITCH)     Logical motor ID, use the enumerator.
     * @param target_current[in]                Input variable of motor. Varies from motor.
     * @details Input variables, range and mapping relation for each motor:
     * @code
     * Motor Type    |    GM6020     |     M3508     |         M2006
     * -------------------------------------------------------------
     * Input Vars    |    Voltage    |    Current    |       Current
     * -------------------------------------------------------------
     * Input Range   | -30000~30000  | -16384~16384  |  -10000~10000
     * -------------------------------------------------------------
     * Output Range  | Not Provided  |   -20A~20A    |      -10A~10A
     * */
    static void set_current(motor_id_t motor_id, int target_current);

    /**
     * @brief The function send the stored txmsg.
     * @param can_channel_ [in]  [can_channel_1/can_channel_2]   The can channel to post.
     * @param SID          [in]  [0x200/0x1FF/0x2FF]             The identifier of the CAN Tx Frame.
     * */
     static bool post_target_current(CANMotorBase::can_channel_t can_channel_, uint32_t SID);

     /**
      * @brief CAN Tx Frame enable flag
      * @details Indicate whether the can frame was used.
      * @code
      *   CAN1 | 0x200 | 0x1FF | 0x2FF
      *   ----------------------------
      *   CAN2 | 0x200 | 0x1FF | 0x2FF
      * @endcode
      * */
     static bool enable_CAN_tx_frames[2][3];
    /********************************************//**
     *             SID and ID mapping
     ***********************************************/
private:

    /**
     * @brief SID and CAN Channel list for check feedback.
     * */
    static CANMotorBase can_motor_profile[MOTOR_COUNT];

    /**
     * @brief Mapping from motor SID(0x20*) to motor id (YAW, PITCH, ...).
     * */
    static motor_id_t mapping_SID2ID[2][11];

    /**
     * @brief Mapping from logical motor(YAW, PITCH, ...) id to motor SID(0x20*).
     * */
    static struct mapping_ID2SID_t {
        int SID;
        CANMotorBase::can_channel_t can_channel;
    } mapping_ID2SID[MOTOR_COUNT];

    /********************************************//**
     *               CAN Mechanism
     ***********************************************/
private:

    /**
     * @brief A 2x3 array stored txmsgs to send. Structure:\n
     *
     * @code
     * CAN1  |  0x200  |  0x1FF  |  0x2FF
     *       ----------------------------
     * CAN2  |  0x200  |  0x1FF  |  0x2FF
     * @endcode
     * */
    static CANTxFrame txmsg[2][3];

    /**
     * @brief Overall can1 callback function.
     * @param rxmsg CAN Rx Frame.
     * */
    static void can1_callback_func(CANRxFrame const *rxmsg);

    /**
     * @brief Overall can2 callback function.
     * @param rxmsg CAN Rx Frame.
     * */
    static void can2_callback_func(CANRxFrame const *rxmsg);

    /**
     * @brief CANInterfaces to use [CAN1, CAN2].
     * */
    static CANInterface *can[2];
};

#endif //META_INFANTRY_CAN_MOTOR_INTERFACE_H
