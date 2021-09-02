//
// Created by liuzikai on 2018-12-29.
//

/**
 * @file    can_interface.h
 * @brief   CAN interface to receive, distribute and send message.
 *
 * @addtogroup CANInterface
 * @{
 */

#ifndef META_INFANTRY_CAN_INTERFACE_H
#define META_INFANTRY_CAN_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#if defined(BOARD_RM_2018_A)
// CAN1_RX - PD0, CAN1_TX - PD1
#elif defined(BOARD_RM_2017)
// CAN1_RX - PD0, CAN1_TX - PD1
#else
#error "CANInterface has not been defined for selected board"
#endif

#define CAN_INTERFACE_ENABLE_X2FF_FRAME              FALSE
#define CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD   TRUE
#define CAN_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL   FALSE

/**
 * CAN interface to receive, distribute and send message
 * @pre CAN pins are configured properly in board.h
 * @usage 1. Create an instance with specific CAN driver
 *        2. Call start() to start the CAN driver and receive thread
 *        3. Set Motor types
 *        4. Invoke set_target_current() to set current of certain motor, the current will be send automatically.
 */
class CANInterface {
public:

    /**
     * Initialize a can interface
     * @param driver   Pointer to CAN driver such as &CAND1
     */
    CANInterface(CANDriver *driver) :
            can_driver(driver)
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)
            , errorFeedbackThread(driver, last_error_time)
#endif
            , txThread(driver), rxThread(driver)
    {}

    /**
     * Start the CAN driver and the receiving thread
     * @param rx_thread_prio   Thread priority of the receiving thread
     * @return A reference to the created thread
     */
    void start(tprio_t rx_thread_prio, tprio_t tx_thread_prio);

    static constexpr unsigned MAXIMUM_MOTOR_COUNT = 11;

    enum motor_type_t {
        NONE_MOTOR,
        RM6623,
        M2006,
        GM6020,
        GM3510,
        M3508
    };

    static constexpr float CAN_INTERFACE_RAW_TO_ANGLE_RATIO = 0.043945312f; // 360 / 8192
    static constexpr float CAN_INTERFACE_RPM_TO_DPS = 6.f; // 360 / 60, round per minute to degree per second
    static constexpr float DEFAULT_DECELERATION_RATIO = 1.f;
    static constexpr float M2006_WITH_DECELERATION_RATIO = 36.f;
    static constexpr float M3508_WITH_DECELERATION_RATIO = 19.203208556f; // 3591 / 187

    struct motor_feedback_t {
    public:

        motor_type_t type;

        uint32_t sid;

        float deceleration_ratio = 1.f;

        int32_t accumulated_movement_raw = 0;

        /**
         * Normalized angle
         * @note Viewing from TOP of 6623/2006 motor. 180 <--CCW-- front_angle_raw --CW--> -180
         */
        float actual_angle = 0.0f;     // [degree]

        /**
         * Velocity
         * @note Viewing from TOP of 6623/2006 motor. Positive for CCW. Negative for CW.
         */
        float actual_velocity = 0.0f;  // [degree/s]

        /**
         * Actual current
         * @note Direction is UNKNOWN yet. In reality, it vibrates significantly, and it's not useful for now.
         */
        int actual_current = 0;  // [mA]

        /**
         * Last update time (ms, from system start)
         */
        time_msecs_t last_update_time = 0;

        /**
         * Set current actual angle as the zero reference angle and clear the round count (accumulated angle = 0)
         */
        void reset_front_angle();

        /**
         * Get total angle from the original front angle
         * @return Accumulated angle since last reset_front_angle [degree, positive for CCW viewing from top]
         */
        float accumulated_angle();

        uint16_t last_angle_raw = 0;  // in the range of [0, 8191]

#if CAN_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL
        // Variables for velocity sampling
        time_msecs_t sample_time_stamp = 0;
        int sample_count = 0;
        int sample_movement_sum = 0;
#endif

    };

    struct cap_feedback_t {
        float input_voltage;      // [V]
        float capacitor_voltage;  // [V]
        float input_current;      // [A]
        float output_power;       // [W]
    };

    /**
     * Send a frame
     * @param txmsg   The frame to be sent to super capacitor
     * @return Whether the message has been sent successfully
     */
    bool send_cap_msg(const CANTxFrame *txmsg);

    /**
     * Get target current address
     * @param id    The motor id.
     */
    int *register_target_current_address(unsigned id, motor_type_t motor_type);

    /**
     * Event source to broadcast CAN error message
     */
    EVENTSOURCE_DECL(error_event_src);

    /**
     * Get the address of a certain feedback.
     * @param id  The motor id.
     */
    motor_feedback_t *register_feedback_address(unsigned id, motor_type_t motor_type, float deceleration_ratio);

    /**
     * Get the address of super capacitor feedback.
     */
    cap_feedback_t *get_cap_feedback_address();

    /**
     * Get the address of lidar feedback.
     */
    float *get_lidar_feedback_address();
private:

    CANDriver *can_driver;

    static constexpr unsigned MAXIMUM_REGISTRATION_COUNT = 20;

    static constexpr unsigned int SEND_THREAD_INTERVAL = 1;  // can message send interval [ms]

#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)

public:

    time_msecs_t last_error_time = 0;

private:

    class ErrorFeedbackThread : public chibios_rt::BaseStaticThread<512> {
    public:
        ErrorFeedbackThread(CANDriver *can_driver_, time_msecs_t &last_error_time_) :
                can_driver(can_driver_), last_error_time(last_error_time_) {}
        CANDriver *can_driver;
    private:
        time_msecs_t &last_error_time;

        void main() final;
    };

    ErrorFeedbackThread errorFeedbackThread;

#endif

    class TxThread : public chibios_rt::BaseStaticThread<512> {
    public:
        explicit TxThread(CANDriver *can_driver_) :
                can_driver(can_driver_) {}
        CANDriver *can_driver;
        int x1ff_target_current[4 + 1];
        motor_type_t x1ff_motorType[4 + 1];
        int x200_target_current[4 + 1];
        motor_type_t x200_motorType[4 + 1];
#if CAN_INTERFACE_ENABLE_X2FF_FRAME
        int x2ff_target_current[4 + 1];
        motor_type_t x2ff_motorType[4 + 1];
#endif
        void cap_send(const CANTxFrame *txmsg);
    private:
        bool send_msg(const CANTxFrame *txmsg);

        void main() final;

        const CANTxFrame *capMsg;
        bool capMsgSent = true;
    };

    TxThread txThread;

    class RxThread : public chibios_rt::BaseStaticThread<1024> {
    public:
        explicit RxThread(CANDriver *can_driver_) :
        can_driver(can_driver_) {}
        CANDriver *can_driver;

        motor_feedback_t feedback[MAXIMUM_MOTOR_COUNT + 1];
        cap_feedback_t capfeedback;
        float lidar_dist;
    private:
        void main() final;
    };

    RxThread rxThread;

    CANConfig can_cfg = {
            CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
            CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
            CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
    };
    static constexpr unsigned TRANSMIT_TIMEOUT_MS = 10;

};


#endif //META_INFANTRY_CAN_INTERFACE_H
