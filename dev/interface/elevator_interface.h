//
// Created by Zhu Kerui on 2019/1/16.
//

#ifndef META_INFANTRY_RMDS108_INTERFACE_H
#define META_INFANTRY_RMDS108_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

#define ELEVATOR_INTERFACE_SAFETY_BUTTON_PAD GPIOE

#if defined(BOARD_RM_2018_A)
#else
#error "ElevatorInterface is only developed for RM board 2018 A."
#endif

/**
 * @name ElevatorInterface
 * @brief an interface to control elevator motor height and handle feedback
 * @pre hardware is properly configured. CAN IDs of RMDS should be the same as wheel_can_id_t below.
 * @usage 1. init() with properly intialized CANInterface
 *        2. apply_front/read_position(), and read feedback from elevator_wheels[]
 */
class ElevatorInterface {
public:

    enum wheel_t {
        FRONT_RIGHT,
        FRONT_LEFT,
        REAR_LEFT,
        REAR_RIGHT,
        WHEEL_COUNT // = 4
    };

    /**
     * @brief set the target position for the front wheels
     * @param front_wheel_position_cm the target position of the front wheels, - for downward
     * @return whether the position is valid 
     * @note we grant that the startup position is the lowest point, so this function DISALLOW positive position
     */
    static bool apply_front_position(float front_wheel_position_cm);

    /**
     * @brief set the target position for the rear wheels
     * @param rear_wheel_position_cm the target position of the rear wheels, - for downward
     * @return whether the position is valid 
     * @note we grant that the startup position is the lowest point, so this function DISALLOW positive position
     */
    static bool apply_rear_position(float rear_wheel_position_cm);


    /** Unit Interface for each RMDS module **/
    class UnitInterface {
    public:
        int32_t real_position; // the real position of the motor, [qc]
        int16_t real_current;  // the real current in the motor
        int16_t real_velocity;  // the real velocity of the motor

        /**
         * @brief return whether the elevator unit is in action
         * @return
         * @note if the abs distance between real_position and target_position is greater than stable_range (configured
         *       below, the unit is considered to be in action
         */
        bool is_in_action();

        bool get_safety_button_status();

        UnitInterface() : UnitInterface(0xFF) {};

        UnitInterface(unsigned int safety_button_pin_id) : safety_button_pin(safety_button_pin_id) {};

    private:
        unsigned int safety_button_pin;
        bool is_actioning_;

        friend ElevatorInterface;
    };

    /**
     * @brief contains the real information of the four wheels
     * elevator_wheels[0] and elevator_wheels[1] are for the front_left and front_right wheels accordingly
     * elevator_wheels[2] and elevator_wheels[3] are for the left_rear and right_rear wheels accordingly
     */
    static UnitInterface wheels[4];

    /**
     * @brief set the CAN interface
     * @param can_interface
     */
    static void init(CANInterface *can_interface);

    /**
     *
     */
    static float sensor_height[4];

private:

    /**
     * @brief contains the target position of both the front wheels and the rear wheels
     * target_position[0] contains the target position of the front wheels
     * target_position[1] contains the target position of the rear wheels
     */
    static int32_t target_position_[2]; // [pc]


    enum wheel_can_id_t {
        FRONT_RIGHT_CAN_ID = 1,
        FRONT_LEFT_CAN_ID = 2,
        REAR_LEFT_CAN_ID = 3,
        REAR_RIGHT_CAN_ID = 4
    };

    /**
     * @brief send message of each motor
     * @return true if success, false otherwise
     */
    static bool send_target_position_(int wheel_index);

    /**
     * @brief Get the feedback (real position, real velocity, real current) of each motor from the driver
     * @return true if success, false otherwise
     */
    static void process_feedback_(CANRxFrame const *rxmsg);

    static CANInterface *can_;

    static void adc_callback_(ADCDriver *adcp, adcsample_t *buffer, size_t n);
    static void adc_error_callback_(ADCDriver *adcp, adcerror_t err);

    static constexpr unsigned int RMDS_CAN_GROUP_ID = 3;
    static constexpr uint8_t RMDS_FEEDBACK_INTERVAL = 100;
    static constexpr uint16_t RMDS_DRIVER_PWM = 2500;  // the pwm of the current
    static constexpr int RMDS_STABLE_RANGE = 1000; // the range that is regarded as target has been reached. [qc], 0.4 cm

    static constexpr ADCConversionGroup ADC_CONFIG = {
            TRUE,
            WHEEL_COUNT,
            adc_callback_,
            adc_error_callback_,
            0,                        /* CR1 */
            ADC_CR2_SWSTART,          /* CR2 */
            ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) |
            ADC_SMPR1_SMP_AN13(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN14(ADC_SAMPLE_3),
            0,                        /* SMPR2 */
            0,                        /* SQR1 */
            0,                        /* SQR2 */
            ADC_SQR3_SQ4_N(ADC_CHANNEL_IN14)   | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN13) |
            ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12)   | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)
    };

    static adcsample_t adc_sample_[WHEEL_COUNT];

};

#endif //META_INFANTRY_RMDS108_INTERFACE_H
