//
// Created by LaiXinyi on 7/15/2019.
//

#ifndef META_INFANTRY_ENGINEER_LOGIC_H
#define META_INFANTRY_ENGINEER_LOGIC_H

#include "ch.hpp"
#include "hal.h"
#include "engineer_elevator_skd.h"
#include "engineer_elevator_interface.h"
#include "engineer_chassis_skd.h"
#include "dms_interface.h"
#include "referee_interface.h"

// TODO: embed as constants
#define FF_SWITCH_PAD GPIOB
#define FFL_SWITCH_PIN_ID GPIOB_PIN0    // PB0 - L2
#define FFR_SWITCH_PIN_ID GPIOB_PIN1    // PB1 - M2
#define SWITCH_TOUCH_PAL_STATUS PAL_HIGH

/** @note for the switch: BLACK -> GPIO; RED -> GND; BLUE -> VCC */

class EngineerElevatorLG {

public:

    enum elevator_state_t{
        STOP,           // not using elevator
        PREPARING,      // preparing to elevate, move forward to reach the stage or backward to reach the edge
        ASCENDING,      // elevator standing up
        DESCENDING,     // elevator squatting down
        AIDING,         // aided motor moving
        GIVING_BULLET
    };

    static void init(tprio_t logic_thread_prio);

    static void elevator_enable(bool enable_);

    static void set_test_mode(bool test_mode_);

    /** @brief Set target height */
    static void set_elevator_height(float new_height);

    /** @brief Get current height */
    static float get_elevator_height();

    /** @brief Set target aided_motor velocity */
    static void set_aided_motor_velocity(float target_velocity);

    /** @brief Set auto elevating mode or user controlling step-by-step mode */
    static void set_auto_elevating(bool auto_elevating_);

    /** @brief Switch the process to going down-stairs state */
    static void set_elevate_dir(bool going_up_);

    /** @brief Force the elevator to move into the next step */
    static void next_step();

    static void give_bullet();
/*
    ///////// for finding better aided motor params
    static void aided_motor_test_forward();
    static void aided_motor_test_backward();
    static bool a_t_forward;
    static bool a_t_backward;

    static uint32_t delay_time;     // ms
*/
private:

    static bool test_mode;

    class EngineerElevatorLGThread: public chibios_rt::BaseStaticThread<512>{

        static constexpr unsigned ELEVATOR_LG_INTERVAL = 2;  // [ms]
        void main()final ;
    };

    static EngineerElevatorLGThread engineerLogicThread;

    static elevator_state_t state;

    static bool pause;

    static bool going_up;


    static bool auto_elevating;

    static constexpr uint16_t hanging_trigger = 1800;//TODO need to determine
    static constexpr uint16_t landed_trigger = 2300;//TODO need to determine

    /**
 * @brief Force the elevator to stop
 */
    static void pause_action();

    /** @brief Continue to move. */
    static void continue_action();

};


#endif //META_INFANTRY_ENGINEER_LOGIC_H
