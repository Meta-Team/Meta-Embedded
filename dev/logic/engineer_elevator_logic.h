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
#include "referee_interface.h"
#include "vehicle_engineer.h"

// TODO: embed as constants
#define FF_SWITCH_PAD GPIOB
#define FFL_SWITCH_PIN_ID GPIOB_PIN0    // PB0 - L2
#define FFR_SWITCH_PIN_ID GPIOB_PIN1    // PB1 - M2
#define SWITCH_TOUCH_PAL_STATUS PAL_HIGH
#define FR_SENSOR GPIOC_PIN2
#define FL_SENSOR GPIOC_PIN3
#define BL_SENSOR GPIOC_PIN4
#define BR_SENSOR GPIOC_PIN5


/** @note for the switch: BLACK -> GPIO; RED -> GND; BLUE -> VCC */

class EngineerElevatorLG {

public:

    enum elevator_state_t{
        STOP,           // not using elevator
        PREPARING,      // preparing to elevate, move forward to reach the stage or backward to reach the edge
        ASCENDING,      // elevator standing up
        AIDING,         // aided motor moving
        DESCENDING,     // elevator squatting down
        GIVING_BULLET
    };

    static elevator_state_t get_current_state();

    static void init(tprio_t logic_thread_prio);

    static void elevator_enable(bool enable_);

    static void set_test_mode(bool test_mode_);

    /** @brief Set target height */
    static void set_elevator_height(float new_height);

    /** @brief Get current height */
    static float get_elevator_height();

    /** @brief Set target aided_motor velocity */
    static void set_aided_motor_velocity(float target_velocity);

    /** @brief Switch the process to going down-stairs state */
    static void set_elevate_dir(bool going_up_);

    static void give_bullet();

    static void change_auto_status();

private:

    static void set_state(elevator_state_t new_state);

    static bool test_mode;

    class EngineerElevatorLGThread: public chibios_rt::BaseStaticThread<2048>{

        static constexpr unsigned ELEVATOR_LG_INTERVAL = 2;  // [ms]
        void main()final ;
    };

    static EngineerElevatorLGThread engineerLogicThread;

    static elevator_state_t state;

    static bool going_up;

    static bool auto_elevator;

    static constexpr uint16_t hanging_trigger = 1800;//TODO need to determine
    static constexpr uint16_t landed_trigger = 2300;//TODO need to determine
};


#endif //META_INFANTRY_ENGINEER_LOGIC_H
