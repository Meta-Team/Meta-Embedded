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

#define FF_SWITCH_PAD GPIOB
#define FFL_SWITCH_PIN_ID GPIOB_PIN0    // PB0 - L2
#define FFR_SWITCH_PIN_ID GPIOB_PIN1    // PB1 - M2
#define SWITCH_TOUCH_PAL_STATUS PAL_HIGH

/** @note for the switch: BLACK -> GPIO; RED -> GND; BLUE -> VCC */

class EngineerElevatorLG {

public:

    static void init();

    enum action_t {
        FREE,           // free to control, for debug
        LOCK,           // not elevating
        UPWARD,         // going upstairs
        DOWNWARD,       // going down stairs
    };

    /**
     * @pre action == UPWARD || action == DOWNWARD
     * @brief Force the elevator to stop during going UPWARD or going DOWNWARD
     */
    static void forced_stop();

    /**
     * @pre action == STOP
     * @brief Quit prev_action by reversing.
     */
    static void quit_action();

    /**
     * @pre action == STOP
     * @brief Continue prev_action.
     */
    static void continue_action();

    /** @brief Start a whole auto process of going up-stairs */
    static void start_going_up();

    /** @brief Start a whole auto process of going down-stairs */
    static void start_going_down();

    /** @brief Set action as FREE, only for debugging separately */
    static void set_action_free();

    // both the two sensors in front detect the stage, can start to go up-stairs
    static bool reach_stage;
    // both the two sensors at the back wheels landed on stage, enter the last step of going up-stairs
    static bool back_landed;
    // both the two sensors at the back wheels reach the edge, can start to go down-stairs
    static bool back_edged;
    // both the two sensors at the front wheels leave the stage, enter the last step of going down-stairs
    static bool front_leave_stage;

    class EngineerElevatorLGThread: public chibios_rt::BaseStaticThread<512>{

        static constexpr unsigned ELEVATOR_LG_INTERVAL = 5;  // [ms]
        void main()final ;
    };

    static EngineerElevatorLGThread engineerLogicThread;

    // TODO
    // static bool ignore_DMS;


private:

    enum elevator_state_t{
        STOP,           // not using elevator
        PREPARING,      // preparing to elevate, move forward to reach the stage or backward to reach the edge
        ASCENDING,      // elevator standing up
        DESCENDING,     // elevator squatting down
        AIDING,         // aided motor moving
    };

    static action_t action;

    static action_t prev_action;    // keep record of the action before forced stop, used for reversing

    static elevator_state_t state;

    static uint16_t hanging_trigger;

    static uint16_t landed_trigger;

    /**
     * @brief light up the client light if the wheel is hanging
     * @note the client lights are arranged in this way: [FL BL BR FR]
     */
    static void update_hanging_status();

    /**
     * @brief the automatic movements of going up-stairs
     * @pre near the stage
     */
    static void going_up();

    /**
     * @brief the automatic movements of going down-stairs
     * @pre near the edge
     */
    static void going_down();

};


#endif //META_INFANTRY_ENGINEER_LOGIC_H
