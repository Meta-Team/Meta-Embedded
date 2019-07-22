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
#define FFR_SWITCH_PIN_ID GPIOB_PIN1    // PB1 - M2x
#define SWITCH_TOUCH_PAL_STATUS PAL_HIGH

/** @note for the switch: BLACK -> GPIO; RED -> GND; BLUE -> VCC */

class EngineerElevatorLG {

public:

    enum action_t {
        FREE,           // free to control, for debug
        LOCK,           // not elevating
        PAUSE,          // from pause_action
        UPWARD,         // going upstairs
        DOWNWARD,       // going down stairs
    };

    static void init(tprio_t logic_thread_prio);

    static action_t get_action() {      return action;      }
    static action_t get_prev_action() { return prev_action; }

    /** @brief Set action as FREE, only for debugging each part of the process separately.
     *  @note Can only be set free when LOCK OR PAUSE */
    static void set_action_free();

    /** @brief LOCK anyway, for safe mode.
     *  @note need to reset manually by setting FREE. */
    static void set_action_lock();

    /** @brief Start an auto process of going up-stairs */
    static void start_going_up();

    /** @brief Start an auto process of going down-stairs */
    static void start_going_down();


    /**
     * @pre action == UPWARD || action == DOWNWARD || action == FREE
     * @brief Force the elevator to stop during going UPWARD or going DOWNWARD
     */
    static void pause_action();

    /** @brief Continue prev_action from PAUSE. */
    static void continue_action();

    /** @brief Quit prev_action by reversing from PAUSE. */
    static void quit_action();



    // both the two sensors in front detect the stage, can start to go up-stairs
    static bool reach_stage;
    // both the two sensors at the back wheels landed on stage, enter the last step of going up-stairs
    static bool back_landed;
    // both the two sensors at the back wheels reach the edge, can start to go down-stairs
    static bool back_edged;
    // both the two sensors at the front wheels leave the stage, enter the last step of going down-stairs
    static bool front_leave_stage;

private:

    class EngineerElevatorLGThread: public chibios_rt::BaseStaticThread<512>{

        static constexpr unsigned ELEVATOR_LG_INTERVAL = 5;  // [ms]
        void main()final ;
    };

    static EngineerElevatorLGThread engineerLogicThread;

    // TODO
    // static bool ignore_DMS;


    enum elevator_state_t{
        STOP,           // not using elevator
        PREPARING,      // preparing to elevate, move forward to reach the stage or backward to reach the edge
        ASCENDING,      // elevator standing up
        DESCENDING,     // elevator squatting down
        AIDING,         // aided motor moving
    };

    static action_t action;

    static action_t prev_action;    // keep record of the action before forced stop, used for cont or quit

    static elevator_state_t state;

    static uint16_t hanging_trigger;
    static uint16_t landed_trigger;

    // for debugging, support pause and resume
    static float prev_e_tg_h;
    static float prev_aR_tg_v;
    static float prev_aL_tg_v;

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
