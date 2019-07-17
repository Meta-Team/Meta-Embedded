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


class EngineerElevatorLG {

public:

    static void init();

    enum action_t {
        FREE,           // free to control, for debug
        LOCK,           // not elevating
        UPWARD,         // going upstairs
        DOWNWARD,       // going down stairs
    };

    static void set_action(action_t act);

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

    static elevator_state_t state;

    static float reach_stage_trigger;

    static float hanging_trigger;   //TODO need an interval??

    /**
     * @brief check the hanging status of each wheels, and light up the corresponding client light
     * @note the client lights are arranged in this way: [FL BL BR FR]
     */
    static void update_hanging_status();

    /**
     * @brief the automatic movements of going up-stairs
     * @pre the aided motors should be on the stage
     */
    static void going_up();

    /**
     * @brief the automatic movements of going down-stairs
     * @pre the aided wheels should be out of the stage
     */
    static void going_down();

};


#endif //META_INFANTRY_ENGINEER_LOGIC_H
