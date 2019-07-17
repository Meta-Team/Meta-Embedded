//
// Created by Kerui Zhu on 7/15/2019.
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
        FREE,           // not elevating
        UPWARD,         // going upstairs
        DOWNWARD,       // going down stairs
    };

    static void set_action(action_t act);



    class EngineerElevatorLGThread: public chibios_rt::BaseStaticThread<512>{

        static constexpr unsigned ELEVATOR_LG_INTERVAL = 5;  // [ms]
        void main()final ;
    };

    static EngineerElevatorLGThread engineerLogicThread;

    // static bool ignore_DMS;


private:

    enum elevator_state_t{
        STOP,           // not using elevator
        ASCENDING,      // elevator standing up
        DESCENDING,     // elevator squatting down
        AIDING,         // aided motor moving
    };

    static action_t action;

    static elevator_state_t state;

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
