//
// Created by liuzikai on 2019-07-15.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_LOGIC_H
#define META_INFANTRY_SENTRY_CHASSIS_LOGIC_H

#include "ch.hpp"
#include "common_macro.h"

class SChassisLG {

public:

    static void init(tprio_t direction_switch_thd_prio);

    enum mode_t {
        FORCED_RELAX_MODE,
        MANUAL_MODE,
        SHUTTLE_MODE,
        FINAL_AUTO_MODE
    };

    static void set_mode(mode_t value);

    static mode_t get_mode();

    static void set_manual_dest(float dest);
    static float get_manual_dest();

    static void set_shuttle_radius(float shuttle_radius);
    static float get_shuttle_radius();

    static void start_escaping();
    static void stop_escaping();
    static bool get_escaping_status();

private:

    static mode_t mode;

    static float manual_dest; // dest (for MANUAL_MODE)
    static float radius;  // range that sentry can move around the origin (for SHUTTLE_MODE)
    static bool random_mode;  // whether chassis should move randomly (for FINAL_AUTO_MODE)
    static time_msecs_t last_attack_time;  // record of the last time that an attack is detected [FINAL AUTO MODE ONLY]


    // Array containing terminals length information (for FINAL_AUTO_MODE)
    static constexpr float terminals[] = {10.0f, 100.0f, 190.0f, 280.0f, 370.0f, 450.0f};  // [cm]

    static int prev_terminal; // the terminal that chassis is leaving from
    static int next_terminal; // the terminal that chassis is approaching to

    static void update_next_terminal();  // helper function to choose next terminal (for FINAL_AUTO_MODE)

    // When difference between present_position and dest is smaller than this value, we grant that dest has been reached
    static constexpr float DEST_TOLERANT_RANGE = 3.0f;  // [cm]

    static constexpr int STOP_ESCAPING_AFTER = 60000;

    /// Direction Switch Thread for SHUTTLE_MODE and FINAL_AUTO_MODE
    class DirectionSwitchThread : public chibios_rt::BaseStaticThread<512> {
    public:

        bool started = false;

    private:

        // interval to evaluate whether switching dest is needed in SHUTTLE_MODE or FINAL_AUTO_MODE [ms]
        static constexpr unsigned DIRECTION_INSPECTION_INTERVAL = 5;

        void main() final;
    };

    static DirectionSwitchThread directionSwitchThread;
    static chibios_rt::ThreadReference directionThreadReference;



};


#endif //META_INFANTRY_SENTRY_CHASSIS_LOGIC_H
