//
// Created by liuzikai on 2019-07-15.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_LOGIC_H
#define META_INFANTRY_SENTRY_CHASSIS_LOGIC_H


class SChassisLG {

    enum action_t {
        FORCED_RELAX_MODE,
        ONE_STEP_MODE,
        SHUTTLED_MODE,
        FINAL_AUTO_MODE
    };

    /**
     * @brief start escaping, start the randomMode, change terminal immediately if it is not escaping now
     * @pre Enemies are spotted or the sentry is being attacked
     */
    static void start_escaping();

    /**
     * @brief update the next terminal according to the randomMode (true/false)
     */
    static void update_terminal();

    /**
     * @pre stop escaping and start cruising
     */
    static void stop_escaping();

    // Whether chassis should move randomly [FINAL AUTO MODE ONLY]
    static bool randomMode;

    /** PIDs **/



    /** Accessory Parameters **/

    // The range that sentry can move around the origin [SHUTTLED MODE ONLY]
    static float radius;

    // Array containing terminals length information [FINAL AUTO MODE ONLY]
    static float terminals[];

    // The index of terminal in the terminals array [FINAL AUTO MODE ONLY]
    static int prev_terminal; // The terminal that chassis is leaving from
    static int next_terminal; // The terminal that chassis is approaching to

    // Record of the last time that an attack is detected [FINAL AUTO MODE ONLY]
    static unsigned last_attack_time;

    /**
     * @brief use the present data and PIDController to calculate and set the target current that will be sent
     */
    static void update_target_current();

};


#endif //META_INFANTRY_SENTRY_CHASSIS_LOGIC_H
