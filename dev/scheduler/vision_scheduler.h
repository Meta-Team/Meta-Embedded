//
// Created by liuzikai on 7/29/21.
//

#ifndef META_INFANTRY_VISION_SCHEDULER_H
#define META_INFANTRY_VISION_SCHEDULER_H

#include "low_pass_filter.hpp"
#include "position_kalman_filter.hpp"
#include "shell.h"

class Vision {
public:

    static void start(time_msecs_t basic_gimbal_delay, tprio_t thread_prio);

    /**
     * Set bullet speed.
     * @param speed [m/s]
     */
    static void set_bullet_speed(float speed) { bullet_speed = speed; }

    /**
     * Get bullet speed setting.
     * @return [m/s]
     */
    static float get_bullet_speed() { return bullet_speed; }

    /**
     * Get whether an enemy is detected.
     * @return
     */
    static bool is_detected();

    /**
     * Get gimbal target angles.
     * @param yaw    [Out] Gimbal target yaw if return true
     * @param pitch  [Out] Gimbal target pitch if return true
     * @return Whether the target can be reached
     */
    static bool get_gimbal_target_angles(float &yaw, float &pitch);

    /**
     * Get the time point that we should shoot in the TopKiller mode
     * @return
     */
    static time_msecs_t get_expected_shoot_time() { return expected_shoot_time; }

    /**
     * Received updated gimbal target.
     */
    static event_source_t gimbal_updated_event;

    /**
     * Received expected shoot time in the TopKiller mode.
     */
    static event_source_t shoot_time_updated_event;

    /**
     * Get last compute time.
     * @return
     */
    static time_msecs_t get_last_compute_time() { return last_compute_time; }

    /**
     * Get enemy position on user view;
     * @param x
     * @param y
     */
    static void get_user_view_points(uint32_t &x, uint32_t &y);

    static void cmd_feedback(void *);
    static const Shell::Command shell_commands[];

private:

    /** Settings **/
    static float bullet_speed;                // [mm/ms = m/s]
    static time_msecs_t basic_gimbal_delay;   // [ms]

    /** Armor and Control **/

    // Latest armor position (without prediction and compensation)
    static PositionKalmanFilter armor_ypd[3];    // [deg, deg, mm] and velocity (divided by ms)
    static constexpr float ARMOR_Q_POSITION[3] = {1E-2, 1E-2, 1E-6};
    static constexpr float ARMOR_Q_VELOCITY[3] = {1E-4, 1E-6, 1E-3};
    static constexpr float ARMOR_R_POSITION[3] = {0.09, 0.16, 1.6E5};

    // Prediction and compensation
    static bool can_reach_target;
    static float bullet_flight_time;  // [ms]

    // Control commands
    static float latest_target_yaw;
    static float latest_target_pitch;

    /** TopKiller **/

    static PositionKalmanFilter accumulated_rotation;   // [deg] and speed (divided by ms)
    static constexpr float ROTATION_Q_POSITION = 0.09;  // covariance of angle uncertainty [deg^2]
    static constexpr float ROTATION_Q_VELOCITY = 0.06;  // covariance of velocity uncertainty [deg^2/ms^2]
    static constexpr float ROTATION_R_POSITION = 9;     // covariance of measured angle [deg^2]

    static float last_instant_rotation;
    static int rotation_accumulated_cycle;

    // Conditions to enter TopKiller state
    static constexpr float TK_ROTATION_VELOCITY_THRESHOLD = 0.18;   // [deg/ms]
    static constexpr float TK_ROTATION_HOLD_TIME_THRESHOLD = 2000;  // [ms]
    static time_msecs_t tk_rotation_hold_start_time;


    static time_msecs_t expected_shoot_time;          // [ms] for TopKiller, 0 for anytime
    static int expected_shoot_after_periods;

    /** Updates **/

    // Last vision command time
    static uint16_t last_frame_timestamp;        // [ms]
    static time_msecs_t last_detected_time;
    static time_msecs_t last_compute_time;
    static time_msecs_t last_vision_command_time;

    static constexpr time_msecs_t DETECTION_LOSE_TIME = 500;  // [ms]
    static constexpr time_msecs_t POSITION_RELOAD_TIME = 1000;  // [ms]
    static constexpr float SINGLE_ARMOR_2D_OFFSET_PER_FRAME = 500;  // [mm]

    /** User View **/

    static float IMAGE_TO_USER_SCALE;
    static int IMAGE_TO_USER_OFFSET_X;
    static int IMAGE_TO_USER_OFFSET_Y;
    static uint32_t user_view_x;
    static uint32_t user_view_y;

    class CalculationThread : public chibios_rt::BaseStaticThread<1024> {
        event_listener command_listener;
        void main() override;
    };

    static CalculationThread calculation_thread;

    static DECL_SHELL_CMD(cmdInfo);
    static DECL_SHELL_CMD(cmdEnableFeedback);
    static DECL_SHELL_CMD(cmd_user_view_setting);

};


#endif //META_INFANTRY_VISION_SCHEDULER_H
