//
// Created by Kerui Zhu on 7/4/2019.
//

#ifndef META_INFANTRY_VISION_INTERFACE_H
#define META_INFANTRY_VISION_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "low_pass_filter.hpp"
#include "position_kalman_filter.hpp"

class Vision {

public:

    /**
     * Setup Vision module.
     * @param basic_gimbal_delay     Fixed gimbal control delay [ms]
     * @param basic_shoot_delay      Fixed shooter control delay [ms]
     */
    static void init(time_msecs_t basic_gimbal_delay);

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
     * Get gimbal target angles.
     * @note Should be called inside S-Lock to prevent changes from the interrupt.
     * @param yaw    [Out] Gimbal target yaw if return true
     * @param pitch  [Out] Gimbal target pitch if return true
     * @return Whether the target can be reached
     */
    static bool get_gimbal_target_angles_S(float &yaw, float &pitch);

    /**
     * Get the time point that we should shoot in the TopKiller mode
     * @note Should be called inside S-Lock to prevent changes from the interrupt.
     * @return
     */
    static time_msecs_t get_expected_shoot_time_S() { return expected_shoot_time; }

    /**
     * Received updated gimbal target.
     */
    static event_source_t gimbal_updated_event;

    /**
     * Received expected shoot time in the TopKiller mode.
     */
    static event_source_t shoot_time_updated_event;

    /**
     * Get last control command update time.
     * @note Should be called inside S-Lock to prevent changes from the interrupt.
     * @return
     */
    static time_msecs_t get_last_update_time_S() { return last_update_time; }

public:

    /** Settings **/
    static float bullet_speed;                // [mm/ms = m/s]
    static time_msecs_t basic_gimbal_delay;   // [ms]

private:

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

    // Gimbal angles at last vision command
    static float last_gimbal_yaw;    // [deg]
    static float last_gimbal_pitch;  // [deg]

    // Last vision command time
    static uint16_t last_frame_timestamp;        // [ms]
    static time_msecs_t last_update_time;        // [ms]
    static time_msecs_t last_update_time_delta;  // [ms]

private:
    enum vision_flag_t : uint8_t {
        NONE = 0,
        DETECTED = 1,
    };

    __PACKED_STRUCT vision_command_t {
        uint8_t flag;
        uint16_t time_stamp;               // [0.1ms]
        int16_t yaw_delta;                 // yaw relative angle [deg] * 100
        int16_t pitch_delta;               // pitch relative angle [deg] * 100
        int16_t dist;                      // [mm]
        int16_t avg_light_angle;           // [deg] * 100
    };

    __PACKED_STRUCT package_t {
        uint8_t sof;  // start of frame, 0xA5
        uint8_t cmd_id;
        union {  // union takes the maximal size of its elements
            vision_command_t command;
        };
        uint8_t crc8;  // just for reference but not use (the offset of this is not correct)
    };

    enum cmd_id_t : uint8_t {
        VISION_CONTROL_CMD_ID = 0,
        CMD_ID_COUNT
    };

    static constexpr size_t DATA_SIZE[CMD_ID_COUNT] = {
            sizeof(vision_command_t)
    };

    static package_t pak;

    static void uart_rx_callback(UARTDriver *uartp);
    static void uart_err_callback(UARTDriver *uartp, uartflags_t e);
    static void uart_char_callback(UARTDriver *uartp, uint16_t c);

    enum rx_status_t {
        WAIT_STARTING_BYTE,  // receive bytes one by one, waiting for 0xA5
        WAIT_CMD_ID,         // receive cmd_id
        WAIT_DATA_TAIL       // receive data section and 1-byte CRC8 data
    };

    static rx_status_t rx_status;

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

    // See cpp file for configs
    static constexpr UARTDriver *UART_DRIVER = &UARTD8;
    static const UARTConfig UART_CONFIG;

private:

    static void handle_vision_command(const vision_command_t &command);

    friend class FeedbackThread;
};


#endif //META_INFANTRY_VISION_INTERFACE_H
