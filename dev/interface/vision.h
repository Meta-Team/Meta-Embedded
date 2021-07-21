//
// Created by Kerui Zhu on 7/4/2019.
//

#ifndef META_INFANTRY_VISION_H
#define META_INFANTRY_VISION_H

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
    static void init(time_msecs_t basic_gimbal_delay, time_msecs_t basic_shoot_delay);

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
     * @note Should be called inside lock to prevent changes from the interrupt.
     * @param yaw    [Out] Gimbal target yaw if return true
     * @param pitch  [Out] Gimbal target pitch if return true
     * @return Whether the target can be reached
     */
    static bool get_gimbal_target_angles(float &yaw, float &pitch);

    /**
     * Get whether the bullet can reach the target.
     * @note Should be called inside lock to prevent changes from the interrupt.
     * @return
     */
    static bool can_reach_target() { return can_reach_the_target; }

    /**
     * Get the time point that we should shoot in the TopKiller mode
     * @note Should be called inside lock to prevent changes from the interrupt.
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

    static time_msecs_t get_last_update_time() { return last_update_time; }

    static void update_measured_pitch_offset(time_msecs_t offset) { measured_pitch_offset.update(offset); }

    static void update_measured_shoot_delay(time_msecs_t delay) { measured_shoot_delay.update(delay); }

public:

    /** Settings **/
    static float bullet_speed;                // [mm/ms = m/s]
    static time_msecs_t basic_gimbal_delay;   // [ms]
    static time_msecs_t basic_shoot_delay;    // [ms]

private:

    /** Armor and Control **/

    // Latest armor position (without prediction and compensation)
    static PositionKalmanFilter armor_ypd[3];    // [deg, deg, mm]
    static constexpr float Q_POSITION[3] = {1E-9, 1E-9, 1E-6};
    static constexpr float Q_VELOCITY[3] = {4E-4, 1E-5, 1};
    static constexpr float R_POSITION[3] = {0.3, 0.3, 200};

    // Prediction and compensation
    static bool can_reach_the_target;
    static float flight_time_to_target;  // [ms]
    static LowPassFilteredValue measured_pitch_offset;
    static LowPassFilteredValue measured_shoot_delay;

    static constexpr float MEASUREMENT_LPF_ALPHA = 0.9f;

    // Control commands
    static float latest_target_yaw;
    static float latest_target_pitch;
    static time_msecs_t expected_shoot_time;          // [ms] for TopKiller, 0 for anytime
    static int expected_shoot_after_periods;

    /** Updates **/

    // Gimbal angles at last vision command
    static float last_gimbal_yaw;    // [deg]
    static float last_gimbal_pitch;  // [deg]

    // Last vision command time
    static uint16_t last_frame_timestamp;   // [ms]
    static time_msecs_t last_update_time;   // [ms]
    static time_msecs_t last_update_delta;  // [ms]

private:
    enum vision_flag_t : uint8_t {
        NONE = 0,
        DETECTED = 1,
        TOP_KILLER_TRIGGERED = 2,
    };

    __PACKED_STRUCT vision_command_t {
        uint8_t flag;
        uint16_t time_stamp;               // [0.1ms]
        int16_t yaw_delta;                 // yaw relative angle [deg] * 100
        int16_t pitch_delta;               // pitch relative angle [deg] * 100
        int16_t dist;                      // [mm]
        int16_t remaining_time_to_target;  // [0.1ms]
        int16_t period;                    // [0.1ms]
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

    static void uart_rx_callback(UARTDriver *uartp);  // only for internal use
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


#endif //META_INFANTRY_VISION_H
