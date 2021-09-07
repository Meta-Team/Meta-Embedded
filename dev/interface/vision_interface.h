//
// Created by Kerui Zhu on 7/4/2019.
//

#ifndef META_INFANTRY_VISION_INTERFACE_H
#define META_INFANTRY_VISION_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

class VisionIF {
public:

    static void init();

    enum vision_flag_t : uint8_t {
        NONE = 0,
        DETECTED = 1,
        TOP_KILLER_TRIGGERED,
    };

    __PACKED_STRUCT vision_command_t {
        uint8_t flag;
        uint16_t time_stamp;        // [0.1ms]
        int16_t yaw_delta;          // yaw relative angle [deg] * 100
        int16_t pitch_delta;        // pitch relative angle [deg] * 100
        int16_t dist;               // [mm]
        int16_t avg_light_angle;    // [deg] * 100
        int16_t imageX;             // pixel
        int16_t imageY;             // pixel
        int16_t remaining_time_to_target;  // [ms]
        int16_t period;                 // [ms]
    };

    /**
     * Received updated gimbal target.
     */
    static event_source_t command_received_event;

    static time_msecs_t get_last_valid_update_time() { return last_valid_update_time; }

    static void get_latest_valid_command(vision_command_t &command, float &absolute_yaw, float &absolute_pitch);

private:

    static vision_command_t latest_valid_command;
    static float latest_armor_yaw;    // [deg]
    static float latest_armor_pitch;  // [deg]

    // Gimbal angles at last vision command
    static float last_gimbal_yaw;    // [deg]
    static float last_gimbal_pitch;  // [deg]

    static time_msecs_t last_command_receive_time;        // [ms]
    static time_msecs_t last_valid_update_time;        // [ms]
    static time_msecs_t last_valid_update_time_delta;  // [ms]

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

    static void handle_vision_command(const vision_command_t &command);

};


#endif //META_INFANTRY_VISION_INTERFACE_H
