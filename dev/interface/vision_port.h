//
// Created by Kerui Zhu on 7/4/2019.
//

#ifndef META_INFANTRY_VISION_PORT_H
#define META_INFANTRY_VISION_PORT_H

#include "ch.hpp"
#include "hal.h"

class Vision {

public:

    struct VisionControlCommand {
        float gimbal_yaw_target;
        float gimbal_pitch_target;
    };

    static bool getControlCommand(VisionControlCommand &command);

    static void init(float velocity_update_fraction, time_msecs_t predict_forward_amount);

private:

    // Latest armor position (without compensation)
    static float target_armor_yaw;    // [deg]
    static float target_armor_pitch;  // [deg]

    // Gimbal angles at last vision command
    static float last_gimbal_yaw;    // [deg]
    static float last_gimbal_pitch;  // [deg]

    // Last vision command time
    static time_msecs_t last_update_time;  // [ms]

    static float latest_yaw_velocity;    // [deg/ms]
    static float latest_pitch_velocity;  // [deg/ms]

    static float velocity_update_fraction;
    static time_msecs_t predict_forward_amount;  // [ms]

    enum vision_flag_t : uint8_t {
        NONE = 0,
        DETECTED = 1
    };

    __PACKED_STRUCT vision_command_t {
        uint8_t flag;
        int16_t yaw_delta;    // yaw relative angle [deg] * 100
        int16_t pitch_delta;  // pitch relative angle [deg] * 100
        int16_t distance;     // [mm]
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
};


#endif //META_INFANTRY_VISION_PORT_H
