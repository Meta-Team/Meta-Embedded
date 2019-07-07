//
// Created by Kerui Zhu on 7/4/2019.
//

#ifndef META_INFANTRY_VISION_PORT_H
#define META_INFANTRY_VISION_PORT_H

#include "ch.hpp"
#include "hal.h"

/**
 * @brief the interface for vision board
 * @pre hardware is properly connected and GPIOs are properly configured in board.h
 * @pre call start() to start UART port and start receive
 * @note the status machine of receiving:
 *       1. Receive byte one by one. If valid SOF 0xA5 is received, go to 2.
 *       2. Receive the remaining header. If header is valid (CRC8), go to 3. Otherwise, go back to 1.
 *       3. Receive cmdID. Check whether the id is valid and data length is correct. But even if there is error, still
 *          go to status 4, as data length given in header is validated, which is more credible than is unvalidated
 *          cmdID.
 *       4. Receive frameHeader.data_length bytes. Go to status 5.
 *       5. Validate data with CRC16. If it's valid, copy data to corresponding structure, or do nothing if failed.
 *          Go to status 1.
 * @attention Designed for 2017 referee system
 */

class VisionPort {
public:

    static __PACKED_STRUCT enemy_info_t{
        float yaw_angle;
        float pitch_angle;
        float distance;
    } enemy_info;

    static time_msecs_t last_update_time;

    static void init();

    static void send_gimbal(float yaw, float pitch);

    static void send_enemy_color(bool is_blue);

private:

    __PACKED_STRUCT frame_header_t {
        uint8_t sof;  // start byte of header, 0xA5
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };

    __PACKED_STRUCT gimbal_current_t {
        float yaw;
        float pitch;
    };

    __PACKED_STRUCT enemy_color_t {
        uint8_t enemy_color;  // RED = 0, BLUE = 1
    };

    __PACKED_STRUCT package_t {
        frame_header_t header;
        uint16_t cmd_id;
        union {
            enemy_info_t enemy_info_;
            gimbal_current_t gimbal_current_;
            enemy_color_t enemy_color_;
        };
        uint16_t tail;
    };

    static package_t pak;

    static void uart_rx_callback(UARTDriver *uartp);  // only for internal use

    enum rx_status_t {
        WAIT_STARTING_BYTE,  // receive bytes one by one, waiting for 0xA5
        WAIT_REMAINING_HEADER,  // receive remaining header after SOF
        WAIT_CMD_ID_DATA_TAIL  // receive cmd_id, data section and 2-byte CRC16 tailing
    };

    static constexpr size_t FRAME_HEADER_SIZE = 5;
    static constexpr size_t FRAME_SOF_SIZE = 1;
    static constexpr size_t CMD_ID_SIZE = 2;
    static constexpr size_t FRAME_TAIL_SIZE = 2;

    static rx_status_t rx_status;

    static uint16_t tx_seq;

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

    // See cpp file for configs
    static constexpr UARTDriver *UART_DRIVER = &UARTD8;
    static const UARTConfig UART_CONFIG;
};


#endif //META_INFANTRY_VISION_PORT_H
