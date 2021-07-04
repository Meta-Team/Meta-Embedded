//
// Created by Kerui Zhu on 7/4/2019.
//

#ifndef META_INFANTRY_VISION_PORT_H
#define META_INFANTRY_VISION_PORT_H

#include "ch.hpp"
#include "hal.h"

class VisionPort {

public:

    struct VisionControlCommand {
        float gimbal_yaw_target;
        float gimbal_pitch_target;
    };

    static bool getControlCommand(VisionControlCommand &command);

    static void start(tprio_t thread_prio);

private:

    static float vision_yaw_target;
    static float vision_pitch_target;

    static float last_gimbal_yaw;
    static float last_gimbal_pitch;
    static time_msecs_t last_update_time;

    __PACKED_STRUCT header_t {
        uint8_t sof;  // start byte of header, 0xA5
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };

    static constexpr uint16_t VISION_CONTROL_CMD_ID = 0xEA01;

    enum VisionControlFlag : uint8_t {
        NONE = 0,
        DETECTED = 1
    };

    __PACKED_STRUCT vision_control_command_t {
        uint8_t flags;
        float yaw;
        float pitch;
    };

    __PACKED_STRUCT package_t {
        header_t header;
        uint16_t cmdID;
        union {  // the union takes the maximal size
            vision_control_command_t vision;
        };
        uint16_t tail;  // just for reference, the offset is not correct
    };

    static package_t pak;

    static void uart_rx_callback(UARTDriver *uartp);  // only for internal use
    static void uart_err_callback(UARTDriver *uartp, uartflags_t e);
    static void uart_char_callback(UARTDriver *uartp, uint16_t c);

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
