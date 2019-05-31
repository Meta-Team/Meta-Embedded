//
// Created by Administrator on 2019/1/15 0015.
//

#include "referee_interface.h"
#include "CRC16.h"
#include "CRC8.h"
#include "shell.h"
#include "memstreams.h"
#include "string.h"
#include "led.h"

Referee::game_state_t Referee::game_state;
Referee::game_result_t Referee::game_result;
Referee::game_robot_survivors_t Referee::game_robot_survivors;
Referee::event_data_t Referee::event_data;
Referee::supply_projectile_action_t Referee::supply_projectile_action;
Referee::supply_projectile_booking_t Referee::supply_projectile_booking;
Referee::game_robot_state_t Referee::game_robot_state;
Referee::power_heat_data_t Referee::power_heat_data;
Referee::game_robot_pos_t Referee::game_robot_pos;
Referee::buff_musk_t Referee::buff_musk;
Referee::aerial_robot_energy_t Referee::aerial_robot_energy;
Referee::robot_hurt_t Referee::robot_hurt;
Referee::shoot_data_t Referee::shoot_data;
Referee::client_custom_data_t Referee::client_custom_data;
Referee::robot_interactive_data_t Referee::robot_interactive_data;

Referee::frame_header_t Referee::frame_header;
uint16_t Referee::cmd_id;
Referee::rx_status_t Referee::rx_status;
uint8_t Referee::rx_buf[RX_BUF_SIZE];

const UARTConfig Referee::UART_CONFIG = {
        nullptr,
        nullptr,
        Referee::uart_rx_callback, // callback function when the buffer is filled
        nullptr,
        nullptr,
        115200, // speed
        0,
        0,
        0,
};

void Referee::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    // Handle received data and transfer status properly
    switch (rx_status) {

        case WAIT_STARTING_BYTE:

            if (rx_buf[0] == 0xA5) {
                rx_status = WAIT_REMAINING_HEADER;
            } // else, keep waiting for SOF

            break;

        case WAIT_REMAINING_HEADER:
            LED::green_toggle();
            if (Verify_CRC8_Check_Sum((uint8_t *) rx_buf, FRAME_HEADER_SIZE)) {
                memcpy(&frame_header, rx_buf, FRAME_HEADER_SIZE);
                memcpy(&cmd_id, rx_buf + FRAME_HEADER_SIZE, CMD_ID_SIZE);
                rx_status = WAIT_CMD_ID_DATA_TAIL; // go to next status
            } else {
                Shell::printfI("[REFEREE] Invalid frameHeader!" SHELL_NEWLINE_STR);
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_CMD_ID_DATA_TAIL:

            if (Verify_CRC16_Check_Sum((uint8_t *) &rx_buf,
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + frame_header.data_length + FRAME_TAIL_SIZE)) {

                switch (cmd_id) {
                    case 0x0201:
                        memcpy(&game_robot_state, rx_buf + FRAME_HEADER_SIZE + CMD_ID_SIZE, sizeof(game_robot_state));
                        break;
                    case 0x0202:
                        LED::red_toggle();
                        memcpy(&power_heat_data, rx_buf + FRAME_HEADER_SIZE + CMD_ID_SIZE, sizeof(power_heat_data));
                        break;
                    case 0x0207:
                        memcpy(&shoot_data, rx_buf + FRAME_HEADER_SIZE + CMD_ID_SIZE, sizeof(shoot_data));
                        break;
                    case 0x0206:

//                        Shell::printfI("[0x0206] data_length = %u" SHELL_NEWLINE_STR, frame_header.data_length);
                        memcpy(&robot_hurt, rx_buf + FRAME_HEADER_SIZE + CMD_ID_SIZE, sizeof(robot_hurt));
                    default:
                        // FIXME: temporarily disabled since not all ID has been implemented
                        // LOG_ERR("[REFEREE] Unknown cmd_id %u", cmd_id);
                        break;
                }
            } else {
                Shell::printfI("[REFEREE] Invalid data of type %u!" SHELL_NEWLINE_STR, cmd_id);
            }

            rx_status = WAIT_STARTING_BYTE;

            break;
    }

    switch (rx_status) {
        case WAIT_STARTING_BYTE:
            uartStartReceive(uartp, FRAME_SOF_SIZE, rx_buf);
            break;
        case WAIT_REMAINING_HEADER:
            uartStartReceive(uartp, FRAME_HEADER_SIZE - FRAME_SOF_SIZE, rx_buf + FRAME_SOF_SIZE);
            break;
        case WAIT_CMD_ID_DATA_TAIL:
            uartStartReceive(uartp, CMD_ID_SIZE + frame_header.data_length + FRAME_TAIL_SIZE, rx_buf + FRAME_HEADER_SIZE);
            break;
    }
}

void Referee::init() {

    // Start uart driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, rx_buf);

    LOG("sizeof(power_heat_data) = %u", sizeof(power_heat_data));
}