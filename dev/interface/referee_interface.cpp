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

int Referee::count_;
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

Referee::packet_t Referee::pak;

Referee::frame_header_t Referee::frame_header;
Referee::rx_status_t Referee::rx_status;

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

    chSysLockFromISR();

    uint8_t* pak_uint8 = (uint8_t *)&pak;

    // Handle received data and transfer status properly
    switch (rx_status) {

        case WAIT_STARTING_BYTE:
            LED::green_toggle();
            if (pak_uint8[0] == 0xA5) {
                rx_status = WAIT_REMAINING_HEADER;
            }
            break;

        case WAIT_REMAINING_HEADER:

            if (Verify_CRC8_Check_Sum(pak_uint8, FRAME_HEADER_SIZE)) {
                rx_status = WAIT_CMD_ID_DATA_TAIL; // go to next status
            } else {
//                Shell::printfI("[REFEREE] Invalid frameHeader!" SHELL_NEWLINE_STR);
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_CMD_ID_DATA_TAIL:

            if (Verify_CRC16_Check_Sum(pak_uint8,
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + frame_header.data_length + FRAME_TAIL_SIZE)) {

                switch (pak.cmd_id) {
                    case 0x0201:
                        game_robot_state = pak.game_robot_state_;
                        break;
                    case 0x0202:
                        LED::red_toggle();
                        power_heat_data = pak.power_heat_data_;
                        count_++;
                        break;
                    case 0x0207:
                        shoot_data = pak.shoot_data_;
                        break;
                    case 0x0206:

//                        Shell::printfI("[0x0206] data_length = %u" SHELL_NEWLINE_STR, frame_header.data_length);
                        robot_hurt = pak.robot_hurt_;
                    default:
                        // FIXME: temporarily disabled since not all ID has been implemented
                        // LOG_ERR("[REFEREE] Unknown cmd_id %u", cmd_id);
                        break;
                }
            } else {
//                Shell::printfI("[REFEREE] Invalid data of type %u!" SHELL_NEWLINE_STR, cmd_id);
            }

            rx_status = WAIT_STARTING_BYTE;

            break;
    }

    switch (rx_status) {
        case WAIT_STARTING_BYTE:
            uartStartReceiveI(uartp, FRAME_SOF_SIZE, pak_uint8);
            break;
        case WAIT_REMAINING_HEADER:
            uartStartReceiveI(uartp, FRAME_HEADER_SIZE - FRAME_SOF_SIZE, pak_uint8 + FRAME_SOF_SIZE);
            break;
        case WAIT_CMD_ID_DATA_TAIL:
            uartStartReceiveI(uartp, CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE, pak_uint8 + FRAME_HEADER_SIZE);
            break;
    }

    chSysUnlockFromISR();

}

void Referee::init() {
LOG("1");
count_ = 0;
    // Start uart driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, &pak);

    LOG("sizeof(power_heat_data) = %u", sizeof(power_heat_data));
}