//
// Created by Administrator on 2019/1/15 0015.
//

#include "ch.hpp"
#include "referee_interface.h"
#include "CRC16.h"
#include "CRC8.h"
#include "shell.h"
#include "memstreams.h"
#include "string.h"
#include "led.h"

/** Public Parameters **/
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
Referee::robot_interactive_data_t Referee::robot_data_receive[7];
Referee::client_custom_data_t Referee::client_custom_data;
Referee::robot_interactive_data_t Referee::robot_data_send[7];

/** Private Parameters **/
uint16_t Referee::robot_id;
uint16_t Referee::client_id;
bool Referee::is_blue;
Referee::rx_status_t Referee::rx_status;
uint16_t Referee::tx_seq = 0;
Referee::package_t Referee::pak;

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

void Referee::init() {
    // Start uart driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    robot_id = 0;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, &pak);
    LOG("sizeof(power_heat_data) = %u", sizeof(power_heat_data));
}

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
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE)) {

                switch (pak.cmd_id) {

                    case 0x0201:
                        game_robot_state = pak.game_robot_state_;
                        if (robot_id == 0) set_robot_info(game_robot_state.robot_id);
                        break;
                    case 0x0202:
                        LED::red_toggle();
                        power_heat_data = pak.power_heat_data_;
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
            }
//            } else {
//                Shell::printfI("[REFEREE] Invalid data of type %u!" SHELL_NEWLINE_STR, cmd_id);
//            }

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

void Referee::send_data(uint16_t receiver_id, uint16_t data_cmd_id) {
    if (robot_id == 0)
        return;
    package_t tx_pak;
    tx_pak.header.sof = 0xA5;
    if (receiver_id == client_id) {
        tx_pak.header.data_length = sizeof(client_custom_data_t);
    } else {
        if (data_cmd_id == 0) return;
        tx_pak.header.data_length = sizeof(robot_interactive_data_t);
    }
    tx_pak.header.seq = tx_seq++;
    Append_CRC8_Check_Sum((uint8_t *)&tx_pak, FRAME_HEADER_SIZE);
    tx_pak.cmd_id = 0x0301;
    size_t tx_pak_size;
    if (receiver_id == client_id){
        tx_pak.client_custom_data_ = client_custom_data;
        tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(client_custom_data_t) + FRAME_TAIL_SIZE;
    } else{
        uint16_t receiver_index = receiver_id % 16;
        robot_data_send[receiver_index].header.send_ID = robot_id;
        robot_data_send[receiver_index].header.data_cmd_id = data_cmd_id;
        robot_data_send[receiver_index].header.receiver_ID = receiver_id;
        tx_pak.robot_interactive_data_ = robot_data_send[receiver_index];
        tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(robot_interactive_data_t) + FRAME_TAIL_SIZE;
    }
    Append_CRC16_Check_Sum((uint8_t *)&tx_pak, tx_pak_size);
    uartSendTimeout(UART_DRIVER, &tx_pak_size, &tx_pak, TIME_MS2I(20));
}

void Referee::set_robot_info(int id) {
    robot_id = id;
    is_blue = (id < 10);
    client_id = 0x0100 + (id / 10 * 16) + (id % 10);
    client_custom_data.header.send_ID = robot_id;
    client_custom_data.header.receiver_ID = client_id;
    client_custom_data.header.data_cmd_id = 0xD180;
}

void Referee::set_client_data(Referee::client_data_t data_type, float data) {
    if (data_type == DATA_1) client_custom_data.data1 = data;
    else if (data_type == DATA_2) client_custom_data.data2 = data;
    else if (data_type == DATA_3) client_custom_data.data3 = data;
}

void Referee::set_signal_light(Referee::signal_light_t signalLight, bool turn_on) {
    uint8_t picker = (1U) << signalLight;
    if (turn_on){
        client_custom_data.masks |= picker;
    } else{
        picker = ~picker;
        client_custom_data.masks &= picker;
    }
}
