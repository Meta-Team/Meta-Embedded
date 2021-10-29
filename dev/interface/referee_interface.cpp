//
// Created by Administrator on 2019/1/15 0015.
//

#include "referee_interface.h"

#include "led.h"
#include "shell.h"

#include "CRC16.h"
#include "CRC8.h"

/** Public Parameters **/
Referee::game_state_t Referee::game_state;
Referee::game_result_t Referee::game_result;
Referee::game_robot_HP_t Referee::game_robot_HP;
Referee::event_data_t Referee::event_data;
Referee::supply_projectile_action_t Referee::supply_projectile_action;
Referee::supply_projectile_booking_t Referee::supply_projectile_booking;
Referee::referee_warning_t Referee::referee_warning;
Referee::ext_dart_remaining_time_t Referee::ext_dart_remaining_time;
Referee::game_robot_state_t Referee::robot_state;
Referee::power_heat_data_t Referee::power_heat;
Referee::game_robot_pos_t Referee::robot_pos;
Referee::buff_musk_t Referee::buff_musk;
Referee::aerial_robot_energy_t Referee::aerial_robot_energy;
Referee::robot_hurt_t Referee::robot_hurt;
Referee::shoot_data_t Referee::shoot_data;
Referee::bullet_remaining_t Referee::bullet_remaining;
Referee::dart_client_t Referee::dart_client;

Referee::client_custom_data_t Referee::client_custom_data;
Referee::robot_interactive_data_t Referee::robot_data_send;

Referee::graphic_data_struct_t Referee::graphic_data_buffer[7];
int Referee::graphic_buffer_index = 0;

#if REFEREE_USE_EVENTS
// See macro EVENTSOURCE_DECL() for initialization style
event_source_t Referee::data_received_event = {(event_listener_t *) (&Referee::data_received_event)};;
#endif

/** Private Parameters **/
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
    robot_state.robot_id = 0;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, &pak);
}

uint8_t Referee::get_self_id() {
    return robot_state.robot_id;
}

void Referee::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLockFromISR();  /// --- ENTER I-Locked state. DO NOT use LOG, printf, non I-Class functions or return ---

    uint8_t *pak_uint8 = (uint8_t *) &pak;

    // Handle received data and transfer status properly
    switch (rx_status) {

        case WAIT_STARTING_BYTE:
            if (pak_uint8[0] == 0xA5) {
                rx_status = WAIT_REMAINING_HEADER;
            }
            break;

        case WAIT_REMAINING_HEADER:

            if (verify_crc8_check_sum(pak_uint8, FRAME_HEADER_SIZE)) {
                rx_status = WAIT_CMD_ID_DATA_TAIL; // go to next status
            } else {
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_CMD_ID_DATA_TAIL:

            if (verify_crc16_check_sum(pak_uint8,
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE)) {

                switch (pak.cmd_id) {

                    case GAME_ROBOT_STATE_CMD_ID:
                        if (robot_state.robot_id == 0) {
                            // If it is the first time that robot_state cmd is received, we need to initialize robot identity
                            robot_state = pak.game_robot_state_;
                            int robot_id = robot_state.robot_id;
                            client_custom_data.header.send_ID = robot_id;
                            client_custom_data.header.receiver_ID = (robot_id % 100) + (robot_id / 100) * 0x064 + 0x100;
                            robot_data_send.header.send_ID = robot_id;
                        } else robot_state = pak.game_robot_state_;
                        break;
                    case POWER_HEAT_DATA_CMD_ID:
                        power_heat = pak.power_heat_data_;
                        break;
                    case ROBOT_HURT_CMD_ID:
                        robot_hurt = pak.robot_hurt_;
                        break;
                    case SHOOT_DATA_CMD_ID:
                        shoot_data = pak.shoot_data_;
                        break;
                    case GAME_STATE_CMD_ID:
                        game_state = pak.game_state_;
                        break;
                    case GAME_RESULT_CMD_ID:
                        game_result = pak.game_result_;
                        break;
                    case GAME_ROBOT_HP_CMD_ID:
                        game_robot_HP = pak.game_robot_HP_;
                        break;
                    case EVENT_CMD_ID:
                        event_data = pak.event_data_;
                        break;
                    case SUPPLY_PROJECTILE_ACTION_CMD_ID:
                        supply_projectile_action = pak.supply_projectile_action_;
                        break;
                    case REFEREE_WARNING_CMD_ID:
                        referee_warning = pak.referee_warning_;
                        break;
                    case EXT_DART_REMAINING_TIME_CMD_ID:
                        ext_dart_remaining_time = pak.ext_dart_remaining_time_;
                        break;
                    case GAME_ROBOT_POS_CMD_ID:
                        robot_pos = pak.game_robot_pos_;
                        break;
                    case BUFF_MUSK_CMD_ID:
                        buff_musk = pak.buff_musk_;
                        break;
                    case AERIAL_ROBOT_ENERGY_CMD_ID:
                        aerial_robot_energy = pak.aerial_robot_energy_;
                        break;
                    case BULLET_REMAINING_CMD_ID:
                        bullet_remaining = pak.bullet_remaining_;
                        break;
                    case DART_CLIENT_CMD_ID:
                        dart_client = pak.dart_client_;
                        break;
//                    case INTERACTIVE_DATA_CMD_ID: // robot_interactive_data
//                        // Check whether the message is from the same team
//                        if ((game_robot_state.robot_id > 10) ^ (pak.robot_interactive_data_.header.send_ID > 10)) break;
//                        // Check whether the message is for this robot
//                        if (game_robot_state.robot_id != pak.robot_interactive_data_.header.receiver_ID) break;
//                        // If the message pass the check, record it in the corresponding place
//                        switch (pak.robot_interactive_data_.header.data_cmd_id){
//                            case AERIAL_TO_SENTRY:
//                                sentry_guiding_direction_r = pak.robot_interactive_data_.aerial_to_sentry_;
//                        }
//                        //switch (robot_data_receive.)
//                        break;

                    default:
                        break;
                }
            }

#if REFEREE_USE_EVENTS
            chEvtBroadcastFlagsI(&data_received_event, pak.cmd_id);
#endif
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
            uartStartReceiveI(uartp, CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE,
                              pak_uint8 + FRAME_HEADER_SIZE);
            break;
    }

    chSysUnlockFromISR();  /// --- EXIT S-Locked state ---

}

void Referee::set_tx_data_for_graphics() {
    if (graphic_buffer_index != 0) {

        if (graphic_buffer_index == 1) {   // 1 buffer used
            client_custom_data.header.data_cmd_id = 0x0101;
            client_custom_data.ext_client_custom_graphic_single.grapic_data_ = graphic_data_buffer[0];
        } else if (graphic_buffer_index < 3) {  // 2 buffer used
            client_custom_data.header.data_cmd_id = 0x0102;
            client_custom_data.ext_client_custom_graphic_double.grapic_data_[0] = graphic_data_buffer[0];
            client_custom_data.ext_client_custom_graphic_double.grapic_data_[1] = graphic_data_buffer[1];
        } else if (graphic_buffer_index < 6) {  // 3 - 5 buffer used
            client_custom_data.header.data_cmd_id = 0x0103;
            int i;
            for (i = 0; i < graphic_buffer_index; i++) {
                client_custom_data.ext_client_custom_graphic_five.grapic_data_[i] = graphic_data_buffer[i];
            }
            // Zero padding
            for (; i < 5; i++) {
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].graphic_name[0] = 'N';
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].graphic_name[1] = '/';
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].graphic_name[2] = 'A';
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].operate_type = 0;
            }

        } else if (graphic_buffer_index < 8) { // 6 - 7 buffer full filled
            client_custom_data.header.data_cmd_id = 0x0104;
            int i;
            for (i = 0; i < graphic_buffer_index; i++) {
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i] = graphic_data_buffer[i];
            }
            // Zero padding
            for (; i < 7; i++) {
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].graphic_name[0] = 'N';
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].graphic_name[1] = '/';
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].graphic_name[2] = 'A';
                client_custom_data.ext_client_custom_graphic_seven.grapic_data_[i].operate_type = 0;
            }
        }

    }
}

void Referee::set_tx_data_for_label(const ext_client_custom_character_t &character_data) {
    client_custom_data.header.data_cmd_id = 0x0110;
    client_custom_data.ext_client_custom_character = character_data;
}

void Referee::set_tx_data_for_deleting_layer(uint32_t layer) {
    client_custom_data.header.data_cmd_id = 0x0100;
    client_custom_data.ext_client_custom_graphic_delete.operate_type = 1;
    client_custom_data.ext_client_custom_graphic_delete.layer = layer;
}

void Referee::set_tx_data_for_deleting_all() {
    client_custom_data.header.data_cmd_id = 0x0100;
    client_custom_data.ext_client_custom_graphic_delete.operate_type = 2;
    client_custom_data.ext_client_custom_graphic_delete.layer = 0;
}

void Referee::send_tx_data() {
    send_data_(Referee::CLIENT);
    graphic_buffer_index = 0;
}

void Referee::send_data_(receiver_index_t receiver_id) {
    if (robot_state.robot_id == 0) return;
    package_t tx_pak;
    size_t tx_pak_size = 0;
    if (receiver_id == CLIENT) {
        int client_custom_data_length = 0;
        switch (client_custom_data.header.data_cmd_id) {
            case 0x0101:
                client_custom_data_length = sizeof(Referee::student_interactive_header_data_t)
                                            + sizeof(Referee::ext_client_custom_graphic_single_t);
                break;
            case 0x0102:
                client_custom_data_length = sizeof(Referee::student_interactive_header_data_t)
                                            + sizeof(Referee::ext_client_custom_graphic_double_t);
                break;
            case 0x0103:
                client_custom_data_length = sizeof(Referee::student_interactive_header_data_t)
                                            + sizeof(Referee::ext_client_custom_graphic_five_t);
                break;
            case 0x0104:
                client_custom_data_length = sizeof(Referee::student_interactive_header_data_t)
                                            + sizeof(Referee::ext_client_custom_graphic_seven_t);
                break;
            case 0x0110:
                client_custom_data_length = sizeof(Referee::student_interactive_header_data_t)
                                            + sizeof(Referee::ext_client_custom_character_t);
                break;
            case 0x0100:
                client_custom_data_length = sizeof(Referee::student_interactive_header_data_t)
                                            + sizeof(Referee::ext_client_custom_graphic_delete_t);
                break;
        }
        tx_pak.header.sof = 0xA5;
        tx_pak.header.data_length = client_custom_data_length;
        tx_pak.header.seq = tx_seq++;
        append_crc8_check_sum((uint8_t *) &tx_pak, FRAME_HEADER_SIZE);
        tx_pak.cmd_id = 0x0301;

        tx_pak.client_custom_data_ = client_custom_data;
        tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + client_custom_data_length + FRAME_TAIL_SIZE;
    }
    append_crc16_check_sum((uint8_t *) &tx_pak, tx_pak_size);
    uartSendTimeout(UART_DRIVER, &tx_pak_size, &tx_pak, TIME_MS2I(30));
}

bool Referee::add_tx_graphic(const graphic_data_struct_t &graph_data) {
    if (graphic_buffer_index < 7) {
        graphic_data_buffer[graphic_buffer_index] = graph_data;
        graphic_buffer_index++;
        return true;
    } else {
        return false;
    }
}