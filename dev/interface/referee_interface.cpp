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
Referee::game_robot_state_t Referee::game_robot_state;
Referee::power_heat_data_t Referee::power_heat_data;
Referee::game_robot_pos_t Referee::game_robot_pos;
Referee::buff_musk_t Referee::buff_musk;
Referee::aerial_robot_energy_t Referee::aerial_robot_energy;
Referee::robot_hurt_t Referee::robot_hurt;
Referee::shoot_data_t Referee::shoot_data;
Referee::bullet_remaining_t Referee::bullet_remaining;
Referee::aerial_to_sentry_t Referee::sentry_guiding_direction_r;

Referee::client_custom_data_t Referee::client_custom_data;
Referee::robot_interactive_data_t Referee::robot_data_send;
Referee::aerial_to_sentry_t Referee::sentry_guiding_direction_s;

bool Referee::to_send_client = false;
bool Referee::to_send_aerial_to_sentry = false;

Referee::DataSendingThread Referee::dataSendingThread;

#if REFEREE_USE_EVENTS
// See macro EVENTSOURCE_DECL() for initialization style
event_source_t Referee::data_received_event = {(event_listener_t *)(&Referee::data_received_event)};;
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

void Referee::init(tprio_t sending_thread_prio) {
    // Start uart driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    game_robot_state.robot_id = 0;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, &pak);

    dataSendingThread.start(sending_thread_prio);
}

uint8_t Referee::get_self_id() {
    return game_robot_state.robot_id;
}

void Referee::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLockFromISR();  /// --- ENTER I-Locked state. DO NOT use LOG, printf, non I-Class functions or return ---

    uint8_t* pak_uint8 = (uint8_t *)&pak;

    // Handle received data and transfer status properly
    switch (rx_status) {

        case WAIT_STARTING_BYTE:
            if (pak_uint8[0] == 0xA5) {
                rx_status = WAIT_REMAINING_HEADER;
            }
            break;

        case WAIT_REMAINING_HEADER:

            if (Verify_CRC8_Check_Sum(pak_uint8, FRAME_HEADER_SIZE)) {
                rx_status = WAIT_CMD_ID_DATA_TAIL; // go to next status
            } else {
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_CMD_ID_DATA_TAIL:

            if (Verify_CRC16_Check_Sum(pak_uint8,
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE)) {

                switch (pak.cmd_id) {

                    case GAME_ROBOT_STATE_CMD_ID:
                        if (game_robot_state.robot_id == 0) {
                            // If it is the first time that robot_state cmd is received, we need to initialize robot identity
                            game_robot_state = pak.game_robot_state_;
                            int robot_id = game_robot_state.robot_id;
                            client_custom_data.header.send_ID = robot_id;
                            client_custom_data.header.receiver_ID = 0x0100 + (robot_id / 10 * 16) + (robot_id % 10);
                            client_custom_data.header.data_cmd_id = 0xD180;
                            robot_data_send.header.send_ID = robot_id;
                        } else game_robot_state = pak.game_robot_state_;
                        break;
                    case POWER_HEAT_DATA_CMD_ID:
                        power_heat_data = pak.power_heat_data_;
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
                    case GAME_ROBOT_POS_CMD_ID:
                        game_robot_pos = pak.game_robot_pos_;
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
                    case INTERACTIVE_DATA_CMD_ID: // robot_interactive_data
                        // Check whether the message is from the same team
                        if ((game_robot_state.robot_id > 10) ^ (pak.robot_interactive_data_.header.send_ID > 10)) break;
                        // Check whether the message is for this robot
                        if (game_robot_state.robot_id != pak.robot_interactive_data_.header.receiver_ID) break;
                        // If the message pass the check, record it in the corresponding place
                        switch (pak.robot_interactive_data_.header.data_cmd_id){
                            case AERIAL_TO_SENTRY:
                                sentry_guiding_direction_r = pak.robot_interactive_data_.aerial_to_sentry_;
                        }
                        //switch (robot_data_receive.)
                        break;

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
            uartStartReceiveI(uartp, CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE, pak_uint8 + FRAME_HEADER_SIZE);
            break;
    }

    chSysUnlockFromISR();  /// --- EXIT S-Locked state ---

}

bool Referee::request_to_send(Referee::receiver_index_t receiver_id, Referee::interactive_cmd_id_t data_cmd_id) {
    if (receiver_id == CLIENT) {
        to_send_client = true;
    } else {
        switch (data_cmd_id){
            case AERIAL_TO_SENTRY:
                to_send_aerial_to_sentry = true;
                break;
            default:
                return false;
        }
    }
    return true;
}

void Referee::DataSendingThread::main() {
    setName("RefereeSend");
    while (!shouldTerminate()) {

        // The following order indicates priority of message

        if (to_send_aerial_to_sentry) {
            send_data_(SENTRY_EMB, AERIAL_TO_SENTRY);
            to_send_aerial_to_sentry = false;
        } else if (to_send_client) {
            send_data_(CLIENT);
            to_send_client = false;
        }

        sleep(TIME_MS2I(100));  // maximum sending interval 10 Hz
    }
}

void Referee::send_data_(receiver_index_t receiver_id, interactive_cmd_id_t data_cmd_id) {
    if (game_robot_state.robot_id == 0)
        return;
    package_t tx_pak;
    tx_pak.header.sof = 0xA5;
    if (receiver_id == CLIENT) {
        tx_pak.header.data_length = sizeof(client_custom_data_t);
    } else {
        switch (data_cmd_id){
            case AERIAL_TO_SENTRY:
                tx_pak.header.data_length = sizeof(student_interactive_header_data_t) + sizeof(aerial_to_sentry_t);
                break;
            case NOTHING:
            default:
                return;
        }
    }
    tx_pak.header.seq = tx_seq++;
    Append_CRC8_Check_Sum((uint8_t *)&tx_pak, FRAME_HEADER_SIZE);

    tx_pak.cmd_id = 0x0301;

    size_t tx_pak_size = 0;
    if (receiver_id == CLIENT){
        tx_pak.client_custom_data_ = client_custom_data;
        tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(client_custom_data_t) + FRAME_TAIL_SIZE;
    } else{
        robot_data_send.header.receiver_ID = (game_robot_state.robot_id / 10) * 10 + receiver_id;
        robot_data_send.header.data_cmd_id = data_cmd_id;

        if (data_cmd_id == AERIAL_TO_SENTRY){
                tx_pak.robot_interactive_data_.aerial_to_sentry_ = sentry_guiding_direction_s;
                tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(student_interactive_header_data_t) + sizeof(aerial_to_sentry_t) + FRAME_TAIL_SIZE;
        }
    }
    Append_CRC16_Check_Sum((uint8_t *)&tx_pak, tx_pak_size);
    uartSendTimeout(UART_DRIVER, &tx_pak_size, &tx_pak, TIME_MS2I(20));
}

void Referee::set_client_number(unsigned index, float data) {
    if (index == 1) {
        client_custom_data.data1 = data;
    } else if (index == 2) {
        client_custom_data.data2 = data;
    } else if (index == 3) {
        client_custom_data.data3 = data;
    } else return;
}

void Referee::set_client_light(unsigned signal_light, bool turn_on) {
    if (signal_light >= 6)
        return;
    uint8_t picker = (1U) << signal_light;
    if (turn_on){
        client_custom_data.masks |= picker;
    } else{
        picker = ~picker;
        client_custom_data.masks &= picker;
    }
}
