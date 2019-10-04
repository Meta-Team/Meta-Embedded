//
// Created by Administrator on 2019/1/15 0015.
//

#ifndef META_INFANTRY_REFEREE_INTERFACE_H
#define META_INFANTRY_REFEREE_INTERFACE_H

#include "ch.hpp"
#include "hal.h"


/**
 * @name Referee
 * @brief Interface for referee system
 * @pre Hardware is properly connected and UART pins are properly configured in board.h
 * @usage 1. Invoke init()
 *        2. Make use of received data or use APIs to send data
 * @note Designed for 2019 referee protocol V2.0, some functions may not be completed
 * @note Status machine of receiving:
 *       1. Receive byte one by one. If valid SOF 0xA5 is received, go to 2.
 *       2. Receive the remaining header. If header is valid (CRC8), go to 3. Otherwise, go back to 1.
 *       3. Receive cmdID. Check whether the id is valid and data length is correct. But even if there is error, still
 *          go to status 4, as data length given in header is validated, which is more credible than is unvalidated
 *          cmdID.
 *       4. Receive frameHeader.data_length bytes. Go to status 5.
 *       5. Validate data with CRC16. If it's valid, copy data to corresponding structure, or do nothing if failed.
 *          Go to status 1.
 */

#define REFEREE_USE_EVENTS  TRUE

class Referee {

public:

    enum receiver_index_t {
        CLIENT = 0,
        HERO_EMB = 1,
        ENGINEER_EMB = 2,
        STANDARD3_EMB = 3,
        STANDARD4_EMB = 4,
        STANDARD5_EMB = 5,
        AERIAL_EMB = 6,
        SENTRY_EMB = 7
    };

    enum signal_light_t {
        SIGNAL_0 = 0,
        SIGNAL_1 = 1,
        SIGNAL_2 = 2,
        SIGNAL_3 = 3,
        SIGNAL_4 = 4,
        SIGNAL_5 = 5
    };

    static constexpr uint16_t GAME_STATE_CMD_ID = 0x0001;
    __PACKED_STRUCT game_state_t {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
    };

    static constexpr uint16_t GAME_RESULT_CMD_ID = 0x0002;
    __PACKED_STRUCT game_result_t {
        uint8_t winner;
    };

    static constexpr uint16_t GAME_ROBOT_HP_CMD_ID = 0x0003;
    __PACKED_STRUCT game_robot_HP_t {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_base_HP;
    };

    static constexpr uint16_t EVENT_CMD_ID = 0x0101;
    __PACKED_STRUCT event_data_t {
        uint32_t event_type;
    };

    static constexpr uint16_t SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102;
    __PACKED_STRUCT supply_projectile_action_t {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    };

    static constexpr uint16_t SUPPLY_PROJECTILE_BOOKING_CMD_ID = 0x0103;
    __PACKED_STRUCT supply_projectile_booking_t {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_num;
    };

    static constexpr uint16_t REFEREE_WARNING_CMD_ID = 0x0103;
    __PACKED_STRUCT referee_warning_t {
        uint8_t level;
        uint8_t foul_robot_id;
    };

    static constexpr uint16_t GAME_ROBOT_STATE_CMD_ID = 0x0201;
    __PACKED_STRUCT game_robot_state_t {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t remain_HP;
        uint16_t max_HP;
        uint16_t shooter_heat0_cooling_rate;
        uint16_t shooter_heat0_cooling_limit;  // 17mm max heat
        uint16_t shooter_heat1_cooling_rate;
        uint16_t shooter_heat1_cooling_limit;  // 42mm max heat
        uint8_t mains_power_gimbal_output : 1;
        uint8_t mains_power_chassis_output : 1;
        uint8_t mains_power_shooter_output : 1;
    };

    static constexpr uint16_t POWER_HEAT_DATA_CMD_ID = 0x0202;
    __PACKED_STRUCT power_heat_data_t {
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t chassis_power_buffer;
        uint16_t shooter_heat0;
        uint16_t shooter_heat1;
    };

    static constexpr uint16_t GAME_ROBOT_POS_CMD_ID = 0x0203;
    __PACKED_STRUCT game_robot_pos_t {
        float x;
        float y;
        float z;
        float yaw;
    };

    static constexpr uint16_t BUFF_MUSK_CMD_ID = 0x0204;
    __PACKED_STRUCT buff_musk_t {
        uint8_t power_rune_buff;
    };

    static constexpr uint16_t AERIAL_ROBOT_ENERGY_CMD_ID = 0x0205;
    __PACKED_STRUCT aerial_robot_energy_t {
        uint8_t energy_point;
        uint8_t attack_time;
    };

    static constexpr uint16_t ROBOT_HURT_CMD_ID = 0x0206;
    __PACKED_STRUCT robot_hurt_t {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    };

    static constexpr uint16_t SHOOT_DATA_CMD_ID = 0x0207;
    __PACKED_STRUCT shoot_data_t {
        uint8_t bullet_type;
        uint8_t bullet_freq;
        float bullet_speed;
    };

    static constexpr uint16_t BULLET_REMAINING_CMD_ID = 0x0208;
    __PACKED_STRUCT bullet_remaining_t {
        uint16_t bullet_remaining_num;
    };

    /** Robot Interactive **/

    /**
     * data_cmd_id table:
     * ---------------------------------------------------------------------------------------------------------------------
     * CMD          Sender          Receiver            Description             Data
     * ---------------------------------------------------------------------------------------------------------------------
     *  0x0210      Sentry          Hero & Standards    Base Alarm              0: Alarm free; otherwise: Alarm triggered
     *  0x0211      Hero/Standards  Engineer            Robot Die               The position of the dead robot
     *  0x0212      Aerial          Sentry              Attack Instruction      The guide of Sentry gimbal movement
     * ---------------------------------------------------------------------------------------------------------------------
     */

    enum interactive_cmd_id_t{
        NOTHING = 0,
        AERIAL_TO_SENTRY = 0x0212
    };

    static const uint16_t INTERACTIVE_DATA_CMD_ID = 0x0301;

    __PACKED_STRUCT student_interactive_header_data_t {
        uint16_t data_cmd_id;
        uint16_t send_ID;
        uint16_t receiver_ID;
    };

    __PACKED_STRUCT client_custom_data_t {
        student_interactive_header_data_t header;
        float data1;
        float data2;
        float data3;
        uint8_t masks;
    };

    __PACKED_STRUCT aerial_to_sentry_t{
        uint8_t direction_mask : 4;
    };

    __PACKED_STRUCT robot_interactive_data_t {
        student_interactive_header_data_t header;
        union{
            aerial_to_sentry_t aerial_to_sentry_;
        };
    };

    /*** Received Data ***/

    /** Referee Data **/
    static game_state_t game_state;
    static game_result_t game_result;
    static game_robot_HP_t game_robot_HP;
    static event_data_t event_data;
    static supply_projectile_action_t supply_projectile_action;
    static supply_projectile_booking_t supply_projectile_booking;
    static referee_warning_t referee_warning;
    static game_robot_state_t game_robot_state;
    static power_heat_data_t power_heat_data;
    static game_robot_pos_t game_robot_pos;
    static buff_musk_t buff_musk;
    static aerial_robot_energy_t aerial_robot_energy;
    static robot_hurt_t robot_hurt;
    static shoot_data_t shoot_data;
    static bullet_remaining_t bullet_remaining;

    /** Interactive Data **/
    static aerial_to_sentry_t sentry_guiding_direction_r;

    /*** Send Data ***/
    static robot_interactive_data_t robot_data_send;
    static aerial_to_sentry_t sentry_guiding_direction_s;

    /**
     * Start referee interface
     */
    static void init(tprio_t sending_thread_prio);

    /**
     * Get current robotic ID
     * @return   game_robot_state.robot_id
     */
    static uint8_t get_self_id();

    /**
     * Set float numbers to be sent to client
     * @param index   Index of float number, 1-3
     * @param data    Number to be sent
     */
    static void set_client_number(unsigned index, float data);

    /**
     * Set signal lights status to be sent to client
     * @param signal_light   Index of signal lights, 0-5
     * @param turn_on        Status of signal lights
     */
    static void set_client_light(unsigned signal_light, bool turn_on);

    /**
     * Request to send data
     * @param receiver_id   CLIENT or robot ID
     * @param data_cmd_id   Command ID in data section, only available when sending data to other robots
     * @return   Whether the request is valid
     */
    static bool request_to_send(receiver_index_t receiver_id, interactive_cmd_id_t data_cmd_id = NOTHING);


#if REFEREE_USE_EVENTS

    /**
     * Event source when data received
     */
    static event_source_t data_received_event;

#endif

private:

    static client_custom_data_t client_custom_data;

    /**
     * Send data to client or other robots
     * @param receiver_id   CLIENT or robot ID
     * @param data_cmd_id   Command ID in data section, only available when sending data to other robots
     */
    static void send_data_(receiver_index_t receiver_id, interactive_cmd_id_t data_cmd_id = NOTHING);

    static bool to_send_client;
    static bool to_send_aerial_to_sentry;

    class DataSendingThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static DataSendingThread dataSendingThread;

    enum rx_status_t {
        WAIT_STARTING_BYTE,  // receive bytes one by one, waiting for 0xA5
        WAIT_REMAINING_HEADER,  // receive remaining header after SOF
        WAIT_CMD_ID_DATA_TAIL  // receive cmd_id, data section and 2-byte CRC16 tailing
    };

    static rx_status_t rx_status;

    static uint16_t tx_seq;

    __PACKED_STRUCT frame_header_t {
        uint8_t sof;  // start byte of header, 0xA5
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };

    static __PACKED_STRUCT package_t {
        frame_header_t header;
        uint16_t cmd_id;
        union {
            game_state_t game_state_;
            game_result_t game_result_;
            game_robot_HP_t game_robot_HP_;
            event_data_t event_data_;
            supply_projectile_action_t supply_projectile_action_;
            supply_projectile_booking_t supply_projectile_booking_;
            referee_warning_t referee_warning_;
            game_robot_state_t game_robot_state_;
            power_heat_data_t power_heat_data_;
            game_robot_pos_t game_robot_pos_;
            buff_musk_t buff_musk_;
            aerial_robot_energy_t aerial_robot_energy_;
            robot_hurt_t robot_hurt_;
            shoot_data_t shoot_data_;
            bullet_remaining_t bullet_remaining_;

            robot_interactive_data_t robot_interactive_data_;

            client_custom_data_t client_custom_data_;
        };
        uint16_t tail;
    } pak;

    static constexpr size_t FRAME_HEADER_SIZE = 5;
    static constexpr size_t FRAME_SOF_SIZE = 1;
    static constexpr size_t CMD_ID_SIZE = 2;
    static constexpr size_t FRAME_TAIL_SIZE = 2;

    static void uart_rx_callback(UARTDriver *uartp);  // only for internal use

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);

    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

    // See cpp file for configs
    static constexpr UARTDriver *UART_DRIVER = &UARTD7;
    static const UARTConfig UART_CONFIG;
};


#endif //META_INFANTRY_REFEREE_INTERFACE_H
