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

    static constexpr uint16_t GAME_STATE_CMD_ID = 0x0001;
    __PACKED_STRUCT game_state_t {
        uint8_t game_type: 4;
        uint8_t game_progress: 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
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
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    };

    static constexpr uint16_t GAME_ROBOT_ICRA_CMD_ID = 0x0005;
    __PACKED_STRUCT icra_data_t {
        uint8_t F1_zone_status: 1;
        uint8_t F1_zone_buff_debuff_status: 3;
        uint8_t F2_zone_status: 1;
        uint8_t F2_zone_buff_debuff_status: 3;
        uint8_t F3_zone_status: 1;
        uint8_t F3_zone_buff_debuff_status: 3;
        uint8_t F4_zone_status: 1;
        uint8_t F4_zone_buff_debuff_status: 3;
        uint8_t F5_zone_status: 1;
        uint8_t F5_zone_buff_debuff_status: 3;
        uint8_t F6_zone_status: 1;
        uint8_t F6_zone_buff_debuff_status: 3;
        uint16_t red1_bullet_left;
        uint16_t red2_bullet_left;
        uint16_t blue1_bullet_left;
        uint16_t blue2_bullet_left;
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
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    };

    static constexpr uint16_t REFEREE_WARNING_CMD_ID = 0x0104;
    __PACKED_STRUCT referee_warning_t {
        uint8_t level;
        uint8_t foul_robot_id;
    };

    static constexpr uint16_t EXT_DART_REMAINING_TIME_CMD_ID = 0x0105;
    __PACKED_STRUCT ext_dart_remaining_time_t {
        uint8_t dart_remaining_time;
    };

    static constexpr uint16_t GAME_ROBOT_STATE_CMD_ID = 0x0201;
    __PACKED_STRUCT game_robot_state_t {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t remain_HP;
        uint16_t max_HP;
        uint16_t shooter_id1_17mm_cooling_rate;
        uint16_t shooter_id1_17mm_cooling_limit;
        uint16_t shooter_id1_17mm_speed_limit;
        uint16_t shooter_id2_17mm_cooling_rate;
        uint16_t shooter_id2_17mm_cooling_limit;
        uint16_t shooter_id2_17mm_speed_limit;
        uint16_t shooter_id1_42mm_cooling_rate;
        uint16_t shooter_id1_42mm_cooling_limit;
        uint16_t shooter_id1_42mm_speed_limit;
        uint16_t chassis_power_limit;
        uint8_t mains_power_gimbal_output: 1;
        uint8_t mains_power_chassis_output: 1;
        uint8_t mains_power_shooter_output: 1;
    };

    static constexpr uint16_t POWER_HEAT_DATA_CMD_ID = 0x0202;
    __PACKED_STRUCT power_heat_data_t {
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t chassis_power_buffer;
        uint16_t shooter_id1_17mm_cooling_heat;
        uint16_t shooter_id2_17mm_cooling_heat;
        uint16_t shooter_id1_42mm_cooling_heat;
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
        uint8_t attack_time;
    };

    static constexpr uint16_t ROBOT_HURT_CMD_ID = 0x0206;
    __PACKED_STRUCT robot_hurt_t {
        uint8_t armor_id: 4;
        uint8_t hurt_type: 4;
    };

    static constexpr uint16_t SHOOT_DATA_CMD_ID = 0x0207;
    __PACKED_STRUCT shoot_data_t {
        uint8_t bullet_type;
        uint8_t shooter_id;
        uint8_t bullet_freq;
        float bullet_speed;
    };

    static constexpr uint16_t BULLET_REMAINING_CMD_ID = 0x0208;
    __PACKED_STRUCT bullet_remaining_t {
        uint16_t bullet_remaining_num_17mm;
        uint16_t bullet_remaining_num_42mm;
        uint16_t coin_remaining_num;
    };

    static constexpr uint16_t RFID_STAT_ID = 0x209;
    __PACKED_STRUCT rfid_stat_t {
        uint32_t rfid_status;
    };

    static constexpr uint16_t DART_CLIENT_CMD_ID = 0x20A;
    __PACKED_STRUCT dart_client_t {
        uint8_t dart_launch_opening_status;
        uint8_t dart_attack_target;
        uint16_t target_change_time;
        uint16_t operate_launch_cmd_time;
    };

    /**Graphic Settings*/

    static const uint16_t INTERACTIVE_DATA_CMD_ID = 0x0301;

    __PACKED_STRUCT student_interactive_header_data_t {
        uint16_t data_cmd_id;
        uint16_t send_ID;
        uint16_t receiver_ID;
    };

    __PACKED_STRUCT ext_client_custom_graphic_delete_t {
        uint8_t operate_type;
        uint8_t layer;
    };

    __PACKED_STRUCT graphic_data_struct_t {
        uint8_t graphic_name[3];
        uint32_t operate_type: 3;
        uint32_t graphic_type: 3;
        uint32_t layer: 4;
        uint32_t color: 4;
        uint32_t start_angle: 9;
        uint32_t end_angle: 9;
        uint32_t width: 10;
        uint32_t start_x: 11;
        uint32_t start_y: 11;
        uint32_t radius: 10;
        uint32_t end_x: 11;
        uint32_t end_y: 11;
    };

    __PACKED_STRUCT ext_client_custom_graphic_single_t {
        graphic_data_struct_t grapic_data_;
    };

    __PACKED_STRUCT ext_client_custom_graphic_double_t {
        graphic_data_struct_t grapic_data_[2];
    };

    __PACKED_STRUCT ext_client_custom_graphic_five_t {
        graphic_data_struct_t grapic_data_[5];
    };

    __PACKED_STRUCT ext_client_custom_graphic_seven_t {
        graphic_data_struct_t grapic_data_[7];
    };

    __PACKED_STRUCT ext_client_custom_character_t {
        graphic_data_struct_t grapic_data_struct;
        uint8_t data[30];
    };

    __PACKED_STRUCT client_custom_data_t {
        student_interactive_header_data_t header;
        union {
            ext_client_custom_graphic_delete_t ext_client_custom_graphic_delete;
            ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
            ext_client_custom_graphic_double_t ext_client_custom_graphic_double;
            ext_client_custom_graphic_five_t ext_client_custom_graphic_five;
            ext_client_custom_graphic_seven_t ext_client_custom_graphic_seven;
            ext_client_custom_character_t ext_client_custom_character;
        };
    };

    __PACKED_STRUCT aerial_to_sentry_t {
        uint8_t direction_mask: 4;
    };

    __PACKED_STRUCT robot_interactive_data_t {
        student_interactive_header_data_t header;
        union {
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
    static ext_dart_remaining_time_t ext_dart_remaining_time;
    static game_robot_state_t robot_state;
    static power_heat_data_t power_heat;
    static game_robot_pos_t robot_pos;
    static buff_musk_t buff_musk;
    static aerial_robot_energy_t aerial_robot_energy;
    static robot_hurt_t robot_hurt;
    static shoot_data_t shoot_data;
    static bullet_remaining_t bullet_remaining;
    static dart_client_t dart_client;

    /*** Send Data ***/
    static robot_interactive_data_t robot_data_send;

    /**
     * Start referee interface
     */
    static void init();

    /**
     * Get current robotic ID
     * @return   game_robot_state.robot_id
     */
    static uint8_t get_self_id();

    static bool add_tx_graphic(const graphic_data_struct_t &graph_data);

    static void set_tx_data_for_deleting_all();

    static void set_tx_data_for_deleting_layer(uint32_t layer);

    static void set_tx_data_for_label(const ext_client_custom_character_t &character_data);

    static void set_tx_data_for_graphics();

    static void send_tx_data();

#if REFEREE_USE_EVENTS

    /**
     * Event source when data received
     */
    static event_source_t data_received_event;

#endif

private:

    static int graphic_buffer_index;
    static graphic_data_struct_t graphic_data_buffer[7];

    static client_custom_data_t client_custom_data;

    /**
     * Send data to client or other robots
     * @param receiver_id   CLIENT or robot ID
     * @param data_cmd_id   Command ID in data section, only available when sending data to other robots
     */
    static void send_data_(receiver_index_t receiver_id);

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
            ext_dart_remaining_time_t ext_dart_remaining_time_;
            game_robot_state_t game_robot_state_;
            power_heat_data_t power_heat_data_;
            game_robot_pos_t game_robot_pos_;
            buff_musk_t buff_musk_;
            aerial_robot_energy_t aerial_robot_energy_;
            robot_hurt_t robot_hurt_;
            shoot_data_t shoot_data_;
            bullet_remaining_t bullet_remaining_;
            dart_client_t dart_client_;

            robot_interactive_data_t robot_interactive_data_;
            client_custom_data_t client_custom_data_;
        };
        uint16_t tail;  // offset is not correct since union takes the maximal size of its elements, do not use directly
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
