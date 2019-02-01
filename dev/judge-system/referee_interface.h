//
// Created by Administrator on 2019/1/15 0015.
//

#ifndef META_INFANTRY_JUDGE_SYSTEM_PARSER_H
#define META_INFANTRY_JUDGE_SYSTEM_PARSER_H

#include "ch.hpp"
#include "hal.h"


class RefereeSystem {
public:


    typedef __PACKED_STRUCT {
        uint8_t flag;
        float x;
        float y;
        float z;
        float compass;
    } LocData_t;

    typedef __PACKED_STRUCT {
        uint32_t remainTime;  // remain time since last 3 minutes [s]
        uint16_t remainLifeValue;
        float realChassisOutV;  // instant output voltage of chassis [V]
        float realChassisOutA;  // instant output current of chassis [A]
        LocData_t locData;
        float remainPower;
    } GameInfo_t;

    enum HitArmorID_t {
        ARMOR_FRONT = 0x00,
        ARMOR_LEFT = 0x01,
        ARMOR_BACK = 0x02,
        ARMOR_RIGHT = 0x03,
        ARMOR_UP_ONE = 0x04,
        ARMOR_UP_TWO = 0x05
    };

    enum bloodChangedWay_t {
        ARMOR_HIT = 0x0,
        BULLET_OVER_SPEED = 0x1,
        BULLET_OVER_FREQ = 0x2,
        OVER_POWER = 0x3,
        MODULE_OFFLINE = 0x4,
        BREAK_RULE = 0x6,
        RECOVER_AT_AIRPORT = 0xA,
        RECOVER_OF_ENGINEER = 0xB
    };

    typedef __PACKED_STRUCT {
        uint8_t hitArmorID:4;  // when way == ARMOR_HIT, hitArmorID is the id of hit armor
        uint8_t way:4;  // bloodChangedWay_t
        uint16_t value;
    } RealBloodChangedData_t;

    typedef __PACKED_STRUCT {
        float realBulletShootSpeed;  // [m/s]
        float realBulletShootFreq;  // [number per second]
        float _realGolfShootSpeed;  // [m/s]
        float _realGolfShootFreq;  // [number per second]
    } RealShootData_t;

    typedef __PACKED_STRUCT {
        float data1;
        float data2;
        float data3;
    } ClientData_t;


    static GameInfo_t gameInfo;
    static RealBloodChangedData_t realBloodChangedData;
    static RealShootData_t realShootData;

    static void sendClientData(ClientData_t data);

    static void uartStart();

    static void uartRxCallback(UARTDriver *uartp);


private:

    typedef __PACKED_STRUCT {
        uint8_t sof;  // start byte of header, 0xA5
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    } FrameHeader_t;

    enum UartWaitingType_t {
        FRAME_STARTING_BYTE,
        FRAME_REMAINING_HEADER,
        FRAME_CMD_ID,
        FRAME_DATA_AND_TAILING
    };

    enum CmdID_t {
        CMD_GAME_INFO = 0x01,   // real time game info
        CMD_REAL_BLOOD_CHANGE = 0x02,   // real time blood change info
        CMD_SHOOT_DATA = 0x03,   // real time shooting data
        CMD_CLIENT_DATA_CHANGE = 0x05,   // client defined data
    };

    static constexpr size_t data_length[] = {
            0, // 0x00
            31, // 0x01, real time game info
            3, // 0x02, real time blood change info
            16, // 0x03, real time shooting data
            0, // 0x04
            12 // 0x05, client defined data

    };

    static FrameHeader_t frameHeader;
    static uint16_t cmdID;  // CmdID_t
    
    static UartWaitingType_t uartWaitingType;

    static uint8_t rx_buf[];

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

};


#endif //META_INFANTRY_JUDGE_SYSTEM_PARSER_H
