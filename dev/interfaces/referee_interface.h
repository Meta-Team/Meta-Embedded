//
// Created by Administrator on 2019/1/15 0015.
//

#ifndef META_INFANTRY_REFEREE_INTERFACE_H
#define META_INFANTRY_REFEREE_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

/**
 * @brief the interface for referee system
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
class RefereeSystem {

public:

    typedef __PACKED_STRUCT {
        uint8_t flag;
        float x;
        float y;
        float z;
        float compass;
    } LocData_t;  // location data

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
        float realGolfShootSpeed;  // [m/s]
        float realGolfShootFreq;  // [number per second]
    } RealShootData_t;

    typedef __PACKED_STRUCT {
        float data1;
        float data2;
        float data3;
    } ClientData_t;

    /* Data from referee system */
    static GameInfo_t gameInfo;
    static RealBloodChangedData_t realBloodChangedData;
    static RealShootData_t realShootData;

    /**
     * @brief send custom data (three float)
     * @param data
     */
    static void sendClientData(ClientData_t data);

    /**
     * @brief start UART and receive status machine
     */
    static void start();

    static void uartRxCallback(UARTDriver *uartp);  // only for internal use


private:

    typedef __PACKED_STRUCT {
        uint8_t sof;  // start byte of header, 0xA5
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    } FrameHeader_t;

    enum UartWaitingType_t {
        FRAME_STARTING_BYTE,  // receive bytes one by one, waiting for 0xA5
        FRAME_REMAINING_HEADER,  // receive remaining header after SOF
        FRAME_CMD_ID,  // receive 2-byte cmdID
        FRAME_DATA_AND_TAILING  // receive data section and 2-byte CRC16 tailing
    };

    enum CmdID_t {
        CMD_GAME_INFO = 0x01,   // real time game info
        CMD_REAL_BLOOD_CHANGE = 0x02,   // real time blood change info
        CMD_SHOOT_DATA = 0x03,   // real time shooting data
        CMD_CLIENT_DATA_CHANGE = 0x05,   // client defined data
    };

    static const size_t data_length[];  // a const array of length of data section using cmdID as index

    static FrameHeader_t frameHeader;
    static uint16_t cmdID;  // CmdID_t
    
    static UartWaitingType_t uartWaitingType;

    static constexpr size_t rx_buf_size = 100;
    static uint8_t rx_buf[rx_buf_size];

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

    // See cpp file for configs
    static constexpr UARTDriver* uartDriver = &UARTD3;
    static const UARTConfig uartConfig;
};


#endif //META_INFANTRY_REFEREE_INTERFACE_H
