//
// Created by Administrator on 2019/1/15 0015.
//

#ifndef META_INFANTRY_JUDGE_SYSTEM_PARSER_H
#define META_INFANTRY_JUDGE_SYSTEM_PARSER_H

#include "ch.hpp"
#include "hal.h"


typedef struct {
    uint16_t _sof;
    uint16_t _data_length;
    uint8_t _seq;
    uint8_t _crc_8;
} FrameHeader_t;

typedef struct {
    uint8_t _flag;
    float _x;
    float _y;
    float _z;
    float _compass;
} LocData_2;

typedef struct {
    uint32_t _remainTime;
    uint16_t _remainLifeValue;
    float _realChassisOutV;
    float _realChassisOutA;
    LocData_2 _locData;
    float _remainPower;
} GameInfo_t;

typedef struct {
    uint8_t _weakId:4;
    uint8_t _way:4;
    uint16_t _value;
} RealBloodChangedData_t;

typedef struct
{
    float _realBulletShootSpeed;
    float _realBulletShootFreq;
    float _realGolfShootSpeed;
    float _realGolfShootFreq;
} RealShootData_t;

typedef struct
{
    float data1;
    float data2;
    float data3;
} ClientData_t;

typedef union {
    float f;
    char c[4];
} ByteFloatUnion_t;

class JudgeSystem {
public:
    bool debug;
    char _client_data_buf[21];
    FrameHeader_t frameHeader;
    uint8_t cmdID;
    GameInfo_t gameInfo;
    RealBloodChangedData_t realBloodChangedData;
    RealShootData_t realShootData;
    ClientData_t clientData;

    static void uart_rx_callback(UARTDriver *uartp);

    int read_buf_info(BaseSequentialStream *chp, const unsigned char* buf, int length);

    char* send_client_info_parser(float data1, float data2, float data3);

private:




    enum judge_system_frame_type_t{  // feedback types
        FRAME_STARTING_BYTE = 0xFF,
        FRAME_HEADER = 0x00,
        FRAME_GAME_INFO = 0x01,   // real time game info
        FRAME_REAL_BLOOD_CHANGE = 0x02,   // real time blood change info
        FRAME_SHOOT_DATA_CHANGE = 0x03,   // real time shooting data
        FRAME_CLIENT_DATA_CHANGE = 0x05   // client defined data
    };

    static judge_system_frame_type_t parse_header(const uint8_t* buf);
    static bool parse_info(judge_system_frame_type_t info_type, const uint8_t* buf);

    static judge_system_frame_type_t rx_waiting_type;
    static uint8_t rx_buf[];
};



unsigned int my_byte_to_int(const unsigned char** buf, int length);

float my_byte_to_float(const unsigned char** buf);

void my_float_to_byte(const float f, char* ptr);



#endif //META_INFANTRY_JUDGE_SYSTEM_PARSER_H
