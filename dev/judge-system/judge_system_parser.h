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
} FrameHeader;

typedef struct {
    uint8_t _flag;
    float _x;
    float _y;
    float _z;
    float _compass;
} LocData;

typedef struct {
    uint32_t _remainTime;
    uint16_t _remainLifeValue;
    float _realChassisOutV;
    float _realChassisOutA;
    LocData _locData;
    float _remainPower;
} GameInfo;

typedef struct {
    uint8_t _weakId:4;
    uint8_t _way:4;
    uint16_t _value;
} RealBloodChangedData;

typedef struct
{
    float _realBulletShootSpeed;
    float _realBulletShootFreq;
    float _realGolfShootSpeed;
    float _realGolfShootFreq;
} RealShootData;

typedef struct
{
    float data1;
    float data2;
    float data3;
} ClientData;

typedef union U_float_byte_t {
    float f;
    char c[4];
} ByteFloatUnion;

class judge_system_parser {
public:
    bool debug;
    char _client_data_buf[21];
    FrameHeader frameHeader;
    uint8_t cmdID;
    GameInfo gameInfo;
    RealBloodChangedData realBloodChangedData;
    RealShootData realShootData;
    ClientData clientData;

    int read_buf_info(BaseSequentialStream *chp, const unsigned char* buf, int length);

    char* send_client_info_parser(float data1, float data2, float data3);
};



unsigned int my_byte_to_int(const unsigned char** buf, int length);

float my_byte_to_float(const unsigned char** buf);

void my_float_to_byte(const float f, char* ptr)



#endif //META_INFANTRY_JUDGE_SYSTEM_PARSER_H
