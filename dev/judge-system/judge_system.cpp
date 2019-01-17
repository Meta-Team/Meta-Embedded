//
// Created by Administrator on 2019/1/15 0015.
//

#include "judge_system.h"
#include "CRC16.h"
#include "CRC8.h"
#include "serial_shell.h"

#define JUDGE_SYSTEM_RX_BUF_SIZE 100

// TODO: Make sure the program works without initial value here
JudgeSystem::judge_system_frame_type_t JudgeSystem::rx_waiting_type = JudgeSystem::FRAME_HEADER;

uint8_t JudgeSystem::rx_buf[JUDGE_SYSTEM_RX_BUF_SIZE];


static ByteFloatUnion_t byteFloatUnion;       //union for float char transfer

static unsigned char pack_buf[100];     //store data in a pack

static bool in_pack_reading = false;    //whether reach the end of a pack, if not in_pack_reading is true

static int remain_data_length;  //remain data length in a pack

static unsigned char* pack_buf_ptr; //pointer to pack buf



void JudgeSystem::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLock();

    if (rx_waiting_type == FRAME_HEADER) {

    }

    chSysUnlock();

    uartStartReceive(uartp, REMOTE_DATA_BUF_SIZE, rx_buf);
}

JudgeSystem::judge_system_frame_type_t JudgeSystem::parse_header(const uint8_t* buf) {


}
bool JudgeSystem::parse_info(JudgeSystem::judge_system_frame_type_t info_type, const uint8_t* buf) {

}

int JudgeSystem::read_buf_info(BaseSequentialStream *chp, const unsigned char *buf, int length) {
    /**
     * @brief:  read buf information, store all information into class judge_system_parser, return 1 if succeed, 0 if fail
     * @param:  chp:
     * @param:  buf: pointer to the input buf
     * @param:  length: length of the buf
     */
    int begin_id = -1;
    int end_id = -1;
    //if a complete data pack is sent, record its begin index
    for (int i = length - 1; i >= 0; i--) {
        //search the buf inversely
        if (buf[i] == 0xA5) {
            if (end_id == -1) {
                end_id = i - 1;
                continue;
            }
            begin_id = i;
            break;
        }
    }
    if (end_id == -1 || begin_id == -1) return 0;	//no new send found, information not change

    //read frame header info
    Shell::printf("Parsing Info..." SHELL_NEWLINE_STR);
    const unsigned char* ptr = &buf[begin_id];
    const unsigned char** byte = &ptr;
    frameHeader._sof = (uint16_t) my_byte_to_int(byte, 1);
    frameHeader._data_length = (uint16_t) my_byte_to_int(byte, 2);
    frameHeader._seq = (uint8_t) my_byte_to_int(byte, 1);
    frameHeader._crc_8 = (uint8_t) my_byte_to_int(byte, 1);

    //check CRC_16
    if (!debug) {
        //check sending errors in frame header
        if (!Verify_CRC8_Check_Sum((uint8_t *) &buf[begin_id], 5)) {
            Shell::printf("Error occurred during sending frame header..." SHELL_NEWLINE_STR);
            return 0;
        }
        //check sending errors in data field
        if (!Verify_CRC16_Check_Sum((uint8_t *) &buf[begin_id], 9 + frameHeader._data_length)) {
            Shell::printf("Error occurred during sending data..." SHELL_NEWLINE_STR);
            return 0;
        }
    }
    //read cmdID
    cmdID = (uint8_t) my_byte_to_int(byte, 2);
    //check whether a whole pack is sent
    if (frameHeader._seq == 0) {
        //the first frame of a pack
        //initialize remain_data_length, point pack_buf_pointer to the beginning of pack_buf
        //包序号也有可能是从1开始的，总之都试试
        if (cmdID == GAME_INFO) remain_data_length = 31;
        if (cmdID == REAL_BLOOD_CHANGE) remain_data_length = 3;
        if (cmdID == SHOOT_DATA_CHANGE) remain_data_length = 16;
        pack_buf_ptr = &pack_buf[0];
    }
    remain_data_length -= frameHeader._data_length;
    in_pack_reading = remain_data_length > 0;   //if still remain data to read
    if (in_pack_reading){
        //if not reach the end of a pack read all data in that frame
        for (int i = 0; i < frameHeader._data_length; i++) {
            *pack_buf_ptr = **byte;
            pack_buf_ptr += 1;
            *byte += 1;
        }
        return 1;
    } else {
        //if reached the end of a pack, read data with length equals to length we need to read
        for (int i = 0; i < frameHeader._data_length + remain_data_length; i++) {
            *pack_buf_ptr = **byte;
            pack_buf_ptr += 1;
            *byte += 1;
        }
    }
    //check to debug
    if (cmdID == GAME_INFO && pack_buf_ptr - &pack_buf[0] != 31) {
        Shell::printf("Error: GAME_INFO required 31 bytes, get %d bytes!" SHELL_NEWLINE_STR, pack_buf_ptr - &pack_buf[0]);
        return 0;
    }
    if (cmdID == REAL_BLOOD_CHANGE && pack_buf_ptr - &pack_buf[0] != 3) {
        Shell::printf("Error: REAL_BLOOD_CHANGE required 3 bytes, get %d bytes!" SHELL_NEWLINE_STR, pack_buf_ptr - &pack_buf[0]);
        return 0;
    }
    if (cmdID == SHOOT_DATA_CHANGE && pack_buf_ptr - &pack_buf[0] != 16) {
        Shell::printf("Error: SHOOT_DATA_CHANGE required 16 bytes, get %d bytes!" SHELL_NEWLINE_STR, pack_buf_ptr - &pack_buf[0]);
        return 0;
    }
    ptr = &pack_buf[0];
    byte = &ptr;
    //reading data...
    switch (cmdID) {
        case GAME_INFO:
            Shell::printf("\tReading Game Info..." SHELL_NEWLINE_STR);
            gameInfo._remainTime = (uint32_t) my_byte_to_int(byte, 4);
            gameInfo._remainLifeValue = (uint16_t) my_byte_to_int(byte, 2);
            gameInfo._realChassisOutV = my_byte_to_float(byte);
            gameInfo._realChassisOutA = my_byte_to_float(byte);
            gameInfo._locData._flag = (uint8_t)my_byte_to_int(byte, 1);
            gameInfo._locData._x = my_byte_to_float(byte);
            gameInfo._locData._y = my_byte_to_float(byte);
            gameInfo._locData._z = my_byte_to_float(byte);
            gameInfo._locData._compass = my_byte_to_float(byte);
            gameInfo._remainPower = my_byte_to_int(byte, 4);
            break;
        case REAL_BLOOD_CHANGE:
            Shell::printf("\tReading Blood Change Info..." SHELL_NEWLINE_STR);
            if (frameHeader._data_length != 3) {
                Shell::printf("Error: required 3 bytes, get %d bytes!" SHELL_NEWLINE_STR,frameHeader._data_length);
                return 0;
            }
            realBloodChangedData._weakId = ((uint8_t)my_byte_to_int(byte, 1)>>4);
            realBloodChangedData._way = (uint8_t) ((uint8_t)my_byte_to_int(byte, 1) & 0x0F);
            realBloodChangedData._value = (uint16_t) my_byte_to_int(byte, 2);
            break;
        case SHOOT_DATA_CHANGE:
            Shell::printf("\tReading Shooting data..." SHELL_NEWLINE_STR);
            if (frameHeader._data_length != 16) {
                Shell::printf("Error: required 16 bytes, get %d bytes!" SHELL_NEWLINE_STR,frameHeader._data_length);
                return 0;
            }
            realShootData._realBulletShootSpeed = my_byte_to_float(byte);
            realShootData._realBulletShootFreq = my_byte_to_float(byte);
            realShootData._realGolfShootSpeed = my_byte_to_float(byte);
            realShootData._realGolfShootFreq = my_byte_to_float(byte);
            break;
        default:
            Shell::printf("Error: Invalid cmdID: %d" SHELL_NEWLINE_STR,cmdID);
            return 0;   //ID read error!
    }
    return 1;
}

char* JudgeSystem::send_client_info_parser(float data1, float data2, float data3) {
    /**
     * @brief   construct char array to send back to the server
     * @param   data{1,2,3}: data to sent back to the sever
     */
     //TODO: in here, I just assume 21 byte can be sent in one frame, if not, give it back to me, I will fix it
    clientData.data1 = data1;
    clientData.data2 = data2;
    clientData.data3 = data3;
    //construct frameHeader
    _client_data_buf[0] = (char) 0xA5;
    _client_data_buf[1] = (uint8_t)((uint16_t) 12 >> 8);
    _client_data_buf[2] = (uint8_t)((uint16_t) 12 & 0xFF);
    _client_data_buf[3] = (uint8_t) 0;
    Append_CRC8_Check_Sum((uint8_t*) _client_data_buf, 5);
    //construct data
    my_float_to_byte(data1, &_client_data_buf[5]);
    my_float_to_byte(data2, &_client_data_buf[9]);
    my_float_to_byte(data3, &_client_data_buf[13]);
    //construct tail
    Append_CRC16_Check_Sum((uint8_t*) _client_data_buf, 21);
    return _client_data_buf;
}

unsigned int my_byte_to_int(const unsigned char** buf, int length) {
    /**
     * @brief   transfer a {length} byte char array into an int with {length} bytes
     * @param   buf: pointer to pointer to the char array
     * @param   length: length of data in byte, (ep. uint8_t is length 1)
     */
    unsigned int value = 0x00;
    int mask = 0xFF;
    for (int i = 0; i < length; i++, mask <<= 8) {
        value |= ((*buf)[length - i - 1] << i*8) & mask;
    }
    *buf += length;
    return value;
}

float my_byte_to_float(const unsigned char** buf) {
    /**
     * @brief   transfer a 4 byte char array into an float
     * @param   buf: pointer to pointer to the char array
     */
    byteFloatUnion.c[0] = (*buf)[0];
    byteFloatUnion.c[1] = (*buf)[1];
    byteFloatUnion.c[2] = (*buf)[2];
    byteFloatUnion.c[3] = (*buf)[3];
    *buf += 4;
    return byteFloatUnion.f;
}

void my_float_to_byte(const float f, char* ptr) {
    /**
     * @brief   transfer a float into a 4 byte char array
     * @param   f: the float
     * @param   buf: pointer to the char array
     */
    byteFloatUnion.f = f;
    ptr[0] = byteFloatUnion.c[0];
    ptr[1] = byteFloatUnion.c[1];
    ptr[2] = byteFloatUnion.c[2];
    ptr[3] = byteFloatUnion.c[3];
}
