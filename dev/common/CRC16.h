//
// Created by Administrator on 2019/1/15 0015.
//

#ifndef META_INFANTRY_CRC16_H
#define META_INFANTRY_CRC16_H

#include "ch.hpp"

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif //META_INFANTRY_CRC16_H
