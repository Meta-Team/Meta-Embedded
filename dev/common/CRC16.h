//
// Created by Administrator on 2019/1/15 0015.
//

#ifndef META_INFANTRY_CRC16_H
#define META_INFANTRY_CRC16_H

#include "ch.hpp"

uint16_t get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

uint32_t verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

void append_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#endif //META_INFANTRY_CRC16_H
