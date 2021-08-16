//
// Created by Administrator on 2019/1/17 0017.
//

#ifndef META_INFANTRY_CRC8_H
#define META_INFANTRY_CRC8_H

#include <cstdint>

uint8_t get_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength);

bool verify_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength);

void append_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#endif //META_INFANTRY_CRC8_H
