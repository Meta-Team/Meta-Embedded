//
// Created by Administrator on 2019/1/17 0017.
//

#ifndef META_INFANTRY_CRC8_H
#define META_INFANTRY_CRC8_H

unsigned char get_crc8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

unsigned int verify_crc8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

void append_crc8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

#endif //META_INFANTRY_CRC8_H
