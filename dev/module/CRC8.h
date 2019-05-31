//
// Created by Administrator on 2019/1/17 0017.
//

#ifndef META_INFANTRY_CRC8_H
#define META_INFANTRY_CRC8_H

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

#endif //META_INFANTRY_CRC8_H
