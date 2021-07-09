//
// Created by liuzikai on 2019-05-13.
//

#ifndef META_INFANTRY_IST8310_REG_H
#define META_INFANTRY_IST8310_REG_H

/** IST8310 Register Maps **/
#define IST8310_STAT1      0x02
#define IST8310_XOUT_L     0x03
#define IST8310_XOUT_H     0x04
#define IST8310_YOUT_L     0x05
#define IST8310_YOUT_H     0x06
#define IST8310_ZOUT_L     0x07
#define IST8310_ZOUT_H     0x08
#define IST8310_STAT2      0x09
#define IST8310_CTRL1      0x0A
#define IST8310_CTRL2      0x0B
#define IST8310_TEMP_OUT_L 0x1C
#define IST8310_TEMP_OUT_H 0x1D
#define IST8310_AVGCNTL    0x41
#define IST8310_PDCTNL     0x42

/** IST8310_STAT1 **/
#define IST8310_STAT1_DOR  0x02
#define IST8310_STAT1_DRDY 0x01

/** IST8310_CTRL1 **/
#define IST8310_CONTINUOUS_200HZ 0x0B
/* IST8310 datasheet only shows single-measurement mode, while IST8303 datasheet shows continuous mode.
 * It seems(?) to work on IST8310... */

/** IST8310_CTRL2 **/
#define IST8310_CTRL2_DREN 0x08
#define IST8310_CTRL2_DRP  0x04
#define IST8310_CTRL2_SRST 0x01

/** Unknown **/
#define IST8310_SINGLE_MEASUREMENT    0
#define IST8310_SAMPLE_RATE_1_2HZ   255

/** IST8310_AVGCNTL **/
#define IST8310_X_AND_Z_NO_AVG        0
#define IST8310_X_AND_Z_AVG_2_TIMES   1
#define IST8310_X_AND_Z_AVG_4_TIMES   2          /* default */
#define IST8310_X_AND_Z_AVG_8_TIMES   3
#define IST8310_X_AND_Z_AVG_16_TIMES  4
#define IST8310_Y_NO_AVG              (0 << 3U)
#define IST8310_Y_AVG_2_TIMES         (1 << 3U)
#define IST8310_Y_AVG_4_TIMES         (2 << 3U)  /* default */
#define IST8310_Y_AVG_8_TIMES         (3 << 3U)
#define IST8310_Y_AVG_16_TIMES        (4 << 3U)

/** IST8310_PDCTNL **/
#define IST8310_PULSE_DURATION_LONG    0x40
#define IST8310_PULSE_DURATION_NORMAL  0xC0

#define IST8310_IIC_ADDRESS 0x0E
#define IST8310_IIC_READ_MSB 0x80

#define IST8310_PSC 0.3f  // Resolution: 0.3 uT/LSB (Least Significant Bit), p.11 on datasheet

#endif //META_INFANTRY_IST8310_REG_H
