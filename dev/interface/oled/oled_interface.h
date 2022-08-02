//
// Created by Quoke on 3/24/2022.
//

#ifndef META_INFANTRY_OLED_INTERFACE_H
#define META_INFANTRY_OLED_INTERFACE_H

#include "ch.h"
#include "hal.h"
#include "common_macro.h"

#if defined(BOARD_RM_2018_A)
#define OLED_SPI_DRIVER SPID1
#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01
#else
#error "oledIF is only for RMboard 2018!"
#endif

class oledIF {
public:
    typedef enum
    {
        Pen_Clear = 0x00,
        Pen_Write = 0x01,
        Pen_Inversion = 0x02,
    }Pen_Typedef;

    typedef enum{
        GPIO_PIN_RESET = 0,
        GPIO_PIN_SET
    } GPIO_PinState;

    static void init();

    static void oled_reset_clear();
    static void oled_reset_set();
    static void oled_cmd_clear();
    static void oled_cmd_set();
    static void oled_clear(oledIF::Pen_Typedef pen);
    static void oled_refresh_gram();

    static void oled_write_byte(uint8_t data, uint8_t cmd);
    static void oled_set_pos(uint8_t x, uint8_t y);
    static void oled_drawpoint(int8_t x, int8_t y, oledIF::Pen_Typedef pen);
    static void oled_LOGO();
    static uint8_t OLED_GRAM[130][8];

    static const unsigned char LOGO_BMP[128][8];

private:
    static SPIConfig SPI1_cfg;
};


#endif //META_INFANTRY_OLED_INTERFACE_H
