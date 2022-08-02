//
// Created by liuzikai on 2019-07-13.
//

#ifndef META_INFANTRY_SD_CARD_INTERFACE_H
#define META_INFANTRY_SD_CARD_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

class SDCard {

public:

    static constexpr SDCDriver& SDC_DRIVER = SDCD1;

    static constexpr uint8_t PROTOCOL_VERSION = 1;

    static bool init();

    static bool available();

    enum result_t {
        NOT_AVAILABLE = -1,
        OK = 0,
        IO_ERROR = 1,
        DATA_ERROR = 2,
        DATA_UNAVAILABLE = 3
    };

    static int read_all();

    static int get_data(uint16_t data_id, void *data, size_t n);

    static int write_data(uint16_t data_id, void *data, size_t n);

    static int erase();

private:

    static bool available_;

    static constexpr unsigned MAX_BLOCK_COUNT = 64;

    __PACKED_STRUCT sd_info_t {
        uint8_t protocal_ver;
        uint16_t available_blk;
    };

    static constexpr uint16_t SD_INFO_DATA_ID = 0xCAFE;
    static sd_info_t sd_info;

    __PACKED_STRUCT block_header_t {
        uint16_t data_id;
        uint16_t data_length;
    };

    static __PACKED_STRUCT block_t {
        block_header_t header;
        uint8_t data[MMCSD_BLOCK_SIZE - sizeof(header)];  // make sure buf has size as a block
    } blk[MAX_BLOCK_COUNT];

    static int find_blk(uint16_t data_id);

    static int update_header();

    static uint8_t sd_scratchpad_[512];

    static constexpr SDCConfig SDCCFG = {
            sd_scratchpad_,
            SDC_MODE_4BIT
    };

};


#endif //META_INFANTRY_SD_CARD_INTERFACE_H
