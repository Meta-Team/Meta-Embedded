//
// Created by liuzikai on 2019-07-13.
//

#include "sd_card_interface.h"
#include "CRC16.h"
#include <string.h>

bool SDCard::available_ = false;
uint8_t SDCard::sd_scratchpad_[512];
constexpr SDCConfig SDCard::SDCCFG;
SDCard::sd_info_t SDCard::sd_info = {0, 0};
SDCard::block_t SDCard::blk[MAX_BLOCK_COUNT];

bool SDCard::init() {
    if (available_) return true;

    sdcStart(&SDC_DRIVER, &SDCCFG);
    if (!blkIsInserted(&SDC_DRIVER)) return false;
    if (sdcConnect(&SDC_DRIVER) == HAL_FAILED) return false;

    available_ = true;
    return true;
}

bool SDCard::available() {
    return available_;
}

int SDCard::read_all() {

    if(!available_) return NOT_AVAILABLE;

    // Read SD header
    if (blkRead(&SDC_DRIVER, 0, (uint8_t *) blk, 1) == HAL_FAILED) return IO_ERROR;

    // Verify SD header
    if (blk[0].header.data_id != SD_INFO_DATA_ID) return DATA_ERROR;
    if (blk[0].header.data_length != sizeof(sd_info)) return DATA_ERROR;
    if (!Verify_CRC16_Check_Sum((uint8_t *) &blk[0], sizeof(block_header_t) + sizeof(sd_info) + 2)) return DATA_ERROR;

    // Check protocol version
    memcpy(&sd_info, blk[0].data, sizeof(sd_info));
    if (sd_info.protocal_ver != PROTOCOL_VERSION) return DATA_ERROR;

    // Read data
    if (sd_info.available_blk != 0) {
        if (blkRead(&SDC_DRIVER, 1, (uint8_t *) &blk[1], sd_info.available_blk)) return IO_ERROR;
    }
    return OK;
}

int SDCard::find_blk(uint16_t data_id) {
    for (unsigned i = 1; i <= sd_info.available_blk; i++) {
        if (blk[i].header.data_id == data_id) return i;
    }
    return -1;
}

int SDCard::get_data(uint16_t data_id, void *data, size_t n) {

    int idx = find_blk(data_id);
    if (idx == -1) return DATA_UNAVAILABLE;

    if (blk[idx].header.data_length != n) return DATA_ERROR;
    if (!Verify_CRC16_Check_Sum((uint8_t *) &blk[idx], sizeof(block_header_t) + n + 2)) return DATA_ERROR;

    memcpy(data, blk[idx].data, n);

    return OK;
}

int SDCard::write_data(uint16_t data_id, void *data, size_t n) {

    if(!available_) return NOT_AVAILABLE;

    int idx = find_blk(data_id);
    if (idx == -1) {
        idx = ++sd_info.available_blk;
        int ret = update_header();
        if (ret != OK) return ret;
    }

    blk[idx].header.data_id = data_id;
    blk[idx].header.data_length = n;
    memcpy(blk[idx].data, data, n);
    Append_CRC16_Check_Sum((uint8_t *) &blk[idx], sizeof(block_header_t) + n + 2);

    if (sdcWrite(&SDC_DRIVER, idx, (uint8_t *) &blk[idx], 1) == HAL_FAILED) return IO_ERROR;
    return OK;
}

int SDCard::update_header() {

    if(!available_) return NOT_AVAILABLE;

    blk[0].header.data_id = SD_INFO_DATA_ID;
    blk[0].header.data_length = sizeof(sd_info);
    memcpy(blk[0].data, &sd_info, sizeof(sd_info));
    Append_CRC16_Check_Sum((uint8_t *) &blk[0], sizeof(block_header_t) + sizeof(sd_info) + 2);

    if (sdcWrite(&SDC_DRIVER, 0, (uint8_t *) &blk[0], 1) == HAL_FAILED) return IO_ERROR;
    return OK;
}

int SDCard::erase() {

    if(!available_) return NOT_AVAILABLE;

    sd_info.protocal_ver = PROTOCOL_VERSION;
    sd_info.available_blk = 0;
    return update_header();
}