//
// Created by Administrator on 2019/1/15 0015.
//

#include "referee_interface.h"
#include "CRC16.h"
#include "CRC8.h"
#include "serial_shell.h"
#include "memstreams.h"
#include "string.h"

RefereeSystem::FrameHeader_t RefereeSystem::frameHeader;
uint16_t RefereeSystem::cmdID;

RefereeSystem::GameInfo_t RefereeSystem::gameInfo;
RefereeSystem::RealBloodChangedData_t RefereeSystem::realBloodChangedData;
RefereeSystem::RealShootData_t RefereeSystem::realShootData;

const size_t RefereeSystem::data_length[] = {
        0, // 0x00
        31, // 0x01, real time game info
        3, // 0x02, real time blood change info
        16, // 0x03, real time shooting data
        0, // 0x04
        12 // 0x05, client defined data

};

RefereeSystem::UartWaitingType_t RefereeSystem::uartWaitingType;
uint8_t RefereeSystem::rx_buf[RefereeSystem::rx_buf_size];

const UARTConfig RefereeSystem::uartConfig = {
        nullptr,
        nullptr,
        RefereeSystem::uartRxCallback, // callback function when the buffer is filled
        nullptr,
        nullptr,
        115200, // speed
        0,
        0,
        0,
};

// See comments before class definition to see the status machine
void RefereeSystem::uartRxCallback(UARTDriver *uartp) {
    (void) uartp;

    // Handle received data and transfer status properly
    switch (uartWaitingType) {
        case FRAME_STARTING_BYTE:
            if (frameHeader.sof == 0xA5) {
                uartWaitingType = FRAME_REMAINING_HEADER;
            } // else, keep waiting for SOF
            break;
        case FRAME_REMAINING_HEADER:
            if (Verify_CRC8_Check_Sum((uint8_t *) &frameHeader, 5)) {
                uartWaitingType = FRAME_CMD_ID;
            } else {
                SHELL_PRINTF("Invalid frameHeader!" SHELL_NEWLINE_STR);
                uartWaitingType = FRAME_STARTING_BYTE;
            }
            break;
        case FRAME_CMD_ID:
            switch (cmdID) {
                case CMD_GAME_INFO:
                case CMD_REAL_BLOOD_CHANGE:
                case CMD_SHOOT_DATA:
                    if (frameHeader.data_length != data_length[cmdID]) {
                        SHELL_PRINTF("Invalid data length %u for type %u!" SHELL_NEWLINE_STR, frameHeader.data_length, cmdID);
                    }
                    break;
                default:
                    SHELL_PRINTF("Invalid cmdID %u!" SHELL_NEWLINE_STR, cmdID);
            }
            // No matter there is error or not, receive the entire data, since frameHeader.data_length is more creditable
            uartWaitingType = FRAME_DATA_AND_TAILING;
            break;
        case FRAME_DATA_AND_TAILING:
            if (Verify_CRC16_Check_Sum((uint8_t *) &rx_buf, frameHeader.data_length + 2)) {
                switch (cmdID) {
                    case CMD_GAME_INFO:
                        memcpy(&gameInfo, rx_buf, data_length[CMD_GAME_INFO]);
                        break;
                    case CMD_REAL_BLOOD_CHANGE:
                        memcpy(&realBloodChangedData, rx_buf, data_length[CMD_REAL_BLOOD_CHANGE]);
                        break;
                    case CMD_SHOOT_DATA:
                        memcpy(&realShootData, rx_buf, data_length[CMD_SHOOT_DATA]);
                        break;
                    default:
                        // No more error message here since it has been reported at FRAME_CMD_ID
                        break;
                }
            } else {
                SHELL_PRINTF("Invalid data!" SHELL_NEWLINE_STR);
            }
            uartWaitingType = FRAME_STARTING_BYTE;
            break;
    }

    switch (uartWaitingType) {
        case FRAME_STARTING_BYTE:
            uartStartReceive(uartp, 1, &frameHeader.sof);
            break;
        case FRAME_REMAINING_HEADER:
            uartStartReceive(uartp, 4, &frameHeader.data_length);
            break;
        case FRAME_CMD_ID:
            uartStartReceive(uartp, 2, &cmdID);
            break;
        case FRAME_DATA_AND_TAILING:
            uartStartReceive(uartp, frameHeader.data_length + 2, rx_buf); // receive data and CRC16 tailing
            break;
    }
}

void RefereeSystem::start() {
    // GPIOs have been set in board.h

    // Start uart driver
    uartStart(uartDriver, &uartConfig);

    // Wait for starting byte
    uartWaitingType = FRAME_STARTING_BYTE;
    uartStartReceive(uartDriver, 1, &frameHeader.sof);
}

void RefereeSystem::sendClientData(RefereeSystem::ClientData_t data) {
    uartStartSend(uartDriver, sizeof(data), &data);
}