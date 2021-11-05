//
// Created by Chen Qian on 10/29/21.
//

#include "can_motor_interface.h"

CANInterface *can_motor_interface::can[2];

CANMotorFeedback can_motor_interface::motor_feedback[MOTOR_COUNT];

can_motor_interface::motor_id_t can_motor_interface::mapping_SID2ID[2][11];
can_motor_interface::mapping_ID2SID_t can_motor_interface::mapping_ID2SID[MOTOR_COUNT];

CANTxFrame can_motor_interface::txmsg[2][3];

// Initialize code just run for one time, thus the time complexity is not so critical.
void can_motor_interface::init(CANInterface *can1_, CANInterface *can2_) {
    // Assign can channel.
    can[0] = can1_;
    can[1] = can2_;
    // Initialize the Mapping, the initial value is invalid.
    for (auto &i : mapping_SID2ID) {
        for (auto &j: i) {
            j = MOTOR_COUNT;
        }
    }
    // Creat mapping from ID to SID, vice versa.
    for (int i = 0; i<MOTOR_COUNT; i++) {
        mapping_SID2ID[(int)CANMotorProfile[i].can_channel][CANMotorProfile[i].CAN_SID - 0x201] = (motor_id_t)i;
        mapping_ID2SID[i] = {CANMotorProfile[i].CAN_SID, CANMotorProfile[i].can_channel};
        motor_feedback[i].init(CANMotorProfile[i].motor_type, CANMotorProfile[i].initial_encoder_angle);
    }
    // Initialize the continuous SID field lists. Record the continuous SID fields.
    // Theoretically, the maximum number of SID fields for each can are 6 as there are only 11 SID for CAN motors.
    int can_SID_field_list[6][2];
    int can_SID_field_count = 0;
    // Detect fields' upper and lower bounds and register the callback function.
    for (int can_channel = 0; can_channel < 2; can_channel++) {
        // Reset parameters for each can channel.
        for (auto &i : can_SID_field_list) {
            for (auto &j: i) {
                j = 0;
            }
        }
        can_SID_field_count = 0;
        // Search and Record field for can1.
        for (int i = 0; i < 0x20B-0x200; i++) {
            // Detect for SID lower bound.
            if(i == 0 && mapping_SID2ID[can_channel][i]!=MOTOR_COUNT){                // Boundary condition
                can_SID_field_list[can_SID_field_count][0] = i + 0x201;
            } else if(mapping_SID2ID[can_channel][i]!=MOTOR_COUNT && mapping_SID2ID[can_channel][i-1]==MOTOR_COUNT){
                can_SID_field_list[can_SID_field_count][0] = i + 0x201;
            }
            // Detect for SID upper bound.
            if(i == (0x20B-0x200-1) && mapping_SID2ID[can_channel][i]!=MOTOR_COUNT) { // Boundary condition
                can_SID_field_list[can_SID_field_count][1] = i + 0x201;
                can_SID_field_count++;//Count value need to be used for later.
            } else if(mapping_SID2ID[can_channel][i]!=MOTOR_COUNT && mapping_SID2ID[can_channel][i+1]==MOTOR_COUNT) {
                can_SID_field_list[can_SID_field_count][1] = i + 0x201;
                can_SID_field_count++;//After detect the lower bound, move to next.
            }
        }
        // Arrange the field.
        for (int i = 0; i < can_SID_field_count; i++) {
            int max_len_field[2] = {can_SID_field_list[i][0], can_SID_field_list[i][1]};
            for (int j = i; j < can_SID_field_count; j++) {
                if((can_SID_field_list[j][0]-can_SID_field_list[j][1]) > (max_len_field[0]-max_len_field[1])){
                    max_len_field[0] = can_SID_field_list[j][0];
                    max_len_field[1] = can_SID_field_list[j][1];
                    can_SID_field_list[j][0] = can_SID_field_list[i][0];
                    can_SID_field_list[j][1] = can_SID_field_list[i][1];
                    can_SID_field_list[i][0] = max_len_field[0];
                    can_SID_field_list[i][1] = max_len_field[1];
                }
            }
        }
        // Register the field by length of the field.
        for (int i = 0; i < can_SID_field_count; i++) {
            switch (can_channel) {
                case 0:
                    can[can_channel]->register_callback(can_SID_field_list[i][1], can_SID_field_list[i][0], can1_callback_func);
                    break;
                case 1:
                    can[can_channel]->register_callback(can_SID_field_list[i][1], can_SID_field_list[i][0], can2_callback_func);
                    break;
            }
        }
    }
    // Filling the IDE, RTR, DLC and SID for CANTxFrames.
    for (auto & i : txmsg) {
        for (auto & j: i) {
            j.IDE = CAN_IDE_STD;
            j.RTR = CAN_RTR_DATA;
            j.DLC = 0x08;
        }
        i[0].SID = 0x200;
        i[1].SID = 0x1FF;
        i[2].SID = 0x2FF;
    }
}

void can_motor_interface::can1_callback_func(CANRxFrame const *rxmsg) {
    // As the can callback SID range are performed by program, no need to check the SID.
    motor_feedback[mapping_SID2ID[0][(int)rxmsg->SID - 0x201]].process_feedback(rxmsg);
}

void can_motor_interface::can2_callback_func(CANRxFrame const *rxmsg) {
    // As the can callback SID range are performed by program, no need to check the SID.
    motor_feedback[mapping_SID2ID[1][(int)rxmsg->SID - 0x201]].process_feedback(rxmsg);
}

void can_motor_interface::set_current(motor_id_t motor_id, int target_current) {
    int motor_SID = mapping_ID2SID[motor_id].SID;
    if(motor_SID > 0x200 && motor_SID < 0x205) {
        int start_id = (motor_SID-0x201)*2;
        txmsg[mapping_ID2SID[motor_id].can_channel][0].data8[start_id] = (uint8_t)(target_current >> 8);
        txmsg[mapping_ID2SID[motor_id].can_channel][0].data8[start_id+1] = (uint8_t)target_current;
    } else if (motor_SID < 0x209) {
        int start_id = (motor_SID-0x205)*2;
        txmsg[mapping_ID2SID[motor_id].can_channel][1].data8[start_id] = (uint8_t)(target_current >> 8);
        txmsg[mapping_ID2SID[motor_id].can_channel][1].data8[start_id+1] = (uint8_t)target_current;
    } else if (motor_SID < 0x20B) {
        int start_id = (motor_SID-0x209)*2;
        txmsg[mapping_ID2SID[motor_id].can_channel][2].data8[start_id] = (uint8_t)(target_current >> 8);
        txmsg[mapping_ID2SID[motor_id].can_channel][2].data8[start_id+1] = (uint8_t)target_current;
    } else {
        return;
    }
}

bool can_motor_interface::post_target_current(CANMotorBase::can_channel_t can_channel_, uint32_t SID) {
    if(!can[can_channel_]) return false;
    switch (SID) {
        case 0x200:
            return can[can_channel_]->send_msg(&txmsg[can_channel_][0]);
        case 0x1FF:
            return can[can_channel_]->send_msg(&txmsg[can_channel_][1]);
        case 0x2FF:
            return can[can_channel_]->send_msg(&txmsg[can_channel_][2]);
        default:
            return false;
    }
}
