//
// Created by kerui on 2021/7/11.
//

#ifndef META_INFANTRY_MOTOR_INTERFACE_H
#define META_INFANTRY_MOTOR_INTERFACE_H

class MotorIFBase {
public:
    enum motor_can_channel_t {
        none_can_channel,
        can_channel_1,
        can_channel_2
    };

    struct motor_can_config_t {
        motor_can_channel_t motor_can_channel;
        unsigned motor_can_id;
        CANInterface::motor_type_t motor_type;
        float deceleration_ratio;
    };
};

#endif //META_INFANTRY_MOTOR_INTERFACE_H
