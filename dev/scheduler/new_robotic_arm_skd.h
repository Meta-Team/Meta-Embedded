//
// Created by Kerui Zhu on 2/17/2020.
//

#ifndef META_INFANTRY_NEW_ROBOTIC_ARM_SKD_H
#define META_INFANTRY_NEW_ROBOTIC_ARM_SKD_H

enum robotic_arm_motor_state{
    RETRIEVED,
    RETRIEVING,
    STRECHING,
    STRECHED
};

enum robotic_arm_motor_instruction{
    STRECH,
    RETRIEVE
};


class new_robotic_arm_skd {

public:

    static robotic_arm_motor_state get_motor_state();
    static void set_motor_instruction(robotic_arm_motor_instruction command);
    static bool is_extended();
    static void set_extension(bool extend);
    static bool is_raised();
    static void set_raise(bool raise);

};


#endif //META_INFANTRY_NEW_ROBOTIC_ARM_SKD_H
