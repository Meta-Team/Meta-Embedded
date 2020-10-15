//
// Created by 18767 on 2020/1/20.
//

#ifndef META_INFANTRY_ROBOTIC_ARM_LOGIC_H
#define META_INFANTRY_ROBOTIC_ARM_LOGIC_H

#include"new_robotic_arm_skd.h"
#include"engineer_rescue_skd.h"
#include "ch.hpp"
#include "hal.h"

class RoboticArmLG{

public :

    enum grab_state_t{
        GRAB,
        FINISHED
    };

    struct target_box_t{
        int id;
        int x;
        int y;
        struct target_box_t *next;
        struct target_box_t *prev;
    };
    struct action_plan_t{
        int num;    //the number of boxes as target in the box
        struct target_box_t * head;
    };
    struct command_info_t{
        bool slide_y;
        bool clamp;
        int slide_x;
        NewRoboticArmSkd::RA_motor_instruction command;
        struct command_info_t * next;
        struct command_info_t * prev;
    };
    struct command_list_t{
        int num;    //the number of commands in one list
        struct command_info_t * head;
    };
    typedef struct target_box_t target_box_t;
    typedef struct action_plan_t action_plan_t;
    typedef struct command_list_t command_list_t;
    typedef struct command_info_t command_info_t;

    class RoboticArmLGThread:  public chibios_rt::BaseStaticThread<256>{
        void main() final;
    };
    static RoboticArmLGThread roboticArmLgThread;

    static void init();

    //I dont't think this function is necessary
    //static void get_prepared();

    static void start_grabbing();

    static void finish();

    static void auto_logic_enable(bool enable);

    static void using_plan(int val);

    static void generate_plans();

    static void generate_command_list();

    static void update_command_list();

    static void release_memory(command_list_t cl, action_plan_t * ap, int num_plans);

    static void next_step();

    static void previous_step();

    static void next_bin();

    static void previous_bin();

private :
    static bool auto_logic_enabled;
    static bool grabbing;
    static int plan;
    static grab_state_t grabState;
    static action_plan_t actionPlan[2];
    static target_box_t * targetBox;      //current taarget
    static command_info_t * commandInfo;  //current command
    static command_list_t commandList;
};


#endif //META_INFANTRY_ROBOTIC_ARM_LOGIC_H
