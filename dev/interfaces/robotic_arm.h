//
// Created by Ye Anbang on 2019/2/2.
//

#ifndef META_INFANTRY_FETCH_BULLET_H
#define META_INFANTRY_FETCH_BULLET_H

#include "ch.hpp"
#include "hal.h"
#include "hal_pal.h"
#include "can_interface.h"
#include "port_to_string.h"

static uint16_t rotate_current = 30;  //TODO change it
static uint16_t release_current = 30;  //TODO change it
static uint16_t inactive_current = 30;  //TODO: change it

class RoboticArm {

public:

    static void process_rotation_motor_feedback(CANRxFrame *rxFrame);

    static void set_initial_angle_as_current();

    enum motor_status_t {
        MOTOR_READY,
        MOTOR_ROTATED
    };

    static motor_status_t get_motor_status();

    static bool motor_act(motor_status_t target_status);


    enum clamp_status_t {
        CLAMP_RELAX,
        CLAMP_CLAMPED
    };

    static clamp_status_t get_clamp_status();

    static void clamp_action(clamp_status_t target_status);

private:
    static clamp_status_t _clamp_status; // local storage

};

class FetchBulletThread : public chibios_rt::BaseStaticThread<128> {
private:
    ioportid_t _ioportid;
    ioportmask_t _ioportmask;

protected:
    void main(void) override;
public:

    typedef enum {
        INACTIVE,   //initial state, ready for holding a box
        MACHINE_HAND_CLAMP, //clamp the machine hand
        ROTATE, //rotate the hand
        RELEASE,    //release all bullets
        RESTORE     //return to inactive state
    } activity_mode;

    uint16_t delay1;
    uint16_t delay2;
    uint16_t delay3;
    CANInterface* can;
    uint16_t mode;
    FetchBulletThread(ioportid_t ioportid, ioportmask_t ioportmask, uint16_t delay_1,
            uint16_t delay_2, uint16_t delay_3, CANInterface *can_interface);
    void fetch_once();
};


#endif //META_INFANTRY_FETCH_BULLET_H
