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

class FetchBulletThread : public chibios_rt::BaseStaticThread<128> {
private:
    ioportid_t _ioportid;
    ioportmask_t _ioportmask;

protected:
    void main(void) override;
public:

    typedef enum {
        INACTIVE,
        MACHINE_HAND_CLAMP,
        ROTATE,
        RELEASE,
        RESTORE
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
