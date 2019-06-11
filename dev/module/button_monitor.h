#ifndef META_INFANTRY_BUTTON_MONITOR_H
#define META_INFANTRY_BUTTON_MONITOR_H

#include "ch.hpp"
#include "hal.h"

#if defined(BOARD_RM_2017) // press to be low
#define BUTTON_PRESS_PAL_STATUS PAL_LOW
#elif defined(BOARD_RM_2018_A) // press to be high
#define BUTTON_PRESS_PAL_STATUS PAL_HIGH 
#endif

class ButtonMonitorThread : public chibios_rt::BaseStaticThread<128> {
private:
    ioportid_t _ioportid;
    ioportmask_t _ioportmask;
protected:
    void main(void) override;
public:
    bool pressed = false;
    bool toggle = false;
    uint32_t counter = 0;
    ButtonMonitorThread(ioportid_t ioportid, ioportmask_t ioportmask);
};

#endif