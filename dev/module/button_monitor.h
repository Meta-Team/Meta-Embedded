#ifndef META_INFANTRY_BUTTON_MONITOR_H
#define META_INFANTRY_BUTTON_MONITOR_H

#include "ch.hpp"
#include "hal.h"
#include "port_to_string.h"

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