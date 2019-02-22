#include "button_monitor.h"

using namespace chibios_rt;

ButtonMonitorThread::ButtonMonitorThread(ioportid_t ioportid, ioportmask_t ioportmask)
        : BaseStaticThread<128>() {
    _ioportid = ioportid;
    _ioportmask = ioportmask;
}

void ButtonMonitorThread::main() {
    // Enable input on the given port
    palSetPadMode(_ioportid, _ioportmask, PAL_MODE_INPUT_PULLUP);
    // Set the name of this thread
    char name[] = "btn_PXXX";
    portToString(_ioportid, _ioportmask, name + 4);
    setName(name);
    // Monitor the button
    while(!shouldTerminate()) {
        while(palReadPad(_ioportid, _ioportmask) != BUTTON_PRESS_PAL_STATUS) {
            // Wait for the button to be pressed
            sleep(TIME_MS2I(5));
        }
        pressed = true;
        toggle = !toggle;
        counter++;
        //if(_function) _function();
        while(palReadPad(_ioportid, _ioportmask) == BUTTON_PRESS_PAL_STATUS) {
            // Wait for the button to be released
            sleep(TIME_MS2I(5));
        }
        pressed = false;
    }
}