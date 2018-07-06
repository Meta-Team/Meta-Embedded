#include "button_monitor.hpp"

using namespace chibios_rt;

ButtonMonitorThread::ButtonMonitorThread(ioportid_t ioportid, ioportmask_t ioportmask)
        : BaseStaticThread<256>() {
    _ioportid = ioportid;
    _ioportmask = ioportmask;
}

void ButtonMonitorThread::main(void) {
    palSetPadMode(_ioportid, _ioportmask, PAL_MODE_INPUT_PULLUP);
    char name[] = "btn_PXXX";
    portToString(_ioportid, _ioportmask, name + 4);
    setName(name);
    while(true) {
        // The keys are inverted (press -> 0, release -> 1)
        while(palReadPad(_ioportid, _ioportmask)) {
            // Wait for the button to be pressed
            sleep(TIME_MS2I(5));
        }
        pressed = true;
        toggle = !toggle;
        counter++;
        //if(_function) _function();
        while(!palReadPad(_ioportid, _ioportmask)) {
            // Wait for the button to be released
            sleep(TIME_MS2I(5));
        }
        pressed = false;
    }
}