#include "button_monitor.h"

using namespace chibios_rt;

ButtonMonitorThread::ButtonMonitorThread(ioportid_t ioportid, ioportmask_t ioportmask)
        : BaseStaticThread<128>() {
    _ioportid = ioportid;
    _ioportmask = ioportmask;
}

void portToString(ioportid_t ioportid, ioportmask_t ioportmask, char* result) {
    result[0] = 'P';
    if(ioportid == GPIOA) {
        result[1] = 'A';
    } else if(ioportid == GPIOB) {
        result[1] = 'B';
    } else if(ioportid == GPIOC) {
        result[1] = 'C';
    } else if(ioportid == GPIOD) {
        result[1] = 'D';
    } else if(ioportid == GPIOE) {
        result[1] = 'E';
    } else if(ioportid == GPIOF) {
        result[1] = 'F';
    } else if(ioportid == GPIOG) {
        result[1] = 'G';
    } else if(ioportid == GPIOH) {
        result[1] = 'H';
    }
    result[2] = ioportmask > 9 ? '1' : '0';
    result[3] = '0' + (ioportmask % 10);
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