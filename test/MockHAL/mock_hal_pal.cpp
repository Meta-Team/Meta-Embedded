//
// Created by liuzikai on 8/18/21.
//

#include "mock_hal_pal.h"
#include <CppUTestExt/MockSupport.h>

void palSetPad(int port, int pad) {
    mock().actualCall("palSetPad").withIntParameter("port", port).withIntParameter("pad", pad);
}

void palClearPad(int port, int pad) {
    mock().actualCall("palClearPad").withIntParameter("port", port).withIntParameter("pad", pad);
}

void palTogglePad(int port, int pad) {
    mock().actualCall("palTogglePad").withIntParameter("port", port).withIntParameter("pad", pad);
}
