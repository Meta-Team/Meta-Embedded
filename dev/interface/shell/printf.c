//
// Created by liuzikai on 2021-07-14.
//

#include "hal.h"
#include "printf.h"
#include "memstreams.h"

#define MAX_FILLER 11
#define FLOAT_PRECISION 9

static char *long_to_string_with_divisor(char *p,
                                         long num,
                                         unsigned radix,
                                         long divisor) {
    int i;
    char *q;
    long l, ll;

    l = num;
    if (divisor == 0) {
        ll = num;
    } else {
        ll = divisor;
    }

    q = p + MAX_FILLER;
    do {
        i = (int)(l % radix);
        i += '0';
        if (i > '9')
            i += 'A' - '0' - 10;
        *--q = i;
        l /= radix;
    } while ((ll /= radix) != 0);

    i = (int)(p + MAX_FILLER - q);
    do
        *p++ = *q++;
    while (--i);

    return p;
}

static char *ch_ltoa(char *p, long num, unsigned radix) {

    return long_to_string_with_divisor(p, num, radix, 0);
}

#if CHPRINTF_USE_FLOAT
static const long pow10[FLOAT_PRECISION] = {
        10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};

static char *ftoa(char *p, double num, unsigned long precision) {
    long l;

    if ((precision == 0) || (precision > FLOAT_PRECISION))
        precision = FLOAT_PRECISION;
    precision = pow10[precision - 1];

    l = (long)num;
    p = long_to_string_with_divisor(p, l, 10, 0);
    *p++ = '.';
    l = (long)((num - l) * precision);
    return long_to_string_with_divisor(p, l, 10, precision / 10);
}
#endif

// Original printf from ChibiOS
#include "chprintf_core.c"

/// Revised printf as I-Class function

// Rename functions
#define chvprintf chvprintfI
#define chprintf chprintfI
#define chsnprintf chsnprintfI

// Replace stream functions

// Original streamPut (os/hal/include/hal_streams.h:137): ((ip)->vmt->put(ip, b))
// Original Serial's vmt (os/hal/src/hal_serial.c:117): static const struct SerialDriverVMT vmt = { ... _put ...};
// Original Serial's _put (os/hal/src/hal_serial.c:62): oqPutTimeout(&((SerialDriver *)ip)->oqueue, b, TIME_INFINITE);
#undef streamPut
#define streamPut(ip, b) (oqPutI(&((SerialDriver *)ip)->oqueue, b))

// Similarly
#undef streamWrite
#define streamWrite(ip, bp, n) (oqWriteI(&((SerialDriver *)ip)->oqueue, bp, n))
#undef streamRead
#define streamRead(ip, bp, n) (iqReadI(&((SerialDriver *)ip)->iqueue, bp, n))
#undef streamGet
#define streamGet(ip) (iqGetI(&((SerialDriver *)ip)->iqueue))

// Updated chprintf
#include "chprintf_core.c"

#undef chvprintf
#undef chprintf
#undef chsnprintf