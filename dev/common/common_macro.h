//
// Created by liuzikai on 2018-12-28.
//

#ifndef META_INFANTRY_COMMON_MACRO_H
#define META_INFANTRY_COMMON_MACRO_H

#define ABS_CROP(n, limit) do { \
    if (n > limit) n = limit; \
    if (n < -(limit)) n = -(limit); \
} while(0)

#define VAL_CROP(n, max, min) do { \
    if (n > max) n = max; \
    if (n < min) n = min; \
} while(0)

#define ABS_IN_RANGE(n, abs_limit) ((n) >= -(abs_limit) && (n) <= (abs_limit))

#define SIGN(n) (n == 0 ? 0 : (n > 0 ? 1 : -1))

#define SYSTIME (TIME_I2MS(chVTGetSystemTimeX()))
// X indicates that this function can be call in all conditions (Thread, ISR, and their critical sections)

#endif //META_INFANTRY_COMMON_MACRO_H
