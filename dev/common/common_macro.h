//
// Created by liuzikai on 2018-12-28.
//

#ifndef META_INFANTRY_COMMON_MACRO_H
#define META_INFANTRY_COMMON_MACRO_H


#define ABS_LIMIT(n, limit) { \
    if (n > limit) n = limit; \
    if (n < -(limit)) n = -(limit); \
}

#define VAL_LIMIT(n, max, min) { \
    if (n > max) n = max; \
    if (n < min) n = min; \
}

#define SIGN(n) (n == 0 ? 0 : (n > 0 ? 1 : -1))

#endif //META_INFANTRY_COMMON_MACRO_H
