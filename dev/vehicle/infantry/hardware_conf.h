//
// Created by 钱晨 on 3/16/22.
// This file contains some feature enabling masks.
//

#ifndef META_INFANTRY_HARDWARE_CONF_H
#define META_INFANTRY_HARDWARE_CONF_H

#if !defined(ENABLE_VISION) || defined(__DOXYGEN__)
#define ENABLE_VISION                   TRUE
#endif

#if !defined(ENABLE_REFEREE) || defined(__DOXYGEN__)
#define ENABLE_REFEREE                  TRUE
#endif

#if !defined(ENABLE_SUBPITCH) || defined(__DOXYGEN__)
#define ENABLE_SUBPITCH                 FALSE
#endif

#if !defined(ENABLE_CAPACITOR) || defined(__DOXYGEN__)
#define ENABLE_CAPACITOR                TRUE
#endif

#endif //META_INFANTRY_HARDWARE_CONF_H
