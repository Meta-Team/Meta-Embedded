//
// Created by Tianyi Han on 2/21/2023.
//

#ifndef META_EMBEDDED_HARDWARE_CONF_H
#define META_EMBEDDED_HARDWARE_CONF_H

#if !defined(ENABLE_VISION) || defined(__DOXYGEN__)
#define ENABLE_VISION                   FALSE
#endif

#if !defined(ENABLE_REFEREE) || defined(__DOXYGEN__)
#define ENABLE_REFEREE                  FALSE
#endif

#if !defined(ENABLE_SUBPITCH) || defined(__DOXYGEN__)
#define ENABLE_SUBPITCH                 FALSE
#endif

#if !defined(ENABLE_CAPACITOR) || defined(__DOXYGEN__)
#define ENABLE_CAPACITOR                FALSE
#endif

#if !defined(ENABLE_USB_SHELL) || defined(__DOXYGEN__)
#define ENABLE_USB_SHELL                TRUE
#endif

#endif //META_EMBEDDED_HARDWARE_CONF_H
