#ifndef _PORT_TO_STRING_HPP_
#define _PORT_TO_STRING_HPP_

#include "ch.hpp"
#include "hal.h"

/**
 * Translates ioportid & ioportmask, and writes the 4 char result
 * to a given pointer of char location (usually a string).
 */
void portToString(ioportid_t ioportid, ioportmask_t ioportmask, char* result);

#endif