#include "port_to_string.hpp"

/**
 * Translates ioportid & ioportmask, and writes the 4 char result
 * to a given pointer of char location (usually a string).
 */
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