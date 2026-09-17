#ifndef HEX_UTILS_H_
#define HEX_UTILS_H_

#include <Arduino.h>

/**
 * Converts a ASCII character as a single HEX digit to integer.
 *
 * Return value:   The value, 0-15.
 *
 * @param ascii    The hex character.
 */
uint8_t hex_utils_nibble_to_val(char ascii);

/**
 * Converts a an integer 0-15 to ASCII character as a single HEX digit.
 *
 * Return value:   The hex digit as ASCII char, '0'-'F'.
 *
 * @param nibble   The integer.
 */
char hex_utils_nibble_to_hex(uint8_t nibble);

#endif
