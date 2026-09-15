#include "hex_utils.h"

uint8_t hex_utils_nibble_to_val(char ascii) {
    if (ascii >= '0' && ascii <= '9') {
        return ascii - '0';
    } else if (ascii >= 'A' && ascii <= 'F') {
        return ascii - 'A' + 10;
    } else if (ascii >= 'a' && ascii <= 'f') {
        return ascii - 'a' + 10;
    }
    Serial.print("! ");
    Serial.print("0x");
    Serial.print(ascii, HEX);
    Serial.println(" is not a valid hex symbol");
    return 0;
}

char hex_utils_nibble_to_hex(uint8_t nibble) {
    nibble &= 0x0f;
    if (nibble >= 0 && nibble <= 9) {
        return '0' + nibble;
    } else {
        return 'A' + nibble - 10;
    }
}
