#include "debug_print.h"

#include "hex_utils.h"

void debug_print_uid(uint64_t uid) {
    for (int i = 11; i >= 0; i--) {
        Serial.print(hex_utils_nibble_to_hex(uid >> (4 * i)));
    }
    Serial.flush();
}

void debug_print_response(int16_t irq_flags, uint8_t *data, uint32_t len) {
    if (irq_flags < 0) {
        Serial.println("! Timeout");
        return;
    }
    Serial.print("< ");
    debug_print_irq_flags(irq_flags);
    Serial.print(" ");
    Serial.flush();
    for (uint32_t i = 0; i < len; i++) {
        Serial.print(hex_utils_nibble_to_hex(0x0f & (data[i] >> 4)));
        Serial.print(hex_utils_nibble_to_hex(0x0f & data[i]));
        Serial.print(" ");
        Serial.flush();
    }
    Serial.println();
}

void debug_print_irq_flags(int16_t irq_flags) {
    for (int i = 7; i >= 0; i--) {
        if (irq_flags & (1 << i)) {
            Serial.print('1');
        } else {
            Serial.print('0');
        }
    }
    Serial.flush();
}
