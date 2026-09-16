#include "serial.h"
#include "../util/hex_utils.h"

void serial_init(unsigned long baud) {
    Serial.begin(baud);

    /* Clear the Serial port from any garbage bytes */
    while (Serial.available() > 0) {
        Serial.read();
    }
}

/* Output */

void serial_flush() { Serial.flush(); }

void serial_println() { Serial.println(); }

void serial_print_uid(uint64_t uid) {
    for (int i = 11; i >= 0; i--) {
        serial_print(hex_utils_nibble_to_hex(uid >> (4 * i)));
    }
    serial_flush();
}

void serial_print_irq_flags(int16_t irq_flags) {
    for (int i = 7; i >= 0; i--) {
        if (irq_flags & (1 << i)) {
            serial_print('1');
        } else {
            serial_print('0');
        }
    }
    serial_flush();
}

void serial_print_response(int16_t irq_flags, uint8_t *data, uint32_t len) {
    if (irq_flags < 0) {
        serial_println("! Timeout");
        return;
    }
    serial_print("< ");
    serial_print_irq_flags(irq_flags);
    serial_print(" ");
    serial_flush();
    for (uint32_t i = 0; i < len; i++) {
        serial_print(hex_utils_nibble_to_hex(0x0f & (data[i] >> 4)));
        serial_print(hex_utils_nibble_to_hex(0x0f & data[i]));
        serial_print(" ");
        serial_flush();
    }
    serial_println();
}

/* Input */

void serial_set_timeout(unsigned long timeout) { Serial.setTimeout(timeout); }

long serial_parse_int() { return Serial.parseInt(); }
