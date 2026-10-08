#ifndef SERIAL_H_
#define SERIAL_H_

#include <Arduino.h>

/**
 * Starts the serial console at the given baud rate and clears any garbage
 * bytes left in the input buffer. Call once from setup().
 *
 * @param baud   The baud rate to use.
 */
void serial_init(unsigned long baud);

/* Output */

/**
 * Blocks until all buffered output has been transmitted.
 */
void serial_flush();

/**
 * Prints a value, forwarding to the matching Serial.print overload.
 *
 * @param value   The value to print.
 */
template <typename T> void serial_print(T value) { Serial.print(value); }

/**
 * Prints a value in the given base (e.g. HEX), forwarding to Serial.print.
 *
 * @param value   The value to print.
 * @param base    The base to print it in.
 */
template <typename T> void serial_print(T value, int base) {
    Serial.print(value, base);
}

/**
 * Prints a blank line.
 */
void serial_println();

/**
 * Prints a value followed by a newline, forwarding to Serial.println.
 *
 * @param value   The value to print.
 */
template <typename T> void serial_println(T value) { Serial.println(value); }

/**
 * Prints a UID as 12 hex digits.
 *
 * @param uid   The UID to print.
 */
void serial_print_uid(uint64_t uid);

/**
 * Prints IRQ flags as 8 binary digits.
 *
 * @param irq_flags  The IRQ flags.
 */
void serial_print_irq_flags(int16_t irq_flags);

/**
 * Prints the response from the module as hex, including the IRQ flags.
 *
 * @param irq_flags  The IRQ flags
 * @param *data      Pointer to the data
 * @param len        Length of data
 */
void serial_print_response(int16_t irq_flags, uint8_t *data, uint32_t len);

/* Input */

/**
 * Sets how long serial_parse_int waits for input before giving up.
 *
 * @param timeout   The timeout in milliseconds.
 */
void serial_set_timeout(unsigned long timeout);

/**
 * Blocks until an integer has been entered on the console, or the timeout
 * set by serial_set_timeout elapses.
 *
 * Return value:   The parsed integer.
 */
long serial_parse_int();

#endif
