#ifndef DEBUG_PRINT_H_
#define DEBUG_PRINT_H_

#include <Arduino.h>

/**
 * This function waits for an RDM response to be indicated.
 *
 * @param uid   The UID to print
 */
void debug_print_uid(uint64_t uid);

/**
 * Prints the response from the module as hex, including the IRQ flags.
 *
 * @param irq_flags  The IRQ flags
 * @param *data      Pointer to the data
 * @param len        Length of data
 */
void debug_print_response(int16_t irq_flags, uint8_t *data, uint32_t len);

/**
 * Prints IRQ flags.
 *
 * @param irq_flags  The IRQ flags
 */
void debug_print_irq_flags(int16_t irq_flags);

#endif
