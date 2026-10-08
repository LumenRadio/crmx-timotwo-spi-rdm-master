#ifndef RDM_COMMANDS_H_
#define RDM_COMMANDS_H_

#include <Arduino.h>

/**
 * Prints the MANUFACTURER_LABEL of an RDM device.
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 */
void rdm_commands_print_manufacturer_label(uint64_t rx, uint64_t uid);

/**
 * Prints the DEVICE_MODEL_DESCRIPTION of an RDM device.
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 */
void rdm_commands_print_device_model_description(uint64_t rx, uint64_t uid);

/**
 * Sends IDENTIFY_DEVICE to an RDM device.
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 * @param state   Identify state - true = on, false = off
 */
void rdm_commands_identify(uint64_t rx, uint64_t uid, bool state);

#endif
