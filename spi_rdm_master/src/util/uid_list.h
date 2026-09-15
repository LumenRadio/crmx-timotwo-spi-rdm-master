#ifndef UID_LIST_H_
#define UID_LIST_H_

#include <Arduino.h>

/* Define list sizes */
#define UID_LIST_N_RADIOS 20
#define UID_LIST_N_DEVICES 50

/* Shared radio and RDM device lists, defined in spi_rdm_master.ino */
extern uint64_t radios[];
extern uint8_t n_radios;
extern uint64_t rdm_devices[];
extern uint8_t n_devices;

/**
 * Check if a radio is in our list.
 *
 * @param uid   UID of the receiver to look for in list.
 */
bool uid_list_radio_is_in_list(uint64_t uid);

/**
 * Add a receiver to the list.
 *
 * @param uid   UID of the receiver to add.
 */
void uid_list_add_radio_to_list(uint64_t uid);

/**
 * Remove a receiver from the list.
 *
 * @param uid   UID of the receiver to remove from list.
 */
void uid_list_remove_radio_from_list(uint64_t uid);

/**
 * Check if RDM device is in our list.
 *
 * @param uid   UID of the RDM device to look for in list.
 */
bool uid_list_rdm_device_is_in_list(uint64_t uid);

/**
 * Add a RDM device to the list.
 *
 * @param uid   UID of the RDM device to add to list.
 */
void uid_list_add_rdm_device_to_list(uint64_t uid);

/**
 * Remove a RDM device from the list.
 *
 * @param uid   UID of the RDM device to remove from list.
 */
void uid_list_remove_rdm_device_from_list(uint64_t uid);

#endif
