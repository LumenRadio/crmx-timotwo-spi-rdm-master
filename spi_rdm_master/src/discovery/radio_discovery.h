#ifndef RADIO_DISCOVERY_H_
#define RADIO_DISCOVERY_H_

#include <Arduino.h>

#include "../util/uid_list.h"
#include "discovery_types.h"

/**
 * Issue a radio DISCOVERY UNIQUE BRANCH.
 *
 * Return values:
 *    DiscoveryNone         Empty response
 *    DiscoveryCollission   Collission
 *    DiscoveryUid          Single UID response
 *
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 * @param *uid    Pointer to where to store the UID to in case of a signel
 * response
 */
DiscoveryResponseType radio_discovery_dub(uint64_t lower, uint64_t upper,
                                          uint64_t *uid);

/**
 * Try to mute a radio.
 *
 * Return values:
 *    false         Mute failed
 *    true          Radio muted
 *
 * @param uid   The UID of the radio to mute
 */
bool radio_discovery_mute(uint64_t uid);

/**
 * Unmute all radios.
 */
void radio_discovery_unmute_all(void);

/**
 * Performes a radio discovery on a part of the binary search tree.
 *
 * Return value:  The number of radios found in this part of the tree.
 *
 * @param radios  The list of known radios, updated as new ones are found.
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 */
uint16_t radio_discovery_discover_sub_tree(UidList *radios, uint64_t lower,
                                           uint64_t upper);

#endif
