#ifndef RDM_DISCOVERY_H_
#define RDM_DISCOVERY_H_

#include <Arduino.h>

#include "discovery_types.h"

/**
 * Unmute all RDM devices.
 */
void rdm_discovery_unmute_all();

/**
 * Tries to mute an RDM device.
 *
 * Return value:  Returns if device could successfully be muted.
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 */
bool rdm_discovery_mute_device(uint64_t rx, uint64_t uid);

/**
 * Issue a radio DISCOVERY UNIQUE BRANCH.
 *
 * Return values:
 *    DiscoveryNone         Empty response
 *    DiscoveryCollission   Collission
 *    DiscoveryUid          Single UID response
 *
 * @param rx      The UID of the receiver that we are performing the discover
 * via.
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 * @param *uid    Pointer to where to store the UID to in case of a signel
 * response
 */
DiscoveryResponseType rdm_discovery_dub(uint64_t rx, uint64_t lower,
                                        uint64_t upper, uint64_t *uid);

/**
 * Performes a RDM discovery on a part of the binary search tree.
 *
 * Return value:  The number of radios found in this part of the tree.
 *
 * @param rx      The receiver to perform the discover via.
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 */
uint16_t rdm_discovery_discover_sub_tree(uint64_t rx, uint64_t lower,
                                         uint64_t upper);

/**
 * Fetches a list of downstream RDM devices from a W-DMX G4S receiver.
 *
 * Return value:  The number of RDM devices found, or -1 if fetching list
 * failed. Some older W-DMX receivers does not return a list at all when the
 * list is empty.
 *
 * @param rx      The receiver to fetch RDM devie list from.
 */
int8_t rdm_discovery_fetch_devices_from_wdmx_receiver(uint64_t rx);

#endif
