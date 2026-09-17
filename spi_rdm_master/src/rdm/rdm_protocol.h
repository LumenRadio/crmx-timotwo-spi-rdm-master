#ifndef RDM_PROTOCOL_H_
#define RDM_PROTOCOL_H_

#include <Arduino.h>

#include "e120.h"

#define RDM_PROTOCOL_SWAP16(x)                                                 \
    ((uint16_t)(((uint16_t)x >> 8) | ((uint16_t)x << 8)))

/**
 * Registers the UID this controller uses as the source of every RDM packet
 * it builds. Must be called once before rdm_protocol_fill_packet.
 *
 * @param uid   The 6-byte UID to use.
 */
void rdm_protocol_register_uid(const uint8_t uid[6]);

/**
 * Fills an RDM packet with all the standard values.
 *
 * @param *r      Pointer to the request message.
 * @param dest    The UID of the device we want to send this request to.
 */
void rdm_protocol_fill_packet(RdmRequest *r, uint64_t dest);

/**
 * Calculates the total RDM message length and checksum.
 *
 * @param *r      Pointer to the request message.
 */
void rdm_protocol_set_length_and_checksum(RdmRequest *r);

#endif
