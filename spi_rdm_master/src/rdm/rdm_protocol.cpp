#include "rdm_protocol.h"

#include "../util/uid.h"

static uint8_t my_uid[6];
static uint8_t rdm_tn = 0;

void rdm_protocol_register_uid(const uint8_t uid[6]) { memcpy(my_uid, uid, 6); }

void rdm_protocol_fill_packet(RdmRequest *r, uint64_t dest) {
    r->startCode = SC_RDM;
    r->subStartCode = SC_SUB_MESSAGE;
    r->transactionNumber = ++rdm_tn;
    r->portId = 1;
    r->messageCount = 0;
    r->subDevice = 0;
    memcpy(r->sourceUid, my_uid, 6);
    uid_serialize(r->destinationUid, dest);
}

void rdm_protocol_set_length_and_checksum(RdmRequest *r) {
    uint16_t sum = 0;

    r->messageLength = 24 + r->parameterDataLength;

    for (int i = 0; i < r->messageLength; i++) {
        sum += ((uint8_t *)r)[i];
    }

    ((uint8_t *)r)[r->messageLength] = sum >> 8;
    ((uint8_t *)r)[r->messageLength + 1] = sum;
}
