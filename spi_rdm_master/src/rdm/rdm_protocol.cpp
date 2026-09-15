#include "rdm_protocol.h"

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
    r->destinationUid[0] = dest >> 40;
    r->destinationUid[1] = dest >> 32;
    r->destinationUid[2] = dest >> 24;
    r->destinationUid[3] = dest >> 16;
    r->destinationUid[4] = dest >> 8;
    r->destinationUid[5] = dest;
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
