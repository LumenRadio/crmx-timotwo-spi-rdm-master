#include "uid.h"

void uid_serialize(uint8_t *dest, uint64_t uid) {
    dest[0] = uid >> 40;
    dest[1] = uid >> 32;
    dest[2] = uid >> 24;
    dest[3] = uid >> 16;
    dest[4] = uid >> 8;
    dest[5] = uid;
}

uint64_t uid_deserialize(const uint8_t *src) {
    uint64_t uid = 0;

    for (int i = 0; i < 6; i++) {
        uid = (uid << 8) | src[i];
    }

    return uid;
}
