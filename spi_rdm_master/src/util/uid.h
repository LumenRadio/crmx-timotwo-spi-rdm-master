#ifndef UID_H_
#define UID_H_

#include <Arduino.h>

/**
 * Serializes a 48-bit UID into 6 bytes, most significant byte first.
 *
 * @param dest  Buffer to write the 6 serialized bytes into.
 * @param uid   The UID to serialize.
 */
void uid_serialize(uint8_t *dest, uint64_t uid);

/**
 * Deserializes a 48-bit UID from 6 bytes, most significant byte first.
 *
 * @param src   Buffer holding the 6 bytes to deserialize.
 */
uint64_t uid_deserialize(const uint8_t *src);

#endif
