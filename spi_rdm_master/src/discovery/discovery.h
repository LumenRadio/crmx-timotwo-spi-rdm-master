#ifndef DISCOVERY_H_
#define DISCOVERY_H_

#include <Arduino.h>

/**
 * Performes a discovery of radio and RDM devices.
 *
 * @param rf_protocol  The TIMO_RF_PROTO_* mode the module is running in -
 * determines how downstream RDM devices are discovered.
 * @param incremental  If an incremental discovery should be used instead of a
 * full.
 */
void discovery_all(uint8_t rf_protocol, bool incremental);

/**
 * Return value:  The number of RDM devices currently known.
 */
uint8_t discovery_device_count(void);

/**
 * Return value:  The UID of the RDM device at the given index.
 *
 * @param index   Index into the known device list, 0-based.
 */
uint64_t discovery_device_uid(uint8_t index);

#endif
