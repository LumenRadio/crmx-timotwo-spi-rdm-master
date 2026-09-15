#ifndef DISCOVERY_H_
#define DISCOVERY_H_

#include <Arduino.h>

/* This variable sets if we are going to run in G4S mode or CRMX mode, defined
 * in spi_rdm_master.ino */
extern uint8_t rf_protocol;

/**
 * Performes a discovery of radio and RDM devices.
 *
 * @param incremental  If an incremental discovery should be used instead of a
 * full.
 */
void discovery_all(bool incremental);

#endif
