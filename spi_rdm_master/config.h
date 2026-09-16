#ifndef CONFIG_H_
#define CONFIG_H_

#include "src/timo_spi/timo_spi.h"

/* Which RF protocol this controller runs in - TIMO_RF_PROTO_CRMX,
 * TIMO_RF_PROTO_G4S. Change the value below, or
 * override it with a build flag (e.g. -DRF_PROTOCOL=TIMO_RF_PROTO_G4S)
 * without editing this file. */
#ifndef RF_PROTOCOL
#define RF_PROTOCOL TIMO_RF_PROTO_CRMX
#endif

#endif
