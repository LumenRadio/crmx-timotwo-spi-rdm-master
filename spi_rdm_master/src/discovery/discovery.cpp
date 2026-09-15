#include "discovery.h"

#include "../timo_spi/timo_spi.h"
#include "../util/debug_print.h"
#include "../util/uid_list.h"
#include "radio_discovery.h"
#include "rdm_discovery.h"

#define N_RADIOS 20
#define N_DEVICES 50

static uint64_t radios_storage[N_RADIOS];
static UidList radios = {radios_storage, 0, N_RADIOS};

static uint64_t rdm_devices_storage[N_DEVICES];
static UidList rdm_devices = {rdm_devices_storage, 0, N_DEVICES};

uint8_t discovery_device_count(void) { return rdm_devices.count; }

uint64_t discovery_device_uid(uint8_t index) {
    return rdm_devices.items[index];
}

void discovery_all(uint8_t rf_protocol, bool incremental) {
    uint64_t uid;
    uint16_t n_found = 0;

    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();

    Serial.println("Discovering radio receivers....");
    /* Unmuting all receivers */
    radio_discovery_unmute_all();
    radio_discovery_unmute_all();

    if (incremental) {
        /* if it's an incremental discovery we start off by muting all known
         * radios */
        for (int i = 0; i < radios.count; i++) {
            if (!radio_discovery_mute(radios.items[i])) {
                /* if we could not mute it, it's gone now - remove from list */
                uid_list_remove(&radios, radios.items[i]);
                i--;
            }
        }
    } else {
        /* if full discovery - just clear the list */
        radios.count = 0;
    }

    /* run discovery ofer the full binary search tree - this functions will
     * branch itself when needed
     */
    n_found = radio_discovery_discover_sub_tree(&radios, 0, 0x0000FFFFFFFFFFFF);

    Serial.print("Found ");
    Serial.print(n_found);
    Serial.print(" new radios (");
    Serial.print(radios.count);
    Serial.println(" in total).");

    /* There's some differences in how to get the RDM devices between W-DMX and
     * CRMX */
    if (rf_protocol == TIMO_RF_PROTO_G4S) {
        /* For W-DMX the receivers perform the local discovery, so we do not
         * care about incremental/full discover here */
        rdm_devices.count = 0;
        /* go through all found receivers and fethc the devices */
        for (int i = 0; i < radios.count; i++) {
            int8_t new_found = -1, retries = 2;
            Serial.print("Fetch devices from ");
            debug_print_uid(radios.items[i]);
            Serial.print("... ");
            while (new_found < 0) {
                new_found = rdm_discovery_fetch_devices_from_wdmx_receiver(
                    &rdm_devices, radios.items[i]);
                /* if negative means that we failed fetching, or that an old
                 * W-DMX receiver ignored the request when no devices found */
                if (new_found < 0) {
                    if (retries-- == 0) {
                        new_found = 0;
                        break;
                    }
                }
            }
            Serial.print("Got ");
            Serial.print(new_found);
            Serial.println(" devices...");
        }

        Serial.print("Found ");
        Serial.print(rdm_devices.count);
        Serial.println(" devices in total.");
    } else { // CRMX
        /* Loop over all receivers to do RDM discovery downstream */
        rdm_discovery_unmute_all();
        rdm_discovery_unmute_all();
        if (incremental) {
            /* if it's an incremental discovery we start off by muting all known
             * radios */
            for (int i = 0; i < rdm_devices.count; i++) {
                if (!rdm_discovery_mute_device(0x00FFFFFFFFFFFF,
                                               rdm_devices.items[i])) {
                    /* if we could not mute it, it's gone now - remove from list
                     */
                    uid_list_remove(&rdm_devices, rdm_devices.items[i]);
                    i--;
                }
            }
        } else {
            rdm_devices.count = 0;
        }

        /* now discover any unmuted RDM devices */
        n_found = 0;
        for (int i = 0; i < radios.count; i++) {
            /* run discovery on each receiver */
            uint16_t new_found = 0;
            Serial.print("Running discovery on ");
            debug_print_uid(radios.items[i]);
            Serial.println("...");
            new_found += rdm_discovery_discover_sub_tree(
                &radios, &rdm_devices, radios.items[i], 0, 0x0000FFFFFFFFFFFF);
            Serial.print("Found ");
            Serial.print(new_found);
            Serial.println(" new...");
            n_found += new_found;
        }

        Serial.print("Found ");
        Serial.print(n_found);
        Serial.print(" new devices (");
        Serial.print(rdm_devices.count);
        Serial.println(" in total).");
    }
}
