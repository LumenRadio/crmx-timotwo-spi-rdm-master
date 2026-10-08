#include "radio_discovery.h"

#include "../rdm/e120.h"
#include "../serial/serial.h"
#include "../timo_spi/timo_spi.h"
#include "../util/uid.h"
#include "../util/uid_list.h"

DiscoveryResponseType radio_discovery_dub(uint64_t lower, uint64_t upper,
                                          uint64_t *uid) {
    serial_print("Radio DUB (");
    serial_print_uid(lower);

    serial_print(":");
    serial_print_uid(upper);
    serial_println(")...");

    uid_serialize(tx_buffer, lower);
    uid_serialize(tx_buffer + 6, upper);
    timo_spi_transfer(TIMO_RADIO_DISCOVERY, rx_buffer, tx_buffer, 13);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG);

    timo_spi_transfer(TIMO_RADIO_DISCOVERY_RESULT, rx_buffer, tx_buffer, 8);

    if (rx_buffer[0] == 1) {
        serial_println("No response.");
        return DiscoveryNone;
    }

    if (rx_buffer[0] == 2) {
        serial_println("Collision.");
        return DiscoveryCollision;
    }

    if (rx_buffer[0] == 3) {
        uint64_t found_uid = uid_deserialize(rx_buffer + 1);
        serial_print("Found dev: ");
        serial_print_uid(found_uid);
        serial_println();
        *uid = found_uid;
        return DiscoveryUid;
    }

    return DiscoveryNone;
}

bool radio_discovery_mute(uint64_t uid) {
    serial_print("Trying to mute ");
    serial_print_uid(uid);
    serial_println("...");

    uid_serialize(tx_buffer, uid);
    tx_buffer[6] = 1;
    timo_spi_transfer(TIMO_RADIO_MUTE, rx_buffer, tx_buffer, 8);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG);

    timo_spi_transfer(TIMO_RADIO_MUTE_RESPONSE, rx_buffer, tx_buffer, 2);

    if (rx_buffer[0]) {
        serial_println("Muted.");
        return true;
    } else {
        serial_println("Failed to mute.");
        return false;
    }
}

void radio_discovery_unmute_all(void) {
    serial_println("Unmuting all radios...");

    uid_serialize(tx_buffer, BROADCAST_ALL_DEVICES_ID);
    tx_buffer[6] = 0;
    timo_spi_transfer(TIMO_RADIO_MUTE, rx_buffer, tx_buffer, 8);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG);

    timo_spi_transfer(TIMO_RADIO_MUTE_RESPONSE, rx_buffer, tx_buffer, 2);
}

uint16_t radio_discovery_discover_sub_tree(UidList *radios, uint64_t lower,
                                           uint64_t upper) {
    DiscoveryResponseType dub_resp;
    uint16_t n_found = 0;
    uint64_t uid;

    uint8_t n_retries = 1;

    do {
        /* Send a DUB. If we get no response, try again if there are retries
         * left */
        do {
            dub_resp = radio_discovery_dub(lower, upper, &uid);
        } while ((dub_resp == DiscoveryNone) && (n_retries-- > 0));

        /* If we still did not get any result - there is no radios in this part
         * of the binary tree. */
        if (dub_resp == DiscoveryNone) {
            return n_found;
        }

        /* If there was collisions we need to branch further down in the binary
         * tree */
        if (dub_resp == DiscoveryCollision) {
            uint64_t mid = (lower + upper) / 2;
            /* the total amount of devices found in this part of the tree is the
             * sum of: the number of devices we already found + the number of
             * devices in each half of the tree */
            return n_found +
                   radio_discovery_discover_sub_tree(radios, lower, mid) +
                   radio_discovery_discover_sub_tree(radios, mid + 1, upper);
        }

        /* if we got a single response we will try to mute it to verify it's a
         * true response */
        if (dub_resp == DiscoveryUid) {
            if (radio_discovery_mute(uid)) {
                /* if we could mute it and we do not already have it in the list
                 * - add it. */
                if (!uid_list_contains(radios, uid)) {
                    uid_list_add(radios, uid);
                    n_found++;
                }
            }
        }

    } while (
        dub_resp ==
        DiscoveryUid); /* keep trying while we're getting single responses */

    return n_found;
}
