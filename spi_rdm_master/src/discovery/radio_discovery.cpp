#include "radio_discovery.h"

#include "../timo_spi/timo_spi.h"
#include "../util/debug_print.h"
#include "../util/uid_list.h"

DiscoveryResponseType radio_discovery_dub(uint64_t lower, uint64_t upper,
                                          uint64_t *uid) {
    Serial.print("Radio DUB (");
    debug_print_uid(lower);

    Serial.print(":");
    debug_print_uid(upper);
    Serial.println(")...");

    tx_buffer[0] = lower >> 40;
    tx_buffer[1] = lower >> 32;
    tx_buffer[2] = lower >> 24;
    tx_buffer[3] = lower >> 16;
    tx_buffer[4] = lower >> 8;
    tx_buffer[5] = lower;
    tx_buffer[6] = upper >> 40;
    tx_buffer[7] = upper >> 32;
    tx_buffer[8] = upper >> 24;
    tx_buffer[9] = upper >> 16;
    tx_buffer[10] = upper >> 8;
    tx_buffer[11] = upper;
    timo_spi_transfer(TIMO_RADIO_DISCOVERY, rx_buffer, tx_buffer, 13);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG);

    timo_spi_transfer(TIMO_RADIO_DISCOVERY_RESULT, rx_buffer, tx_buffer, 8);

    if (rx_buffer[0] == 1) {
        Serial.println("No response.");
        return DiscoveryNone;
    } else if (rx_buffer[0] == 2) {
        Serial.println("Collission.");
        return DiscoveryCollission;
    } else if (rx_buffer[0] == 3) {
        uint64_t found_uid = 0;
        Serial.print("Found dev: ");
        for (int i = 0; i < 6; i++) {
            found_uid = found_uid << 8;
            found_uid |= rx_buffer[1 + i];
        }
        debug_print_uid(found_uid);
        Serial.println();
        *uid = found_uid;
        return DiscoveryUid;
    }

    return DiscoveryNone;
}

bool radio_discovery_mute(uint64_t uid) {
    Serial.print("Trying to mute ");
    debug_print_uid(uid);
    Serial.println("...");

    tx_buffer[0] = uid >> 40;
    tx_buffer[1] = uid >> 32;
    tx_buffer[2] = uid >> 24;
    tx_buffer[3] = uid >> 16;
    tx_buffer[4] = uid >> 8;
    tx_buffer[5] = uid;
    tx_buffer[6] = 1;
    timo_spi_transfer(TIMO_RADIO_MUTE, rx_buffer, tx_buffer, 8);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG);

    timo_spi_transfer(TIMO_RADIO_MUTE_RESPONSE, rx_buffer, tx_buffer, 2);

    if (rx_buffer[0]) {
        Serial.println("Muted.");
        return true;
    } else {
        Serial.println("Failed to mute.");
        return false;
    }
}

void radio_discovery_unmute_all(void) {
    Serial.println("Unmuting all radios...");

    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xFF;
    tx_buffer[2] = 0xFF;
    tx_buffer[3] = 0xFF;
    tx_buffer[4] = 0xFF;
    tx_buffer[5] = 0xFF;
    tx_buffer[6] = 0;
    timo_spi_transfer(TIMO_RADIO_MUTE, rx_buffer, tx_buffer, 8);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG);

    timo_spi_transfer(TIMO_RADIO_MUTE_RESPONSE, rx_buffer, tx_buffer, 2);
}

uint16_t radio_discovery_discover_sub_tree(uint64_t lower, uint64_t upper) {
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

        /* If there was collissions we need to branch further down in the binary
         * tree */
        if (dub_resp == DiscoveryCollission) {
            uint64_t mid = (lower + upper) / 2;
            /* the total amount of devices found in this part of the tree is the
             * sum of: the number of devices we already found + the number of
             * devices in each half of the tree */
            return n_found + radio_discovery_discover_sub_tree(lower, mid) +
                   radio_discovery_discover_sub_tree(mid + 1, upper);
        }

        /* if we got a single response we will try to mute it to verify it's a
         * true response */
        if (dub_resp == DiscoveryUid) {
            if (radio_discovery_mute(uid)) {
                /* if we could mute it and we do not already have it in the list
                 * - add it. */
                if (!uid_list_radio_is_in_list(uid)) {
                    uid_list_add_radio_to_list(uid);
                    n_found++;
                }
            }
        }

    } while (
        dub_resp ==
        DiscoveryUid); /* keep trying while we're getting single responses */

    return n_found;
}
