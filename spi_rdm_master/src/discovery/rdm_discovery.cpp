#include "rdm_discovery.h"

#include "../rdm/rdm_protocol.h"
#include "../serial/serial.h"
#include "../timo_spi/timo_spi.h"
#include "../util/uid_list.h"

void rdm_discovery_unmute_all() {
    RdmRequest req;

    rdm_protocol_fill_packet(&req, BROADCAST_ALL_DEVICES_ID);

    req.commandClass = DISCOVERY_COMMAND;
    req.parameterId = RDM_PROTOCOL_SWAP16(DISC_UN_MUTE);
    req.parameterDataLength = 0;

    rdm_protocol_set_length_and_checksum(&req);

    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xFF;
    tx_buffer[2] = 0xFF;
    tx_buffer[3] = 0xFF;
    tx_buffer[4] = 0xFF;
    tx_buffer[5] = 0xFF;

    memcpy(tx_buffer + 6, &req, req.messageLength + 2);

    timo_spi_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer,
                      1 + 6 + req.messageLength + 2);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RDM_FLAG);

    timo_spi_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer,
                                   260);
}

bool rdm_discovery_mute_device(uint64_t rx, uint64_t uid) {
    RdmRequest req;
    RdmResponse resp;

    rdm_protocol_fill_packet(&req, uid);

    req.commandClass = DISCOVERY_COMMAND;
    req.parameterId = RDM_PROTOCOL_SWAP16(DISC_MUTE);
    req.parameterDataLength = 0;

    rdm_protocol_set_length_and_checksum(&req);

    tx_buffer[0] = rx >> 40;
    tx_buffer[1] = rx >> 32;
    tx_buffer[2] = rx >> 24;
    tx_buffer[3] = rx >> 16;
    tx_buffer[4] = rx >> 8;
    tx_buffer[5] = rx;

    memcpy(tx_buffer + 6, &req, req.messageLength + 2);

    timo_spi_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer,
                      1 + 6 + req.messageLength + 2);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RDM_FLAG);

    timo_spi_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer,
                                   260);

    /* result code 1 means we got an answer */
    if (rx_buffer[0] == 1) {
        memcpy(&resp, rx_buffer + 1, rx_buffer[3] + 2);
        resp.parameterId = RDM_PROTOCOL_SWAP16(resp.parameterId);

        /* check for corrrect response */
        if ((resp.responseType == RESPONSE_TYPE_ACK) &&
            (resp.parameterId == DISC_MUTE)) {
            return true;
        }
    }

    return false;
}

DiscoveryResponseType rdm_discovery_dub(uint64_t rx, uint64_t lower,
                                        uint64_t upper, uint64_t *uid) {
    serial_print("RDM DUB (");
    serial_print_uid(lower);

    serial_print(":");
    serial_print_uid(upper);
    serial_println(")...");

    tx_buffer[0] = rx >> 40;
    tx_buffer[1] = rx >> 32;
    tx_buffer[2] = rx >> 24;
    tx_buffer[3] = rx >> 16;
    tx_buffer[4] = rx >> 8;
    tx_buffer[5] = rx;
    tx_buffer[6] = lower >> 40;
    tx_buffer[7] = lower >> 32;
    tx_buffer[8] = lower >> 24;
    tx_buffer[9] = lower >> 16;
    tx_buffer[10] = lower >> 8;
    tx_buffer[11] = lower;
    tx_buffer[12] = upper >> 40;
    tx_buffer[13] = upper >> 32;
    tx_buffer[14] = upper >> 24;
    tx_buffer[15] = upper >> 16;
    tx_buffer[16] = upper >> 8;
    tx_buffer[17] = upper;
    timo_spi_transfer(TIMO_RDM_DISCOVERY, rx_buffer, tx_buffer, 19);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RDM_DISC_FLAG);

    timo_spi_transfer(TIMO_RDM_DISCOVERY_RESULT, rx_buffer, tx_buffer, 8);

    if (rx_buffer[0] == 1) {
        serial_println("No response.");
        return DiscoveryNone;
    } else if (rx_buffer[0] == 2) {
        serial_println("Collission.");
        return DiscoveryCollission;
    } else if (rx_buffer[0] == 3) {
        uint64_t found_uid = 0;
        serial_print("Found dev: ");
        for (int i = 0; i < 6; i++) {
            found_uid = found_uid << 8;
            found_uid |= rx_buffer[1 + i];
        }
        serial_print_uid(found_uid);
        serial_println();
        *uid = found_uid;
        return DiscoveryUid;
    }

    return DiscoveryNone;
}

uint16_t rdm_discovery_discover_sub_tree(UidList *rdm_devices, uint64_t rx,
                                         uint64_t lower, uint64_t upper) {
    DiscoveryResponseType dub_resp;
    uint16_t n_found = 0;
    uint64_t uid;

    uint8_t n_retries = 1;

    do {
        /* If lower and upper are the same, it means we are at the bottom of the
         * tree. then try to mute that device */
        if (lower == upper) {
            serial_print("At leaf... Try to mute ");
            serial_print_uid(lower);
            serial_println();
            uid = lower;
            if (rdm_discovery_mute_device(rx, uid)) {
                /* add the device to the list if it was not already in the list
                 */
                if (!uid_list_contains(rdm_devices, uid)) {
                    uid_list_add(rdm_devices, uid);
                    return 1;
                }
            }
            /* if mute was not successful there was no device with this UID */
            return 0;
        } else {
            /* Send a DUB. If we get no response, try again if there are retries
             * left */
            do {
                dub_resp = rdm_discovery_dub(rx, lower, upper, &uid);
            } while ((dub_resp == DiscoveryNone) && (n_retries-- > 0));
        }

        /* If still no more responses then this part of the tree is done, return
         * the number of devices found */
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
            return n_found +
                   rdm_discovery_discover_sub_tree(rdm_devices, rx, lower,
                                                   mid) +
                   rdm_discovery_discover_sub_tree(rdm_devices, rx, mid + 1,
                                                   upper);
        }

        /* if we got a single response we will try to mute it to verify it's a
         * true response */
        if (dub_resp == DiscoveryUid) {
            if (rdm_discovery_mute_device(rx, uid)) {
                if (!uid_list_contains(rdm_devices, uid)) {
                    /* if device could be muted and did not exist in list
                     * before, then we found a new device
                     */
                    uid_list_add(rdm_devices, uid);
                    n_found++;
                } else {
                    /* if device was already in the list, but could be muted -
                     * this means we got a rough response caused by collissions
                     */
                    dub_resp = DiscoveryCollission;
                }
            } else {
                /* if device could not be muted - this means we got a rough
                 * response caused by collissions
                 */
                dub_resp = DiscoveryCollission;
            }
        }

        /* If there was collissions we need to branch further down in the binary
         * tree */
        if (dub_resp == DiscoveryCollission) {
            uint64_t mid = (lower + upper) / 2;
            /* the total amount of devices found in this part of the tree is the
             * sum of: the number of devices we already found + the number of
             * devices in each half of the tree */
            return n_found +
                   rdm_discovery_discover_sub_tree(rdm_devices, rx, lower,
                                                   mid) +
                   rdm_discovery_discover_sub_tree(rdm_devices, rx, mid + 1,
                                                   upper);
        }

    } while (
        dub_resp ==
        DiscoveryUid); /* keep trying while we're getting single responses */

    return n_found;
}

int8_t rdm_discovery_fetch_devices_from_wdmx_receiver(UidList *rdm_devices,
                                                      uint64_t rx) {
    RdmRequest req;
    RdmResponse resp;
    uint8_t n_dev = 0;

    rdm_protocol_fill_packet(&req, rx);

    req.commandClass = GET_COMMAND;
    req.parameterId = RDM_PROTOCOL_SWAP16(WDMX_RADIO_QUEUED_MESSAGE);
    req.parameterDataLength = 0;

    rdm_protocol_set_length_and_checksum(&req);

    tx_buffer[0] = rx >> 40;
    tx_buffer[1] = rx >> 32;
    tx_buffer[2] = rx >> 24;
    tx_buffer[3] = rx >> 16;
    tx_buffer[4] = rx >> 8;
    tx_buffer[5] = rx;

    memcpy(tx_buffer + 6, &req, req.messageLength + 2);

    timo_spi_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer,
                      1 + 6 + req.messageLength + 2);

    timo_spi_wait_for_extended_irq(TIMO_EXTIRQ_SPI_RDM_FLAG);

    timo_spi_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer,
                                   260);

    /* return code 1 means we got a proper result */
    if (rx_buffer[0] == 1) {
        memcpy(&resp, rx_buffer + 1, rx_buffer[3] + 2);
        resp.parameterId = RDM_PROTOCOL_SWAP16(resp.parameterId);

        if ((resp.responseType == RESPONSE_TYPE_ACK) &&
            (resp.parameterId == WDMX_RADIO_PROXIED_DEVICES)) {
            for (int i = 0; i < (resp.parameterDataLength / 8); i++) {
                uint64_t uid;

                uid = resp.parameterData[i * 8];
                uid <<= 8;
                uid |= resp.parameterData[i * 8 + 1];
                uid <<= 8;
                uid |= resp.parameterData[i * 8 + 2];
                uid <<= 8;
                uid |= resp.parameterData[i * 8 + 3];
                uid <<= 8;
                uid |= resp.parameterData[i * 8 + 4];
                uid <<= 8;
                uid |= resp.parameterData[i * 8 + 5];
                /* next two bytes are mute flasg, ignore these */

                uid_list_add(rdm_devices, uid);
                n_dev++;
            }
        }
    } else {
        /* no list returned - could be either lost packet or older W-DMX
         * receiver that doesn't return if empty */
        return -1;
    }

    return n_dev;
}
