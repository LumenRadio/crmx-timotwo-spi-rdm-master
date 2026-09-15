#include "rdm_commands.h"

#include "../timo_spi/timo_spi.h"
#include "rdm_protocol.h"

void rdm_commands_print_manufacturer_label(uint64_t rx, uint64_t uid) {
    RdmRequest req;
    RdmResponse resp;

    rdm_protocol_fill_packet(&req, uid);

    req.commandClass = GET_COMMAND;
    req.parameterId = RDM_PROTOCOL_SWAP16(MANUFACTURER_LABEL);
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

    timo_spi_wait_for_rdm_response();

    timo_spi_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer,
                                   260);

    /* result code 1 means we got an answer */
    if (rx_buffer[0] == 1) {
        memcpy(&resp, rx_buffer + 1, rx_buffer[3] + 2);
        resp.parameterId = RDM_PROTOCOL_SWAP16(resp.parameterId);

        /* check for corrrect response */
        if ((resp.responseType == RESPONSE_TYPE_ACK) &&
            (resp.parameterId == MANUFACTURER_LABEL)) {
            for (int i = 0; i < resp.parameterDataLength; i++) {
                Serial.print((char)resp.parameterData[i]);
            }
        }
    }
}

void rdm_commands_print_device_model_description(uint64_t rx, uint64_t uid) {
    RdmRequest req;
    RdmResponse resp;

    rdm_protocol_fill_packet(&req, uid);

    req.commandClass = GET_COMMAND;
    req.parameterId = RDM_PROTOCOL_SWAP16(DEVICE_MODEL_DESCRIPTION);
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

    timo_spi_wait_for_rdm_response();

    timo_spi_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer,
                                   260);

    /* result code 1 means we got an answer */
    if (rx_buffer[0] == 1) {
        memcpy(&resp, rx_buffer + 1, rx_buffer[3] + 2);
        resp.parameterId = RDM_PROTOCOL_SWAP16(resp.parameterId);

        /* check for corrrect response */
        if ((resp.responseType == RESPONSE_TYPE_ACK) &&
            (resp.parameterId == DEVICE_MODEL_DESCRIPTION)) {
            for (int i = 0; i < resp.parameterDataLength; i++) {
                Serial.print((char)resp.parameterData[i]);
            }
        }
    }
}

void rdm_commands_identify(uint64_t rx, uint64_t uid, bool state) {
    RdmRequest req;
    RdmResponse resp;

    rdm_protocol_fill_packet(&req, uid);

    req.commandClass = SET_COMMAND;
    req.parameterId = RDM_PROTOCOL_SWAP16(IDENTIFY_DEVICE);
    req.parameterDataLength = 1;
    req.parameterData[0] = state ? 1 : 0;

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

    timo_spi_wait_for_rdm_response();

    timo_spi_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer,
                                   260);
}
