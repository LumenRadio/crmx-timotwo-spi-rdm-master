/**
 * This example provides a very simple RDM controller using the TimoTwo module's
 * SPI RDM TX function. It can be used in CRMX mode or W-DMX G4S mode, and is
 * controlled be a variable set below. This example does not implements any
 * error checking of missed responses, etc. This must be added in a real-world
 * implementation.
 */

#include "src/discovery/discovery.h"
#include "src/rdm/rdm_commands.h"
#include "src/rdm/rdm_protocol.h"
#include "src/timo_spi/timo_spi.h"
#include "src/util/debug_print.h"
#include <SPI.h>

/* This variable sets if we are going to run in G4S mode or CRMX mode, uncomment
 * the one you want */
uint8_t rf_protocol = TIMO_RF_PROTO_G4S;
// uint8_t rf_protocol = TIMO_RF_PROTO_CRMX;

/* The UID to use for this controller */
uint8_t my_uid[6] = {0x4c, 0x55, 0x00, 0x00, 0x00, 0x12};

#define MIN(A, B) ((A) < (B) ? (A) : (B))

static bool has_set_up = false;

/**
 * This is the Arduino setup function, it's called when the Arduino starts up
 */
void setup() {
    int16_t irq_flags;

    /* Initiate serial port to 115200 bps */
    Serial.begin(115200);

    /* Initiate the TimoTwo module: pins, IRQ handler and SPI */
    timo_spi_init(/*csn_pin=*/5, /*irq_pin=*/3);

    rdm_protocol_register_uid(my_uid);

    delay(1000);

    /* Clear the Serial port from any garbage bytes */
    while (Serial.available() > 0) {
        Serial.read();
    }

    Serial.println("Running");
    Serial.flush();

    /* Wait here until module has booted and IRQ signal is high */
    while (timo_spi_irq_is_pending()) {
        ;
    }

    Serial.println("Version:");
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_VERSION_REG),
                                  rx_buffer, tx_buffer, 9);
    debug_print_response(irq_flags, rx_buffer, 8);

    /* Making sure module is in TX mode */
    Serial.println("Config:");
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_CONFIG_REG),
                                  rx_buffer, tx_buffer, 2);
    debug_print_response(irq_flags, rx_buffer, 1);
    while ((rx_buffer[0] & TIMO_CONFIG_RADIO_TX_RX_MODE) == 0) {
        Serial.println("In RX mode - changing mode");
        /* in RX mode - change to TX */
        tx_buffer[0] = rx_buffer[0] | TIMO_CONFIG_RADIO_TX_RX_MODE;
        timo_spi_transfer(TIMO_WRITE_REG_COMMAND(TIMO_CONFIG_REG), rx_buffer,
                          tx_buffer, 2);
        delay(3000);
        irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_CONFIG_REG),
                                      rx_buffer, tx_buffer, 2);
        debug_print_response(irq_flags, rx_buffer, 1);
    }

    /* Making sure we have the RDM SPI TX option installed, assume it being
     * among the first 5 options, so only read 10 bytes */
    Serial.println("Options:");
    irq_flags =
        timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_INSTALLED_OPTIONS_REG),
                          rx_buffer, tx_buffer, 10);
    debug_print_response(irq_flags, rx_buffer, 1);
    Serial.print("Module has ");
    Serial.print(rx_buffer[0]);
    Serial.println(" installed option(s).");
    int option_counter;
    for (option_counter = 0; option_counter < rx_buffer[0]; option_counter++) {
        uint16_t option =
            ((uint16_t)rx_buffer[1 + option_counter * 2 + 1] << 8) |
            (uint16_t)rx_buffer[1 + option_counter * 2];
        if (option == TIMO_INSTALLED_OPTION_RDM_SPI) {
            Serial.println("RDM SPI TX option found.");
            break;
        }
    }
    if (option_counter == rx_buffer[0]) {
        /* Reached end of list - our option not found in list */
        Serial.println("Option for SPI RDM TX not found...");
        while (1) {
            ;
        }
    }

    /* Turning off BLE - this is important when running W-DMX as old receivers
     * does not deal with BLE
     */
    Serial.println("BLE:");
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_BLE_STATUS_REG),
                                  rx_buffer, tx_buffer, 2);
    debug_print_response(irq_flags, rx_buffer, 1);
    while (rx_buffer[0] & TIMO_BLE_ENABLED) {
        Serial.println("BLE is on - turning off");
        tx_buffer[0] = 0x00;
        timo_spi_transfer(TIMO_WRITE_REG_COMMAND(TIMO_BLE_STATUS_REG),
                          rx_buffer, tx_buffer, 2);
        delay(3000);
        irq_flags =
            timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_BLE_STATUS_REG),
                              rx_buffer, tx_buffer, 2);
        debug_print_response(irq_flags, rx_buffer, 1);
    }

    /* Making sure the module is configured for the correct protocol */
    Serial.println("RF Protocol:");
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_RF_PROTOCOL_REG),
                                  rx_buffer, tx_buffer, 2);
    debug_print_response(irq_flags, rx_buffer, 1);
    while (rx_buffer[0] != rf_protocol) {
        Serial.println("Configured for wrong protocol - changing to G4S");
        tx_buffer[0] = rf_protocol;
        timo_spi_transfer(TIMO_WRITE_REG_COMMAND(TIMO_RF_PROTOCOL_REG),
                          rx_buffer, tx_buffer, 2);
        delay(3000);
        irq_flags =
            timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_RF_PROTOCOL_REG),
                              rx_buffer, tx_buffer, 2);
        debug_print_response(irq_flags, rx_buffer, 1);
    }

    Serial.println("Status:");
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_STATUS_REG),
                                  rx_buffer, tx_buffer, 2);
    debug_print_response(irq_flags, rx_buffer, 1);

    /* Enabling the extended IRQs */
    Serial.println("IRQ mask:");
    tx_buffer[0] = TIMO_IRQ_EXTENDED_FLAG;
    irq_flags = timo_spi_transfer(TIMO_WRITE_REG_COMMAND(TIMO_IRQ_MASK_REG),
                                  rx_buffer, tx_buffer, 2);
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_IRQ_MASK_REG),
                                  rx_buffer, tx_buffer, 2);
    debug_print_response(irq_flags, rx_buffer, 1);

    /* Enabling interrupts for Discovery (Radio and RDM), Mute and RDM */
    Serial.println("Extended IRQ mask:");
    tx_buffer[0] = 0;
    tx_buffer[1] = 0;
    tx_buffer[2] = 0;
    tx_buffer[3] = TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG |
                   TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG |
                   TIMO_EXTIRQ_SPI_RDM_DISC_FLAG | TIMO_EXTIRQ_SPI_RDM_FLAG;
    irq_flags = timo_spi_transfer(TIMO_WRITE_REG_COMMAND(TIMO_EXT_IRQ_MASK_REG),
                                  rx_buffer, tx_buffer, 5);
    irq_flags = timo_spi_transfer(TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_MASK_REG),
                                  rx_buffer, tx_buffer, 5);
    debug_print_response(irq_flags, rx_buffer, 4);

    discovery_all(rf_protocol, false);
}

/**
 * This is the Arduino main loop function, it's called repeatedly
 */
void loop() {
    uint16_t selection;
    int i;

    Serial.println();
    Serial.println();

    /* Print a list of the discovered devices */
    for (i = 1; i < (discovery_device_count() + 1); i++) {
        Serial.print(i);
        Serial.print(") ");
        debug_print_uid(discovery_device_uid(i - 1));
        Serial.print(" ");
        rdm_commands_print_manufacturer_label(0x00FFFFFFFFFFFF,
                                              discovery_device_uid(i - 1));
        Serial.print(" ");
        rdm_commands_print_device_model_description(
            0x00FFFFFFFFFFFF, discovery_device_uid(i - 1));
        Serial.println();
    }

    Serial.print(i++);
    Serial.println(") Find all devices");
    Serial.print(i++);
    Serial.println(") Find new devices");

    Serial.setTimeout(0xFFFFFFFF);

    /* Wait for input from the user */
    selection = Serial.parseInt();

    if (selection == (discovery_device_count() + 1)) {
        /* selection was to Find all devices (full discovery) */
        discovery_all(rf_protocol, false);
    } else if (selection == (discovery_device_count() + 2)) {
        /* selection was to Find new devices (incremental discovery) */
        discovery_all(rf_protocol, true);
    } else if ((selection > 0) && (selection <= discovery_device_count())) {
        /* a device was selected from the list - we will rdm_commands_identify
         * it for 5 seconds */
        Serial.print("Identifying ");
        debug_print_uid(discovery_device_uid(selection - 1));
        Serial.println("...");
        rdm_commands_identify(0x00FFFFFFFFFFFF,
                              discovery_device_uid(selection - 1), true);
        for (i = 0; i < 5; i++) {
            Serial.print(5 - i);
            Serial.print("... ");
            Serial.flush();
            delay(1000);
        }
        rdm_commands_identify(0x00FFFFFFFFFFFF,
                              discovery_device_uid(selection - 1), false);
        Serial.println("done");
    }
}
