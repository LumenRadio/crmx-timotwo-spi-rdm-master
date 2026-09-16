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
#include "src/serial/serial.h"
#include "src/timo_spi/timo_spi.h"
#include "src/timo_spi/timo_spi_reg.h"
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
    serial_init(115200);

    /* Initiate the TimoTwo module: pins, IRQ handler and SPI */
    timo_spi_init(/*csn_pin=*/5, /*irq_pin=*/3);

    rdm_protocol_register_uid(my_uid);

    delay(1000);

    serial_println("Running");
    serial_flush();

    /* Wait here until module has booted and IRQ signal is high */
    while (timo_spi_irq_is_pending()) {
        ;
    }

    serial_println("Version:");
    irq_flags = timo_spi_reg_read_version(rx_buffer, 8);
    serial_print_response(irq_flags, rx_buffer, 8);

    /* Making sure module is in TX mode */
    serial_println("Config:");
    uint8_t config;
    irq_flags = timo_spi_reg_read_config(&config);
    serial_print_response(irq_flags, &config, 1);
    while ((config & TIMO_CONFIG_RADIO_TX_RX_MODE) == 0) {
        serial_println("In RX mode - changing mode");
        /* in RX mode - change to TX */
        timo_spi_reg_write_config(config | TIMO_CONFIG_RADIO_TX_RX_MODE);
        delay(3000);
        irq_flags = timo_spi_reg_read_config(&config);
        serial_print_response(irq_flags, &config, 1);
    }

    /* Making sure we have the RDM SPI TX option installed, assume it being
     * among the first 5 options, so only read 10 bytes */
    serial_println("Options:");
    irq_flags = timo_spi_reg_read_installed_options(rx_buffer, 9);
    serial_print_response(irq_flags, rx_buffer, 1);
    serial_print("Module has ");
    serial_print(rx_buffer[0]);
    serial_println(" installed option(s).");
    int option_counter;
    for (option_counter = 0; option_counter < rx_buffer[0]; option_counter++) {
        uint16_t option =
            ((uint16_t)rx_buffer[1 + option_counter * 2 + 1] << 8) |
            (uint16_t)rx_buffer[1 + option_counter * 2];
        if (option == TIMO_INSTALLED_OPTION_RDM_SPI) {
            serial_println("RDM SPI TX option found.");
            break;
        }
    }
    if (option_counter == rx_buffer[0]) {
        /* Reached end of list - our option not found in list */
        serial_println("Option for SPI RDM TX not found...");
        while (1) {
            ;
        }
    }

    /* Turning off BLE - this is important when running W-DMX as old receivers
     * does not deal with BLE
     */
    serial_println("BLE:");
    uint8_t ble_status;
    irq_flags = timo_spi_reg_read_ble_status(&ble_status);
    serial_print_response(irq_flags, &ble_status, 1);
    while (ble_status & TIMO_BLE_ENABLED) {
        serial_println("BLE is on - turning off");
        timo_spi_reg_write_ble_status(0x00);
        delay(3000);
        irq_flags = timo_spi_reg_read_ble_status(&ble_status);
        serial_print_response(irq_flags, &ble_status, 1);
    }

    /* Making sure the module is configured for the correct protocol */
    serial_println("RF Protocol:");
    uint8_t current_rf_protocol;
    irq_flags = timo_spi_reg_read_rf_protocol(&current_rf_protocol);
    serial_print_response(irq_flags, &current_rf_protocol, 1);
    while (current_rf_protocol != rf_protocol) {
        serial_println("Configured for wrong protocol - changing to G4S");
        timo_spi_reg_write_rf_protocol(rf_protocol);
        delay(3000);
        irq_flags = timo_spi_reg_read_rf_protocol(&current_rf_protocol);
        serial_print_response(irq_flags, &current_rf_protocol, 1);
    }

    serial_println("Status:");
    uint8_t status;
    irq_flags = timo_spi_reg_read_status(&status);
    serial_print_response(irq_flags, &status, 1);

    /* Enabling the extended IRQs */
    serial_println("IRQ mask:");
    timo_spi_reg_write_irq_mask(TIMO_IRQ_EXTENDED_FLAG);
    uint8_t irq_mask;
    irq_flags = timo_spi_reg_read_irq_mask(&irq_mask);
    serial_print_response(irq_flags, &irq_mask, 1);

    /* Enabling interrupts for Discovery (Radio and RDM), Mute and RDM */
    serial_println("Extended IRQ mask:");
    uint32_t ext_irq_mask =
        TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG | TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG |
        TIMO_EXTIRQ_SPI_RDM_DISC_FLAG | TIMO_EXTIRQ_SPI_RDM_FLAG;
    timo_spi_reg_write_ext_irq_mask(ext_irq_mask);
    irq_flags = timo_spi_reg_read_ext_irq_mask(&ext_irq_mask);
    serial_print_response(irq_flags, rx_buffer, 4);

    discovery_all(rf_protocol, false);
}

/**
 * This is the Arduino main loop function, it's called repeatedly
 */
void loop() {
    uint16_t selection;
    int i;

    serial_println();
    serial_println();

    /* Print a list of the discovered devices */
    for (i = 1; i < (discovery_device_count() + 1); i++) {
        serial_print(i);
        serial_print(") ");
        serial_print_uid(discovery_device_uid(i - 1));
        serial_print(" ");
        rdm_commands_print_manufacturer_label(0x00FFFFFFFFFFFF,
                                              discovery_device_uid(i - 1));
        serial_print(" ");
        rdm_commands_print_device_model_description(
            0x00FFFFFFFFFFFF, discovery_device_uid(i - 1));
        serial_println();
    }

    serial_print(i++);
    serial_println(") Find all devices");
    serial_print(i++);
    serial_println(") Find new devices");

    serial_set_timeout(0xFFFFFFFF);

    /* Wait for input from the user */
    selection = serial_parse_int();

    if (selection == (discovery_device_count() + 1)) {
        /* selection was to Find all devices (full discovery) */
        discovery_all(rf_protocol, false);
    } else if (selection == (discovery_device_count() + 2)) {
        /* selection was to Find new devices (incremental discovery) */
        discovery_all(rf_protocol, true);
    } else if ((selection > 0) && (selection <= discovery_device_count())) {
        /* a device was selected from the list - we will rdm_commands_identify
         * it for 5 seconds */
        serial_print("Identifying ");
        serial_print_uid(discovery_device_uid(selection - 1));
        serial_println("...");
        rdm_commands_identify(0x00FFFFFFFFFFFF,
                              discovery_device_uid(selection - 1), true);
        for (i = 0; i < 5; i++) {
            serial_print(5 - i);
            serial_print("... ");
            serial_flush();
            delay(1000);
        }
        rdm_commands_identify(0x00FFFFFFFFFFFF,
                              discovery_device_uid(selection - 1), false);
        serial_println("done");
    }
}
