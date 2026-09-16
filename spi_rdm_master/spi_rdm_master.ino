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
#include "src/timo_spi/timo_spi_mode.h"
#include "src/timo_spi/timo_spi_reg.h"
#include <SPI.h>

/* This variable sets if we are going to run in G4S mode or CRMX mode, uncomment
 * the one you want */
uint8_t rf_protocol = TIMO_RF_PROTO_G4S;
// uint8_t rf_protocol = TIMO_RF_PROTO_CRMX;

/* The UID to use for this controller */
uint8_t my_uid[6] = {0x4c, 0x55, 0x00, 0x00, 0x00, 0x12};

/**
 * Waits here until the module has booted and the IRQ signal is high.
 */
static void wait_for_boot() {
    while (timo_spi_irq_is_pending()) {
        ;
    }
}

/**
 * Reads and prints the module's VERSION register.
 */
static void print_version() {
    serial_println("Version:");
    int16_t irq_flags = timo_spi_reg_read_version(rx_buffer, 8);
    serial_print_response(irq_flags, rx_buffer, 8);
}

/**
 * Makes sure we have the RDM SPI TX option installed, assuming it being
 * among the first 5 options, so only reads 10 bytes. Halts forever if the
 * option is not found.
 */
static void require_rdm_spi_option() {
    serial_println("Options:");
    int16_t irq_flags = timo_spi_reg_read_installed_options(rx_buffer, 9);
    serial_print_response(irq_flags, rx_buffer, 1);
    serial_print("Module has ");
    serial_print(rx_buffer[0]);
    serial_println(" installed option(s).");
    for (int option_counter = 0; option_counter < rx_buffer[0];
         option_counter++) {
        uint16_t option =
            ((uint16_t)rx_buffer[1 + option_counter * 2 + 1] << 8) |
            (uint16_t)rx_buffer[1 + option_counter * 2];
        if (option == TIMO_INSTALLED_OPTION_RDM_SPI) {
            serial_println("RDM SPI TX option found.");
            return;
        }
    }
    /* Reached end of list - our option not found in list */
    serial_println("Option for SPI RDM TX not found...");
    while (1) {
        ;
    }
}

/**
 * Turns off BLE - this is important when running W-DMX as old receivers
 * does not deal with BLE.
 */
static void disable_ble() {
    serial_println("BLE:");
    uint8_t ble_status;
    int16_t irq_flags = timo_spi_reg_read_ble_status(&ble_status);
    serial_print_response(irq_flags, &ble_status, 1);
    while (ble_status & TIMO_BLE_ENABLED) {
        serial_println("BLE is on - turning off");
        timo_spi_reg_write_ble_status(0x00);
        delay(3000);
        irq_flags = timo_spi_reg_read_ble_status(&ble_status);
        serial_print_response(irq_flags, &ble_status, 1);
    }
}

/**
 * Makes sure the module is configured for the rf_protocol set at the top of
 * this file.
 */
static void set_desired_rf_protocol() {
    serial_println("RF Protocol:");
    uint8_t current_rf_protocol;
    int16_t irq_flags = timo_spi_reg_read_rf_protocol(&current_rf_protocol);
    serial_print_response(irq_flags, &current_rf_protocol, 1);
    while (current_rf_protocol != rf_protocol) {
        serial_println("Configured for wrong protocol - changing to G4S");
        timo_spi_reg_write_rf_protocol(rf_protocol);
        delay(3000);
        irq_flags = timo_spi_reg_read_rf_protocol(&current_rf_protocol);
        serial_print_response(irq_flags, &current_rf_protocol, 1);
    }
}

/**
 * Reads and prints the module's STATUS register.
 */
static void print_status() {
    serial_println("Status:");
    uint8_t status;
    int16_t irq_flags = timo_spi_reg_read_status(&status);
    serial_print_response(irq_flags, &status, 1);
}

/**
 * Enables the extended IRQ, and the specific extended IRQs this sketch
 * cares about: Discovery (Radio and RDM), Mute and RDM.
 */
static void enable_extended_irqs() {
    serial_println("IRQ mask:");
    timo_spi_reg_write_irq_mask(TIMO_IRQ_EXTENDED_FLAG);
    uint8_t irq_mask;
    int16_t irq_flags = timo_spi_reg_read_irq_mask(&irq_mask);
    serial_print_response(irq_flags, &irq_mask, 1);

    serial_println("Extended IRQ mask:");
    uint32_t ext_irq_mask =
        TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG | TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG |
        TIMO_EXTIRQ_SPI_RDM_DISC_FLAG | TIMO_EXTIRQ_SPI_RDM_FLAG;
    timo_spi_reg_write_ext_irq_mask(ext_irq_mask);
    irq_flags = timo_spi_reg_read_ext_irq_mask(&ext_irq_mask);
    serial_print_response(irq_flags, rx_buffer, 4);
}

/**
 * Prints the list of discovered devices, followed by the two menu options
 * to trigger a new discovery.
 */
static void print_device_menu() {
    int i;
    for (i = 1; i < (discovery_device_count() + 1); i++) {
        serial_print(i);
        serial_print(") ");
        serial_print_uid(discovery_device_uid(i - 1));
        serial_print(" ");
        rdm_commands_print_manufacturer_label(BROADCAST_ALL_DEVICES_ID,
                                              discovery_device_uid(i - 1));
        serial_print(" ");
        rdm_commands_print_device_model_description(
            BROADCAST_ALL_DEVICES_ID, discovery_device_uid(i - 1));
        serial_println();
    }

    serial_print(i++);
    serial_println(") Find all devices");
    serial_print(i++);
    serial_println(") Find new devices");
}

/**
 * Identifies a device for 5 seconds, printing a countdown while waiting.
 *
 * @param uid   The UID of the device to identify.
 */
static void identify_device(uint64_t uid) {
    serial_print("Identifying ");
    serial_print_uid(uid);
    serial_println("...");
    rdm_commands_identify(BROADCAST_ALL_DEVICES_ID, uid, true);
    for (int i = 0; i < 5; i++) {
        serial_print(5 - i);
        serial_print("... ");
        serial_flush();
        delay(1000);
    }
    rdm_commands_identify(BROADCAST_ALL_DEVICES_ID, uid, false);
    serial_println("done");
}

/**
 * This is the Arduino setup function, it's called when the Arduino starts up
 */
void setup() {
    serial_init(115200);

    timo_spi_init(/*csn_pin=*/5, /*irq_pin=*/3);

    rdm_protocol_register_uid(my_uid);

    delay(1000);

    serial_println("Running");
    serial_flush();

    wait_for_boot();

    print_version();
    timo_spi_mode_change(TIMO_SPI_MODE_TX);
    require_rdm_spi_option();
    disable_ble();
    set_desired_rf_protocol();
    print_status();
    enable_extended_irqs();

    discovery_all(rf_protocol, false);
}

/**
 * This is the Arduino main loop function, it's called repeatedly
 */
void loop() {
    serial_println();
    serial_println();

    print_device_menu();

    serial_set_timeout(0xFFFFFFFF);

    /* Wait for input from the user */
    uint16_t selection = serial_parse_int();

    if (selection == (discovery_device_count() + 1)) {
        /* selection was to Find all devices (full discovery) */
        discovery_all(rf_protocol, false);
    } else if (selection == (discovery_device_count() + 2)) {
        /* selection was to Find new devices (incremental discovery) */
        discovery_all(rf_protocol, true);
    } else if ((selection > 0) && (selection <= discovery_device_count())) {
        /* a device was selected from the list - identify it for 5 seconds */
        identify_device(discovery_device_uid(selection - 1));
    }
}
