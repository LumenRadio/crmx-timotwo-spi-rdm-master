#include "timo_spi_mode.h"

#include "../serial/serial.h"
#include "timo_spi.h"
#include "timo_spi_reg.h"

void timo_spi_mode_change(timo_spi_mode_t mode) {
    uint8_t config;
    int16_t irq_flags = timo_spi_reg_read_config(&config);
    serial_print_response(irq_flags, &config, 1);

    uint8_t desired_bit =
        (mode == TIMO_SPI_MODE_TX) ? TIMO_CONFIG_RADIO_TX_RX_MODE : 0;

    while ((config & TIMO_CONFIG_RADIO_TX_RX_MODE) != desired_bit) {
        serial_print("Changing mode to ");
        serial_println(mode == TIMO_SPI_MODE_TX ? "TX" : "RX");
        uint8_t new_config = (mode == TIMO_SPI_MODE_TX)
                                 ? (config | TIMO_CONFIG_RADIO_TX_RX_MODE)
                                 : (config & ~TIMO_CONFIG_RADIO_TX_RX_MODE);
        timo_spi_reg_write_config(new_config);
        delay(3000);
        irq_flags = timo_spi_reg_read_config(&config);
        serial_print_response(irq_flags, &config, 1);
    }
}
