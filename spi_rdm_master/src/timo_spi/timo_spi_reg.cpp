#include "timo_spi_reg.h"

#include "timo_spi.h"

/* How many extra attempts a register access gets if the transfer times out
 * or the device reports itself busy. */
#define TIMO_SPI_REG_MAX_RETRIES 2

int16_t timo_spi_reg_read_config(uint8_t *config) {
    int16_t irq_flags = timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_CONFIG_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
    *config = rx_buffer[0];
    return irq_flags;
}

int16_t timo_spi_reg_write_config(uint8_t config) {
    tx_buffer[0] = config;
    return timo_spi_transfer_with_retries(
        TIMO_WRITE_REG_COMMAND(TIMO_CONFIG_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
}

int16_t timo_spi_reg_read_status(uint8_t *status) {
    int16_t irq_flags = timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_STATUS_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
    *status = rx_buffer[0];
    return irq_flags;
}

int16_t timo_spi_reg_read_irq_mask(uint8_t *irq_mask) {
    int16_t irq_flags = timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_IRQ_MASK_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
    *irq_mask = rx_buffer[0];
    return irq_flags;
}

int16_t timo_spi_reg_write_irq_mask(uint8_t irq_mask) {
    tx_buffer[0] = irq_mask;
    return timo_spi_transfer_with_retries(
        TIMO_WRITE_REG_COMMAND(TIMO_IRQ_MASK_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
}

int16_t timo_spi_reg_read_ext_irq_mask(uint32_t *ext_irq_mask) {
    int16_t irq_flags = timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_MASK_REG), rx_buffer, tx_buffer, 5,
        TIMO_SPI_REG_MAX_RETRIES);
    *ext_irq_mask = ((uint32_t)rx_buffer[0] << 24) |
                    ((uint32_t)rx_buffer[1] << 16) |
                    ((uint32_t)rx_buffer[2] << 8) | (uint32_t)rx_buffer[3];
    return irq_flags;
}

int16_t timo_spi_reg_write_ext_irq_mask(uint32_t ext_irq_mask) {
    tx_buffer[0] = ext_irq_mask >> 24;
    tx_buffer[1] = ext_irq_mask >> 16;
    tx_buffer[2] = ext_irq_mask >> 8;
    tx_buffer[3] = ext_irq_mask;
    return timo_spi_transfer_with_retries(
        TIMO_WRITE_REG_COMMAND(TIMO_EXT_IRQ_MASK_REG), rx_buffer, tx_buffer, 5,
        TIMO_SPI_REG_MAX_RETRIES);
}

int16_t timo_spi_reg_read_version(uint8_t *dst, uint32_t len) {
    return timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_VERSION_REG), dst, tx_buffer, len + 1,
        TIMO_SPI_REG_MAX_RETRIES);
}

int16_t timo_spi_reg_read_installed_options(uint8_t *dst, uint32_t len) {
    return timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_INSTALLED_OPTIONS_REG), dst, tx_buffer,
        len + 1, TIMO_SPI_REG_MAX_RETRIES);
}

int16_t timo_spi_reg_read_ble_status(uint8_t *ble_status) {
    int16_t irq_flags = timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_BLE_STATUS_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
    *ble_status = rx_buffer[0];
    return irq_flags;
}

int16_t timo_spi_reg_write_ble_status(uint8_t ble_status) {
    tx_buffer[0] = ble_status;
    return timo_spi_transfer_with_retries(
        TIMO_WRITE_REG_COMMAND(TIMO_BLE_STATUS_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
}

int16_t timo_spi_reg_read_rf_protocol(uint8_t *rf_protocol) {
    int16_t irq_flags = timo_spi_transfer_with_retries(
        TIMO_READ_REG_COMMAND(TIMO_RF_PROTOCOL_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
    *rf_protocol = rx_buffer[0];
    return irq_flags;
}

int16_t timo_spi_reg_write_rf_protocol(uint8_t rf_protocol) {
    tx_buffer[0] = rf_protocol;
    return timo_spi_transfer_with_retries(
        TIMO_WRITE_REG_COMMAND(TIMO_RF_PROTOCOL_REG), rx_buffer, tx_buffer, 2,
        TIMO_SPI_REG_MAX_RETRIES);
}
