#ifndef TIMO_SPI_REG_H_
#define TIMO_SPI_REG_H_

#include <Arduino.h>

/**
 * Reads the CONFIG register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *config   Where to store the register value.
 */
int16_t timo_spi_reg_read_config(uint8_t *config);

/**
 * Writes the CONFIG register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param config   The value to write.
 */
int16_t timo_spi_reg_write_config(uint8_t config);

/**
 * Reads the STATUS register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *status   Where to store the register value.
 */
int16_t timo_spi_reg_read_status(uint8_t *status);

/**
 * Reads the IRQ_MASK register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *irq_mask   Where to store the register value.
 */
int16_t timo_spi_reg_read_irq_mask(uint8_t *irq_mask);

/**
 * Writes the IRQ_MASK register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param irq_mask   The value to write.
 */
int16_t timo_spi_reg_write_irq_mask(uint8_t irq_mask);

/**
 * Reads the EXT_IRQ_MASK register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *ext_irq_mask   Where to store the register value.
 */
int16_t timo_spi_reg_read_ext_irq_mask(uint32_t *ext_irq_mask);

/**
 * Writes the EXT_IRQ_MASK register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param ext_irq_mask   The value to write.
 */
int16_t timo_spi_reg_write_ext_irq_mask(uint32_t ext_irq_mask);

/**
 * Reads the VERSION register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *dst   Where to store the register value.
 * @param len    Number of bytes to read.
 */
int16_t timo_spi_reg_read_version(uint8_t *dst, uint32_t len);

/**
 * Reads the INSTALLED_OPTIONS register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *dst   Where to store the register value.
 * @param len    Number of bytes to read.
 */
int16_t timo_spi_reg_read_installed_options(uint8_t *dst, uint32_t len);

/**
 * Reads the BLE_STATUS register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *ble_status   Where to store the register value.
 */
int16_t timo_spi_reg_read_ble_status(uint8_t *ble_status);

/**
 * Writes the BLE_STATUS register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param ble_status   The value to write.
 */
int16_t timo_spi_reg_write_ble_status(uint8_t ble_status);

/**
 * Reads the RF_PROTOCOL register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param *rf_protocol   Where to store the register value.
 */
int16_t timo_spi_reg_read_rf_protocol(uint8_t *rf_protocol);

/**
 * Writes the RF_PROTOCOL register.
 *
 * Return value:   The transfer status - the content of the IRQ flags
 * register, or -1 on failure.
 *
 * @param rf_protocol   The value to write.
 */
int16_t timo_spi_reg_write_rf_protocol(uint8_t rf_protocol);

#endif
