#ifndef TIMO_SPI_MODE_H_
#define TIMO_SPI_MODE_H_

typedef enum {
    TIMO_SPI_MODE_RX,
    TIMO_SPI_MODE_TX,
} timo_spi_mode_t;

/**
 * Changes the TimoTwo module's TX/RX mode, polling and rewriting the CONFIG
 * register until the change has taken effect.
 *
 * @param mode   The mode to change to.
 */
void timo_spi_mode_change(timo_spi_mode_t mode);

#endif
