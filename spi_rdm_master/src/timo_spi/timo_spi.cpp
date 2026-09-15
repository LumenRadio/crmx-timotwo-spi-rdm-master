#include "timo_spi.h"

#include <SPI.h>

static timo_t timo;

static uint32_t tx_buffer32[300 / 4];
static uint32_t rx_buffer32[300 / 4];
uint8_t *tx_buffer = (uint8_t *)tx_buffer32;
uint8_t *rx_buffer = (uint8_t *)rx_buffer32;

static void timo_spi_irq_pin_handler() { timo.irq_pending = 1; }

void timo_spi_init(int csn_pin, int irq_pin) {
    timo.csn_pin = csn_pin;
    timo.irq_pin = irq_pin;
    timo.irq_pending = false;

    pinMode(timo.irq_pin, INPUT);
    pinMode(timo.csn_pin, OUTPUT);
    digitalWrite(timo.csn_pin, HIGH);
    attachInterrupt(digitalPinToInterrupt(timo.irq_pin),
                    timo_spi_irq_pin_handler, FALLING);

    SPI.begin();
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
}

bool timo_spi_irq_is_pending() {
    noInterrupts();
    bool pending = timo.irq_pending;
    timo.irq_pending = false;
    interrupts();
    return pending;
}

int16_t timo_spi_transfer(uint8_t command, uint8_t *dst, uint8_t *src,
                          uint32_t len) {
    uint8_t irq_flags;

    uint32_t start_time = millis();

    /* Perform the transfer of the command byte */
    digitalWrite(timo.csn_pin, LOW);
    irq_flags = SPI.transfer(command);
    timo_spi_irq_is_pending();
    digitalWrite(timo.csn_pin, HIGH);

    /* If no bytes to transfer, this was a NOP command - just wait for IRQ or
     * timeout */
    if (len == 0) {
        start_time = millis();
        while ((!digitalRead(timo.irq_pin)) && (!timo_spi_irq_is_pending())) {
            if (millis() - start_time > 10) {
                break;
            }
        }
        return irq_flags;
    }

    /* wait for IRQ or timeout */
    while (!timo_spi_irq_is_pending()) {
        if (millis() - start_time > 1000) {
            return -1;
        }
    }

    /* start the payload transfer */
    digitalWrite(timo.csn_pin, LOW);
    irq_flags = SPI.transfer(0xff);

    /* If busy flag is set we can't do the transfer now, cancel */
    if (irq_flags & TIMO_SPI_DEVICE_BUSY_IRQ_MASK) {
        digitalWrite(timo.csn_pin, HIGH);
        return irq_flags;
    }

    /* Transfer the data */
    for (uint32_t i = 0; i < len - 1; i++) {
        *dst++ = SPI.transfer(*src++);
    }

    /* End transfer */
    digitalWrite(timo.csn_pin, HIGH);

    /* wair for IRQ or timeout */
    while (!digitalRead(timo.irq_pin)) {
        if (millis() - start_time > 50) {
            break;
        }
    }
    return irq_flags;
}

int16_t timo_spi_transfer_rdm_response(uint8_t command, uint8_t *dst,
                                       uint8_t *src, uint32_t max_len) {
    uint8_t irq_flags;

    uint32_t start_time = millis();

    /* write command */
    digitalWrite(timo.csn_pin, LOW);
    irq_flags = SPI.transfer(command);
    timo_spi_irq_is_pending();
    digitalWrite(timo.csn_pin, HIGH);

    /* if we don't accept any data this was just to issue the command - quit
     * after IRQ was received */
    if (max_len == 0) {
        start_time = millis();
        while ((!digitalRead(timo.irq_pin)) && (!timo_spi_irq_is_pending())) {
            if (millis() - start_time > 10) {
                break;
            }
        }
        return 0;
    }

    /* otherwise wait for IRQ or timeout */
    while (!timo_spi_irq_is_pending()) {
        if (millis() - start_time > 1000) {
            return -1;
        }
    }

    /* start reading the data */
    digitalWrite(timo.csn_pin, LOW);
    irq_flags = SPI.transfer(0xff);

    /* If busy flag is set we can't do the transfer now, cancel */
    if (irq_flags & TIMO_SPI_DEVICE_BUSY_IRQ_MASK) {
        digitalWrite(timo.csn_pin, HIGH);
        return 0;
    }

    /* read the data */
    for (uint32_t i = 0; i < max_len - 1; i++) {
        uint8_t data;
        data = SPI.transfer(*src++);
        if ((i == 0) && (data == 0)) {
            /* if first byte (result code) is 0, that means there was no
             * response */
            *dst++ = data;
            break;
        }
        if (i == 3) {
            /* if we gotten here, this is the RDM packet length field, calculate
             * the actual number of bytes to read */
            if ((data + 2) < (max_len - 1)) {
                max_len = data + 2 + 1;
            }
        }
        *dst++ = data;
    }

    /* end transfer */
    digitalWrite(timo.csn_pin, HIGH);

    /* wait for IRQ or timeout */
    while (!digitalRead(timo.irq_pin)) {
        if (millis() - start_time > 50) {
            break;
        }
    }
    return max_len - 1;
}

/**
 * Waits for an extended IRQ whose flags register has the given bit set.
 *
 * @param flag   The TIMO_EXTIRQ_SPI_*_FLAG bit to wait for.
 */
void timo_spi_wait_for_extended_irq(uint32_t flag) {
    int16_t irq_flags;

    while (1) {
        /* module indicates IRQ */
        if (!digitalRead(timo.irq_pin)) {
            /* Issue NOP - this returs the IRQ flags */
            irq_flags =
                timo_spi_transfer(TIMO_NOP_COMMAND, rx_buffer, tx_buffer, 0);

            while (!timo_spi_irq_is_pending()) {
                ;
            }

            /* if it's an extended IRQ it can be the one we're waiting for */
            if (irq_flags & TIMO_IRQ_EXTENDED_FLAG) {
                uint32_t ext_flags;
                bzero(tx_buffer, 5);
                /* read the extended flags register */
                irq_flags = timo_spi_transfer(
                    TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_FLAGS_REG), rx_buffer,
                    tx_buffer, 5);
                ext_flags = ((uint32_t)rx_buffer[0] << 24) |
                            ((uint32_t)rx_buffer[1] << 16) |
                            ((uint32_t)rx_buffer[2] << 8) |
                            ((uint32_t)rx_buffer[3]);
                if (ext_flags & flag) {
                    while (!timo_spi_irq_is_pending()) {
                        ;
                    }
                    return;
                }
            }
        }
    }
}
