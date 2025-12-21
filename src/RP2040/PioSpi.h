#ifndef _PIOSPI_H
#define _PIOSPI_H

#include <CoreIO.h>
#include <hardware/structs/pio.h>
#include "hardware/pio.h"

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    Pin miso;
    Pin mosi;
    Pin clk;
    uint cpha0_offs;
    uint cpha1_offs;
} pio_spi_inst_t;

extern pio_spi_inst piospi0;

int32_t pio_spi_write_blocking(const pio_spi_inst_t *spi, const uint8_t *src, size_t len) noexcept;

int32_t pio_spi_read_blocking(const pio_spi_inst_t *spi, uint8_t val, uint8_t *dst, size_t len) noexcept;

int32_t pio_spi_write_read_blocking(const pio_spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len) noexcept;

void pio_spi_init(pio_spi_inst_t *spi, Pin clk, Pin miso, Pin mosi) noexcept;

void pio_spi_set_format(const pio_spi_inst_t *spi, uint bits, bool cpol, bool cpha, uint msblsb, uint freq) noexcept;

void pio_spi_disable(const pio_spi_inst_t *spi) noexcept;
#endif