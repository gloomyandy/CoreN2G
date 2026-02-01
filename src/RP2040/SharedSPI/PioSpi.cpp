/**
 * PioSpi.cpp
 * Implement SPI using a PIO device. Based on RPI sample code.
 * Created: 18/12/2025
 * Author: Andy
 * 
 */
#include "PioSpi.h"
#include "PIOassignments.h"

pio_spi_inst piospi0;

// --------- //
// spi_cpha0 //
// --------- //

#define spi_cpha0_wrap_target 0
#define spi_cpha0_wrap 1
#define spi_cpha0_pio_version 0

static const uint16_t spi_cpha0_program_instructions[] = {
            //     .wrap_target
    0x6101, //  0: out    pins, 1         side 0 [1]
    0x5101, //  1: in     pins, 1         side 1 [1]
            //     .wrap
};

static const struct pio_program spi_cpha0_program = {
    .instructions = spi_cpha0_program_instructions,
    .length = 2,
    .origin = -1,
    .pio_version = spi_cpha0_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config spi_cpha0_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + spi_cpha0_wrap_target, offset + spi_cpha0_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

// --------- //
// spi_cpha1 //
// --------- //

#define spi_cpha1_wrap_target 0
#define spi_cpha1_wrap 2
#define spi_cpha1_pio_version 0

static const uint16_t spi_cpha1_program_instructions[] = {
            //     .wrap_target
    0x6021, //  0: out    x, 1            side 0
    0xb101, //  1: mov    pins, x         side 1 [1]
    0x4001, //  2: in     pins, 1         side 0
            //     .wrap
};

static const struct pio_program spi_cpha1_program = {
    .instructions = spi_cpha1_program_instructions,
    .length = 3,
    .origin = -1,
    .pio_version = spi_cpha1_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config spi_cpha1_program_get_default_config(uint offset) noexcept {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + spi_cpha1_wrap_target, offset + spi_cpha1_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

#include "hardware/gpio.h"
static inline void pio_spi_init(PIO pio, uint sm, uint prog_offs, uint n_bits,
        float clkdiv, bool cpha, bool cpol, uint pin_sck, uint pin_mosi, uint pin_miso) noexcept {
    pio_sm_config c = cpha ? spi_cpha1_program_get_default_config(prog_offs) : spi_cpha0_program_get_default_config(prog_offs);
    sm_config_set_out_pins(&c, pin_mosi, 1);
    sm_config_set_in_pins(&c, pin_miso);
    sm_config_set_sideset_pins(&c, pin_sck);
    // Only support MSB-first in this example code (shift to left, auto push/pull, threshold=nbits)
    sm_config_set_out_shift(&c, false, true, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);
    // MOSI, SCK output are low, MISO is input
    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_sck) | (1u << pin_mosi));
    pio_sm_set_pindirs_with_mask(pio, sm, (1u << pin_sck) | (1u << pin_mosi), (1u << pin_sck) | (1u << pin_mosi) | (1u << pin_miso));
    pio_gpio_init(pio, pin_mosi);
    pio_gpio_init(pio, pin_miso);
    pio_gpio_init(pio, pin_sck);
    // The pin muxes can be configured to invert the output (among other things
    // and this is a cheesy way to get CPOL=1
    gpio_set_outover(pin_sck, cpol ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    // SPI is synchronous, so bypass input synchroniser to reduce input delay.
    hw_set_bits(&pio->input_sync_bypass, 1u << pin_miso);
    pio_sm_init(pio, sm, prog_offs, &c);
    pio_sm_set_enabled(pio, sm, true);
}


static int32_t __time_critical_func(pio_spi_write_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, size_t len) noexcept {
    size_t tx_remain = len, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            (void) *rxfifo;
            --rx_remain;
        }
    }
    return len;
}

static int32_t __time_critical_func(pio_spi_read_blocking)(const pio_spi_inst_t *spi, uint8_t val, uint8_t *dst, size_t len) noexcept {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = val;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
    return len;
}

static int32_t __time_critical_func(pio_spi_write_read_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, uint8_t *dst,
                                                         size_t len) noexcept {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
    return len;
}

static void pio_spi_init(pio_spi_inst_t *spi, Pin clk, Pin miso, Pin mosi) noexcept
{
    spi->miso = miso;
    spi->mosi = mosi;
    spi->clk = clk;
    spi->pio = (PioSpiPioNumber) ? pio1_hw : pio0_hw;
    spi->sm = pio_claim_unused_sm(spi->pio, false);
    spi->cpha0_offs = pio_add_program(spi->pio, &spi_cpha0_program);
    spi->cpha1_offs = pio_add_program(spi->pio, &spi_cpha1_program);
}

static void pio_spi_set_format(const pio_spi_inst_t *spi, uint bits, bool cpol, bool cpha, uint msblsb, uint freq) noexcept
{
    pio_sm_set_enabled(spi->pio, spi->sm, false);
    const float div = SystemCoreClock / (freq*((cpha ? spi_cpha1_program.length : spi_cpha1_program.length)+1));
    pio_spi_init(spi->pio, spi->sm, (cpha ? spi->cpha1_offs : spi->cpha0_offs), bits,
        div, cpha, cpol, spi->clk, spi->mosi, spi->miso);
}

#if 0
static void pio_spi_disable(const pio_spi_inst_t *spi) noexcept
{
    pio_sm_set_enabled(spi->pio, spi->sm, false);
    gpio_set_outover(spi->clk, GPIO_OVERRIDE_NORMAL);
}
#endif

// Create SPI devices the actual configuration is set later
PioSPI PioSPI::PIOSPI0;

void PioSPI::initPins(Pin clk, Pin miso, Pin mosi, NvicPriority priority) noexcept
{
    pio_spi_init(&dev, clk, miso, mosi);
}

//setup the master device.
void PioSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    if (clockMode != curClockMode || curBitRate != bitRate)
    {
	    pio_spi_set_format(&dev, 8, ((uint8_t)clockMode & 2) != 0, ((uint8_t)clockMode & 1) != 0, true, bitRate);
        curClockMode = clockMode;
        curBitRate = bitRate;
    }
}


PioSPI::PioSPI() noexcept
{
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
}


spi_status_t PioSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs) noexcept
{
    spi_status_t ret = SPI_OK;
    if (cs != NoPin) fastDigitalWriteLow(cs);
	const int bytesTransferred = (rx_data == nullptr) ? pio_spi_write_blocking(&dev, tx_data, len)
								: (tx_data == nullptr) ? pio_spi_read_blocking(&dev, 0xFF, rx_data, len)
									: pio_spi_write_read_blocking(&dev, tx_data, rx_data, len);
	ret = bytesTransferred == (int)len ? SPI_OK : SPI_ERROR;

    if (cs != NoPin) fastDigitalWriteHigh(cs);
    return ret;
}
