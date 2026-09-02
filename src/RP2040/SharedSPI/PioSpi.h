#ifndef _PIOSPI_H
#define _PIOSPI_H

#include <CoreIO.h>
#include <hardware/structs/pio.h>
#include "hardware/pio.h"
#include "SPI.h"
#ifdef RTOS
#include <RTOSIface/RTOSIface.h>
#include <DmacManager.h>
#endif

typedef struct pio_spi_inst {
    PIO pio;
    uint32_t sm;
    Pin miso;
    Pin mosi;
    Pin clk;
    uint32_t cpha0_offs;
    uint32_t cpha1_offs;
} pio_spi_inst_t;

class PioSPI: public SPI
{
public:
    PioSPI() noexcept;
    spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs = NoPin) noexcept override;
    void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept override; // Master mode
    void initPins(Pin sck, Pin miso, Pin mosi, NvicPriority priority = -1) noexcept override;
    void initDma(DmaChannel txChan, DmaChannel rxChan) noexcept override;
    static PioSPI PIOSPI0;
protected:
    static void DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept;
	volatile TaskHandle taskToWake;

private:
    bool useDma;
    DmaChannel rxChan;
    DmaChannel txChan;
    pio_spi_inst_t dev;
    uint32_t curBitRate;
    uint32_t curBits;
    uint32_t curClockMode;
};

#endif