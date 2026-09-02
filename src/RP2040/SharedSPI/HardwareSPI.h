#ifndef HARDWARESPI_H
#define HARDWARESPI_H

#include "CoreIO.h"
#include "SPI.h"
#include "hardware/spi.h"

#ifdef RTOS
#include <RTOSIface/RTOSIface.h>
#include <DmacManager.h>
#endif

class HardwareSPI: public SPI
{
public:
    HardwareSPI(spi_inst_t *hardware) noexcept;
    spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs = NoPin) noexcept override;
    void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept override; // Master mode
    void initPins(Pin sck, Pin miso, Pin mosi, NvicPriority priority = -1) noexcept override;
    void initDma(DmaChannel txChan, DmaChannel rxChan) noexcept override;

    static HardwareSPI HWSPI0;
    static HardwareSPI HWSPI1;

protected:
    static void DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept;
	volatile TaskHandle taskToWake;

private:
    bool useDma;
    DmaChannel rxChan;
    DmaChannel txChan;
    spi_inst_t *dev;
    uint32_t curBitRate;
    uint32_t curBits;
    uint32_t curClockMode;
};

#endif
