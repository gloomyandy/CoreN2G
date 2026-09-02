#ifndef SPI_H
#define SPI_H

#include "Core.h"

constexpr uint16_t SPITimeoutMillis = 250;
typedef enum
{
    SPI_ERROR = -1,
    SPI_OK = 0,
} spi_status_t;

// SSP/SPI Channels - yes I know the names do not match the numbers...
enum SPIChannel : uint8_t
{
    //Hardware SPI
    SPI0 = 0,
    SPI1,
    SPI2,
    SSPMAX,
    // Not defined
    SSPNONE = 0xff
};
constexpr size_t NumSPIDevices = (uint32_t)SSPMAX;

class SPI
{
public:
    virtual void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept;
    virtual spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs = NoPin) noexcept;    
    virtual void initPins(Pin sck, Pin miso, Pin mosi, NvicPriority priority = -1) noexcept;
    virtual void initDma(DmaChannel txChan, DmaChannel rxChan) noexcept;
    static SPI *getSPIDevice(SPIChannel channel) noexcept;
};

#endif
