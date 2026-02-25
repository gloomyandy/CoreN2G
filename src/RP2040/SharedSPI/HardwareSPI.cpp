//Hardware SPI
#include "HardwareSPI.h"

#ifdef RTOS
#include <RTOSIface/RTOSIface.h>
#include <CoreNotifyIndices.h>
#endif


// Create SPI devices the actual configuration is set later
HardwareSPI HardwareSPI::HWSPI0(spi0);
HardwareSPI HardwareSPI::HWSPI1(spi1);

void HardwareSPI::initPins(Pin clk, Pin miso, Pin mosi, NvicPriority priority) noexcept
{
    if (clk != NoPin && miso != NoPin && mosi != NoPin)
    {
        SetPinFunction(clk, GpioPinFunction::Spi);
        SetPinFunction(miso, GpioPinFunction::Spi);
        SetPinFunction(mosi, GpioPinFunction::Spi);
    }
}

//setup the master device.
void HardwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    if (curBitRate != bitRate)
    {
	    spi_init(dev, bitRate);
        curBitRate = bitRate;
    }
    if (clockMode != curClockMode)
    {
	    spi_set_format(dev, 8, ((uint8_t)clockMode & 2) ? SPI_CPOL_1 : SPI_CPOL_0, ((uint8_t)clockMode & 1) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
        curClockMode = clockMode;
    }
}


HardwareSPI::HardwareSPI(spi_inst_t *hardware) noexcept : dev(hardware)
{
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
}


spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs) noexcept
{
    spi_status_t ret = SPI_OK;
    if (cs != NoPin) fastDigitalWriteLow(cs);
	const int bytesTransferred = (rx_data == nullptr) ? spi_write_blocking(dev, tx_data, len)
								: (tx_data == nullptr) ? spi_read_blocking(dev, 0xFF, rx_data, len)
									: spi_write_read_blocking(dev, tx_data, rx_data, len);
	ret = bytesTransferred == (int)len ? SPI_OK : SPI_ERROR;

    if (cs != NoPin) fastDigitalWriteHigh(cs);
    return ret;
}
