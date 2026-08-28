/*
 * SpiDevice.cpp
 *
 *  Modified 23 Dec 2025 to support HW and PIO based SPI devices
 *      Author: Andy
 */

#include <SPI/SpiDevice.h>
SpiDevice::SpiDevice(const SpiParameters& params) noexcept
	: hardware(SPI::getSPIDevice((SPIChannel)params.instanceNumber))
{
	hardware->initPins(params.sclkPin, params.misoPin, params.mosiPin);
}

void SpiDevice::Disable() const noexcept
{
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	hardware->configureDevice(8, (uint32_t)mode, freq);
}

#if RP2350
bool __time_critical_func(SpiDevice::TransceivePacket)(const uint8_t* tx_data, uint8_t* rx_data, size_t len) noexcept
#else
bool SpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) noexcept
#endif
{
	return hardware->transceivePacket(tx_data, rx_data, len) == SPI_OK;
}

// End
