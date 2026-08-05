/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(const SpiParameters& params) noexcept : SpiDevice(params)
{
#if STM32BTC || RPXXXX
	static const char *name[] = {"SPI0", "SPI1", "SPI2", "SPI3", "SPI4", "SPI5", "SPI6", "SPI7", "SPI8"};
	mutex.Create(name[params.instanceNumber]);
#else
	mutex.Create("SPI");
#endif
}

// End
