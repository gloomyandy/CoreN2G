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
#if STM32 || RPXXXX
	static char name[] = "SPI0";
	name[3] = '0' + params.instanceNumber;
	mutex.Create(name);
#else
	mutex.Create("SPI");
#endif
}

// End
