/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 13 Mar 2021
 *      Author: Andy
 */

#include <I2C/SharedI2CMaster.h>

#ifdef RTOS					// we dn't support I2C in non-RTOS builds
#define USE_I2C_DMA		(0)

constexpr uint32_t DefaultSharedI2CClockFrequency = 400000;
constexpr uint32_t I2CTimeoutTicks = 100;
constexpr uint32_t ShutdownTimeoutMillis = 50;

SharedI2CMaster::SharedI2CMaster(const I2cParameters& params) noexcept
	: hardware((params.instanceNumber == 0) ? i2c0 : i2c1), taskWaiting(nullptr), state(I2cState::idle)
{
	errors.Clear();
	mutex.Create("I2C");
	SetPinFunction(params.sclPin, GpioPinFunction::I2c);
	SetPinFunction(params.sdaPin, GpioPinFunction::I2c);
	currentClockRate = DefaultSharedI2CClockFrequency;
	Enable();
}

// Resetting the processor part way through a transfer leaves the slave device driving SDA low, which hangs the bus until the next power cycle
// Not sure if needed on RPXXXX
void SharedI2CMaster::End() noexcept
{
	(void)Take(ShutdownTimeoutMillis);											// if we time out then the bus is already stuck, so reset anyway
	Disable();
}


// Set the I2C clock frequency. Caller must own the mutex first.
void SharedI2CMaster::SetClockFrequency(uint32_t freq) noexcept
{
	if (freq != currentClockRate)
	{
		// We have to disable I2C device in order to change the baud rate
		Disable();
		currentClockRate = freq;
		Enable();
	}
}

void SharedI2CMaster::Enable() const noexcept
{
	i2c_init(hardware, currentClockRate);
}

void SharedI2CMaster::Disable() const noexcept
{
	i2c_deinit(hardware);
}

// Write then read data. Caller must own the mutex first.
bool SharedI2CMaster::Transfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	// If an empty transfer, nothing to do
	if (numToRead + numToWrite == 0)
	{
		return true;
	}

	for (unsigned int triesDone = 0; triesDone < 3; ++triesDone)
	{
		if (InternalTransfer(address, txBuffer, rxBuffer, numToWrite, numToRead))
		{
			return true;
		}

		// Had an I2C error, so re-initialise
		Disable();
		Enable();
	}
	return false;
}

// Get ownership of this I2C interface, return true if successful
bool SharedI2CMaster::Take(uint32_t timeout) noexcept
{
	const bool success = mutex.Take(timeout);
	if (!success)
	{
		++errors.contentions;
	}
	return success;
}

void SharedI2CMaster::Release() noexcept
{
	// Now that Transfer() has an option to not release the bus, we may have called Take() several times, so we may need to call Release() several times
	while (mutex.GetHolder() == TaskBase::GetCallerTaskHandle())
	{
		mutex.Release();
	}
}

bool SharedI2CMaster::InternalTransfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	if (numToWrite > 0)
	{
		//debugPrintf("i2c write len %d addr %x\n", numToWrite, address);
		int ret = i2c_write_timeout_us(hardware, address, txBuffer, numToWrite, numToRead > 0, I2CTimeoutTicks*1000);
		if (ret <= 0)
		{
			if (ret == PICO_ERROR_TIMEOUT) 
				errors.busErrors++;
			else if (ret == PICO_ERROR_GENERIC)
				errors.naks++;
			else
				errors.otherErrors++;
			
			return false;
		}
	}
	if (numToRead > 0)
	{
		//debugPrintf("i2c read len %d addr %x\n", numToWrite, address);
		int ret = i2c_read_timeout_us(hardware, address, rxBuffer, numToRead, false, I2CTimeoutTicks*1000);
		if (ret <= 0)
		{
			if (ret == PICO_ERROR_TIMEOUT) 
				errors.busErrors++;
			else if (ret == PICO_ERROR_GENERIC)
				errors.naks++;
			else
				errors.otherErrors++;
			return false;
		}
	}
	return true;
}

void SharedI2CMaster::ProtocolError() noexcept
{

}

#endif

// End
