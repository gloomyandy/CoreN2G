/*******************************************************************************
SPI Driver:  
	Implementation

File Name:
    	drv_spi.c

Summary:
    	Implementation of MCU specific SPI functions.

Description:
	SDK NXP ECSPI Wrapper for MCP2517FD purposes.
 *******************************************************************************/

// Include files
#include "drv_spi.h"
#include "CoreImp.h"
#include "HardwareSPI.h"
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static Pin csPin;
static HardwareSPI *spiDev;
extern "C" void DRV_SPI_Initialize()
{
    csPin = PA_15;
    spiDev = &HardwareSPI::SSP3;
    pinMode(csPin, OUTPUT_HIGH);
    spiDev->configureDevice(8, 0, 10000000);
}


extern "C" int8_t DRV_SPI_TransferData(uint32_t index, uint8_t *SpiTxData, uint8_t *SpiRxData, size_t spiTransferSize)
{
    pinMode(csPin, OUTPUT_LOW);
    uint32_t ret = spiDev->transceivePacket((const uint8_t *) SpiTxData, (uint8_t *)SpiRxData, spiTransferSize);
    pinMode(csPin, OUTPUT_HIGH);
    //debugPrintf("SPI transfer len %d ret %d\n", spiTransferSize, ret);
    return (int8_t) ret;
}
