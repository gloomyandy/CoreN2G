/*******************************************************************************
SPI Driver:  
	Header File

File Name:
    	CanSpi.h

Summary:
    	This header file contains the MCU specific SPI 	definitions and declarations.

Description:
	SPI Wrapper for MCP2517FD purposes.
 *******************************************************************************/

#ifndef _DRV_SPI_H
#define _DRV_SPI_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>



#define SPI_DEFAULT_BUFFER_LENGTH       96

//! SPI Initialization
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

bool DRV_SPI_Initialize(void);

void DRV_SPI_Select(void);

void DRV_SPI_Deselect(void);


//! SPI Read/Write Transfer

int8_t DRV_SPI_TransferData(uint32_t index, uint8_t *SpiTxData, uint8_t *SpiRxData, size_t spiTransferSize);
#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif	// _DRV_SPI_H
