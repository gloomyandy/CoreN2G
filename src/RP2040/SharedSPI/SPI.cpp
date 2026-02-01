//Implement the SharedSpi as in RRF

#include "Core.h"
#include "SPI.h"

#include "PioSPI.h"
#include "HardwareSPI.h"

SPI *SPI::getSPIDevice(SPIChannel channel)
{
    switch(channel)
    {
        case SPI0: return &HardwareSPI::HWSPI0; break;
        case SPI1: return &HardwareSPI::HWSPI1; break;
        case SPI2: return &PioSPI::PIOSPI0; break;

        default: return nullptr;
    }
}

