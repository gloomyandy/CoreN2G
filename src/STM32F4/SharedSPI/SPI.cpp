//Implement the SharedSpi as in RRF

#include "Core.h"
#include "SPI.h"

#include "SoftwareSPI.h"
#include "HardwareSPI.h"

SPI *SPI::getSSPDevice(SSPChannel channel)
{
    switch(channel)
    {
#ifdef RTOS
        case SSP1: return &HardwareSPI::SSP1; break;
#endif
        case SSP2: return &HardwareSPI::SSP2; break;
        case SSP3: return &HardwareSPI::SSP3; break;
#ifdef RTOS
        case SWSPI0: return &SoftwareSPI::SWSSP0; break;
        case SWSPI1: return &SoftwareSPI::SWSSP1; break;
        case SWSPI2: return &SoftwareSPI::SWSSP2; break;
#endif
        default: return nullptr;
    }
}

