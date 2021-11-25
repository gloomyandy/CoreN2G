//Implement the SharedSpi as in RRF

#include "Core.h"
#include "SPI.h"

#include "SoftwareSPI.h"
#include "HardwareSPI.h"

SPI *SPI::getSSPDevice(SSPChannel channel)
{
    switch(channel)
    {
        case SSP1: return &HardwareSPI::SSP1; break;
        case SSP2: return &HardwareSPI::SSP2; break;
        case SSP3: return &HardwareSPI::SSP3; break;
        case SWSPI0: return &SoftwareSPI::SWSSP0; break;
        case SWSPI1: return &SoftwareSPI::SWSSP1; break;
        case SWSPI2: return &SoftwareSPI::SWSSP2; break;
#if STM32H7
        case SSP4: return &HardwareSPI::SSP4; break;
        case SSP5: return &HardwareSPI::SSP5; break;
        case SSP6: return &HardwareSPI::SSP6; break;
#endif
        default: return nullptr;
    }
}

