#ifndef HARDWARESPI_H
#define HARDWARESPI_H

#include "CoreImp.h"
#include "SPI.h"

#include "FreeRTOS.h"
#include "task.h"
#include "spi_com.h"
#if SPI1DMA
extern "C" void DMA2_Stream2_IRQHandler(void) noexcept;
extern "C" void DMA2_Stream3_IRQHandler(void) noexcept;
#endif
extern "C" void DMA1_Stream3_IRQHandler(void) noexcept;
extern "C" void DMA1_Stream4_IRQHandler(void) noexcept;
extern "C" void DMA1_Stream0_IRQHandler(void) noexcept;
extern "C" void DMA1_Stream5_IRQHandler(void) noexcept;
#if STM32H7
extern "C" void DMA1_Stream1_IRQHandler() noexcept;
extern "C" void DMA1_Stream2_IRQHandler() noexcept;
extern "C" void SPI1_IRQHandler() noexcept;
extern "C" void SPI2_IRQHandler() noexcept;
extern "C" void SPI3_IRQHandler() noexcept;
extern "C" void SPI4_IRQHandler() noexcept;
extern "C" void SPI5_IRQHandler() noexcept;
extern "C" void SPI6_IRQHandler() noexcept;
#endif

class HardwareSPI;
typedef void (*SPICallbackFunction)(HardwareSPI *spiDevice) noexcept;
class HardwareSPI: public SPI
{
public:
    HardwareSPI(SPI_TypeDef *spi) noexcept;
    HardwareSPI(SPI_TypeDef *spi, IRQn_Type spiIrqNo, DMA_Stream_TypeDef* rxStream, uint32_t rxChan, IRQn_Type rxIrqNo,
                            DMA_Stream_TypeDef* txStream, uint32_t txChan, IRQn_Type txIrqNo) noexcept;
    spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept;
    bool waitForTxEmpty() noexcept;
    void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept; // Master mode
    void configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, bool hardwareCS) noexcept;
    void initPins(Pin clk, Pin miso, Pin mosi, Pin cs = NoPin) noexcept;
    void disable() noexcept;
    void flushRx() noexcept;
    void startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete) noexcept;
    void stopTransfer() noexcept;
    static HardwareSPI SSP1;
    static HardwareSPI SSP2;
    static HardwareSPI SSP3;
#if STM32H7
    static HardwareSPI SSP4;
    static HardwareSPI SSP5;
    static HardwareSPI SSP6;
#endif
private:
    spi_t spi;
    SPI_TypeDef *dev;
    IRQn_Type spiIrq;
    DMA_HandleTypeDef dmaRx;
    DMA_HandleTypeDef dmaTx;
    IRQn_Type rxIrq;
    IRQn_Type txIrq;
    Pin csPin;
    uint32_t curBitRate;
    uint32_t curBits;
    uint32_t curClockMode;
    SPICallbackFunction callback;
    TaskHandle_t waitingTask;
    bool initComplete;
    bool transferActive;
    bool usingDma;
    void configureDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, uint32_t dir, uint32_t minc) noexcept;
    void startTransferAndWait(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept;
    void initDma() noexcept;

#if SPI1DMA
    friend void DMA2_Stream2_IRQHandler() noexcept;
    friend void DMA2_Stream3_IRQHandler() noexcept;
#endif
    friend void DMA1_Stream3_IRQHandler() noexcept;
    friend void DMA1_Stream4_IRQHandler() noexcept;
    friend void DMA1_Stream0_IRQHandler() noexcept;
    friend void DMA1_Stream5_IRQHandler() noexcept;
    friend void transferComplete(HardwareSPI *spiDevice) noexcept;
    friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
    friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
#if STM32H7
    friend void DMA1_Stream1_IRQHandler() noexcept;
    friend void DMA1_Stream2_IRQHandler() noexcept;
    friend void SPI1_IRQHandler() noexcept;
    friend void SPI2_IRQHandler() noexcept;
    friend void SPI3_IRQHandler() noexcept;
    friend void SPI4_IRQHandler() noexcept;
    friend void SPI5_IRQHandler() noexcept;
    friend void SPI6_IRQHandler() noexcept;
#endif
    
};

#endif
