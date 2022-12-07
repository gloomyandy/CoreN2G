//Hardware SPI
#include "HardwareSPI.h"

#ifdef RTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#endif
#include "spi_com.h"
#include "Cache.h"

/*
DMA Notes
The original version of this code only used DMA for all SPI devices. However during testing
I hit a problem when sharing the DMA2 unit between SPI1 and the software UART code. What appeared
to be happening was that write operations took places to the wrong GPIO pins, but this only
seemed to happen when the DMA2 unit was also in use by the SD access code (which uses SPI1).
The following Errata document describes problems which may be related to this issue (even though
we are not using the FiFO in this case):
https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405-407xx-and-stm32f415-417xx-device-limitations-stmicroelectronics.pdf

For now the solution I've choosen is to not use DMA for SPI1 (as it is not possible to DMA1 with
SPI1 and DMA1 is not able to access GPIO memory). This seems to work fine and since the SD card is
only used in a synchronous manner did not require any code to be restructured. It also seems to
be faster (I suspect because the SD card access code uses many short SPI operations and the DMA
setup is relatively large). It is probably worth testing the interrupt based version at some
point.

Andy - 6/8/2020
*/
#if USE_SSP1 || USE_SSP2 || USE_SSP3 || USE_SSP4 || USE_SSP5 || USE_SSP6
// Create SPI devices the actual configuration is set later
#if STM32H7
// On the H7 we need to make sure that and dma address is within a none cached memory area
extern uint32_t _nocache_ram_start;
extern uint32_t _nocache_ram_end;
static constexpr uint32_t MinDMALength = 4;
#define CAN_USE_DMA(ptr, len) ((ptr) == nullptr || (((const char *)(ptr) >= (const char *)&_nocache_ram_start) && ((const char *)(ptr) + (len) < (const char *)&_nocache_ram_end)))

// Create SPI devices the actual configuration is set later
#if USE_SSP1
HardwareSPI HardwareSPI::SSP1(SPI1);
#endif
#if USE_SSP2
HardwareSPI HardwareSPI::SSP2(SPI2, SPI2_IRQn, DMA1_Stream3, DMA_REQUEST_SPI2_RX, DMA1_Stream3_IRQn, DMA1_Stream4, DMA_REQUEST_SPI2_TX, DMA1_Stream4_IRQn);
#endif
#if USE_SSP3
HardwareSPI HardwareSPI::SSP3(SPI3, SPI3_IRQn, DMA1_Stream0, DMA_REQUEST_SPI3_RX, DMA1_Stream0_IRQn, DMA1_Stream5, DMA_REQUEST_SPI3_TX, DMA1_Stream5_IRQn);
#endif
#if USE_SSP4
HardwareSPI HardwareSPI::SSP4(SPI4, SPI4_IRQn, DMA1_Stream1, DMA_REQUEST_SPI4_RX, DMA1_Stream1_IRQn, DMA1_Stream2, DMA_REQUEST_SPI4_TX, DMA1_Stream2_IRQn);
#endif
#if USE_SSP5
HardwareSPI HardwareSPI::SSP5(SPI5);
#endif
#if USE_SSP6
HardwareSPI HardwareSPI::SSP6(SPI6);
#endif
#else
static constexpr uint32_t MinDMALength = 0;
extern uint32_t _sdata;
extern uint32_t _estack;
#define CAN_USE_DMA(ptr, len) ((ptr) == nullptr || (((const char *)(ptr) >= (const char *)&_sdata) && ((const char *)(ptr) + (len) < (const char *)&_estack)))

// Create SPI devices the actual configuration is set later
#if USE_SSP1
HardwareSPI HardwareSPI::SSP1(SPI1);
#endif
#if USE_SSP2
HardwareSPI HardwareSPI::SSP2(SPI2, SPI2_IRQn, DMA1_Stream3, DMA_CHANNEL_0, DMA1_Stream3_IRQn, DMA1_Stream4, DMA_CHANNEL_0, DMA1_Stream4_IRQn);
#endif
#if USE_SSP3
HardwareSPI HardwareSPI::SSP3(SPI3, SPI3_IRQn, DMA1_Stream0, DMA_CHANNEL_0, DMA1_Stream0_IRQn, DMA1_Stream5, DMA_CHANNEL_0, DMA1_Stream5_IRQn);
#endif
#endif

//#define SSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));


extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) noexcept
{
    // Get pointer to containing object
    HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
    s->transferActive = false;
    if (s->callback) s->callback(s);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) noexcept
{
    // Get pointer to containing object
    HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
    s->transferActive = false;
    if (s->callback) s->callback(s);
}    

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) noexcept
{
    // Get pointer to containing object
    HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
    s->transferActive = false;
    if (s->callback) s->callback(s);
}    

#if SPI1DMA
extern "C" void DMA2_Stream2_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP1.dmaRx));
}

extern "C" void DMA2_Stream3_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP1.dmaTx));
}
#endif

extern "C" void DMA1_Stream3_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP2.dmaRx));
}

extern "C" void DMA1_Stream4_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP2.dmaTx));
}

extern "C" void DMA1_Stream0_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP3.dmaRx));
}

extern "C" void DMA1_Stream5_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP3.dmaTx));
}

#if STM32H7
extern "C" void DMA1_Stream1_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP4.dmaRx));
}

extern "C" void DMA1_Stream2_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP4.dmaTx));
}

extern "C" void SPI1_IRQHandler()
{
    HAL_SPI_IRQHandler(&(HardwareSPI::SSP1.spi.handle));
}

extern "C" void SPI2_IRQHandler()
{
    HAL_SPI_IRQHandler(&(HardwareSPI::SSP2.spi.handle));
}

extern "C" void SPI3_IRQHandler()
{
    HAL_SPI_IRQHandler(&(HardwareSPI::SSP3.spi.handle));
}

extern "C" void SPI4_IRQHandler()
{
    HAL_SPI_IRQHandler(&(HardwareSPI::SSP4.spi.handle));
}

extern "C" void SPI5_IRQHandler()
{
    HAL_SPI_IRQHandler(&(HardwareSPI::SSP5.spi.handle));
}

extern "C" void SPI6_IRQHandler()
{
    HAL_SPI_IRQHandler(&(HardwareSPI::SSP6.spi.handle));
}
#endif

static inline void flushTxFifo(SPI_HandleTypeDef *sspDevice) noexcept
{

}

static inline void flushRxFifo(SPI_HandleTypeDef *hspi) noexcept
{
    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
    {
        /* read the received data */
#if STM32H7
        (void)*(__IO uint8_t *)&hspi->Instance->RXDR;
#else
        (void)*(__IO uint8_t *)&hspi->Instance->DR;
#endif
    }
}

void HardwareSPI::flushRx() noexcept
{
    flushRxFifo(&spi.handle);
}

// Disable the device and flush any data from the fifos
void HardwareSPI::disable() noexcept
{
    if (initComplete)
    {
        if (usingDma)
            HAL_SPI_DMAStop(&(spi.handle));
        flushRxFifo(&spi.handle);
        spi_deinit(&spi);
        initComplete = false;
        transferActive = false;
    }
}

// Wait for transmitter empty returning true if timed out
bool HardwareSPI::waitForTxEmpty() noexcept
{
    return false;
}

#ifdef RTOS
// Called on completion of a blocking transfer
void transferComplete(HardwareSPI *spiDevice) noexcept
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(spiDevice->waitingTask, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
#endif

void HardwareSPI::initPins(Pin clk, Pin miso, Pin mosi, Pin cs, NvicPriority priority) noexcept
{
    spi.pin_sclk = clk;
    spi.pin_miso = miso;
    spi.pin_mosi = mosi;
    spi.pin_ssel = csPin = cs;
    if (usingDma)
    {
        initDma(priority);   
    }
    initComplete = false;
}

void HardwareSPI::configureDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, uint32_t dir, uint32_t minc) noexcept
{
    hdma.Instance                 = inst;
#if STM32H7
    hdma.Init.Request             = chan;
#else    
    hdma.Init.Channel             = chan;
#endif
    hdma.Init.Direction           = dir;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = minc;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.Priority            = DMA_PRIORITY_LOW;
    hdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
}

void HardwareSPI::initDma(NvicPriority priority) noexcept
{    
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_DMA_Init(&dmaRx);
    NVIC_SetPriority(rxIrq, priority);
    NVIC_EnableIRQ(rxIrq);      
    __HAL_LINKDMA(&(spi.handle), hdmarx, dmaRx);
    HAL_DMA_Init(&dmaTx); 
    NVIC_SetPriority(txIrq, priority);
    NVIC_EnableIRQ(txIrq);      
    __HAL_LINKDMA(&(spi.handle), hdmatx, dmaTx);
#if STM32H7
    NVIC_SetPriority(spiIrq, priority);
    NVIC_EnableIRQ(spiIrq);
#endif
}

void HardwareSPI::configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, bool hardwareCS) noexcept
{
    Pin cs = (hardwareCS ? csPin : NoPin);
    if (!initComplete || bitRate != curBitRate || bits != curBits || clockMode != curClockMode )
    {
        if (initComplete)
        {
            if (usingDma)
                HAL_SPI_DMAStop(&(spi.handle));
            spi_deinit(&spi);
        }
        spi.pin_ssel = cs;
        spi_init(&spi, dev, deviceMode, bitRate, (spi_mode_e)clockMode, 1);
        initComplete = true;
        transferActive = false;
        curBitRate = bitRate;
        curBits = bits;
        curClockMode = clockMode;
    }
}

//setup the master device.
void HardwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    configureDevice(SPI_MODE_MASTER, bits, clockMode, bitRate, false);
}

HardwareSPI::HardwareSPI(SPI_TypeDef *spi, IRQn_Type spiIrqNo, DMA_Stream_TypeDef* rxStream, uint32_t rxChan, IRQn_Type rxIrqNo,
                            DMA_Stream_TypeDef* txStream, uint32_t txChan, IRQn_Type txIrqNo) noexcept : dev(spi), spiIrq(spiIrqNo), rxIrq(rxIrqNo), txIrq(txIrqNo), initComplete(false), transferActive(false), usingDma(true)
{
    configureDmaStream(dmaRx, rxStream, rxChan, DMA_PERIPH_TO_MEMORY, DMA_MINC_ENABLE);
    dmaRx.Init.Priority = DMA_PRIORITY_HIGH;
    configureDmaStream(dmaTx, txStream, txChan, DMA_MEMORY_TO_PERIPH, DMA_MINC_ENABLE);
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
}

HardwareSPI::HardwareSPI(SPI_TypeDef *spi) noexcept : dev(spi), initComplete(false), transferActive(false), usingDma(false)
{
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
}

void HardwareSPI::startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete) noexcept
{
    // FIXME consider setting dma burst size to 4 for WiFi and SBC transfers
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&(spi.handle));
    if (transferActive) debugPrintf("Warning attempt to start a DMA transfer when one already active\n");
    if (state != HAL_SPI_STATE_READY)
    {
        debugPrintf("SPI not ready %x\n", state);
        delay(100);
    }
    HAL_DMA_StateTypeDef dmaState = HAL_DMA_GetState(spi.handle.hdmarx);
    if (dmaState != HAL_DMA_STATE_READY)
    {
        debugPrintf("RX DMA not ready %x\n", dmaState);
        delay(100);
    }
    dmaState = HAL_DMA_GetState(spi.handle.hdmatx);
    if (dmaState != HAL_DMA_STATE_READY)
    {
        debugPrintf("TX DMA not ready %x\n", dmaState);
        delay(100);
    }

    HAL_StatusTypeDef status;    
    callback = ioComplete;
    transferActive = true;
    if (rx_data == nullptr)
    {
        Cache::FlushBeforeDMASend(tx_data, len);
        status = HAL_SPI_Transmit_DMA(&(spi.handle), (uint8_t *)tx_data, len);
    }
    else if (tx_data == nullptr)
    {
        Cache::FlushBeforeDMAReceive(rx_data, len);
        status = HAL_SPI_Receive_DMA(&(spi.handle), rx_data, len);
    }
    else
    {
        Cache::FlushBeforeDMASend(tx_data, len);
        Cache::FlushBeforeDMAReceive(rx_data, len);
        status = HAL_SPI_TransmitReceive_DMA(&(spi.handle), (uint8_t *)tx_data, rx_data, len);
    }
    if (status != HAL_OK)
        debugPrintf("SPI Error %d\n", (int)status);
}

void HardwareSPI::stopTransfer() noexcept
{
    // Stop a DMA transfer.
    // Note on the STM32F4 HAL_SPI_Abort does not
    // work because it leaves data in the TX fifo (which will not be clocked out 
    // because cs is not set). It seems that the only way to flush this fifo is
    // re-init the device, so we just do that.
    if (initComplete)
    {
        if (transferActive)
        {
#if STM32H7
            HAL_SPI_Abort(&(spi.handle));
            transferActive = false;
#else
            disable();
            configureDevice(spi.handle.Init.Mode, curBits, curClockMode, curBitRate, spi.pin_ssel != NoPin);
#endif
        }
        __HAL_SPI_DISABLE(&(spi.handle));
    }
}

void HardwareSPI::startTransferAndWait(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    HAL_StatusTypeDef status;
    transferActive = true;
    if (rx_data == nullptr)
        status = HAL_SPI_Transmit(&(spi.handle), (uint8_t *)tx_data, len, SPITimeoutMillis);
    else if (tx_data == nullptr)
        status = HAL_SPI_Receive(&(spi.handle), rx_data, len, SPITimeoutMillis);
    else
        status = HAL_SPI_TransmitReceive(&(spi.handle), (uint8_t *)tx_data, rx_data, len, SPITimeoutMillis);
    transferActive = false;
    if (status != HAL_OK)
        debugPrintf("SPI Error %d\n", (int)status);
}

spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    if (usingDma && len > MinDMALength && CAN_USE_DMA(tx_data, len) && CAN_USE_DMA(rx_data, len))
    {
#ifdef RTOS
if (waitingTask != 0) debugPrintf("SPI busy\n");
        waitingTask = xTaskGetCurrentTaskHandle();
        startTransfer(tx_data, rx_data, len, transferComplete);
        spi_status_t ret = SPI_OK;
        uint32_t start = millis();
        const TickType_t xDelay = SPITimeoutMillis / portTICK_PERIOD_MS; //timeout
        if( ulTaskNotifyTake(pdTRUE, xDelay) == 0) // timed out
        {
            ret = SPI_TIMEOUT;
            debugPrintf("SPI timeout delay %d actual %d active %d\n", xDelay, millis()-start, transferActive);
            stopTransfer();
        }
        waitingTask = 0;
#else
        startTransfer(tx_data, rx_data, len, nullptr);
        spi_status_t ret = SPI_OK;
        uint32_t start = millis();
        while(transferActive && millis() - start < SPITimeoutMillis)
        {
        }
        if (transferActive)
        {
            ret = SPI_TIMEOUT;
            debugPrintf("SPI timeout\n");
            stopTransfer();
        }
#endif
        if (rx_data != nullptr) Cache::InvalidateAfterDMAReceive(rx_data, len);
        return ret;
    }
    else
    {
        startTransferAndWait(tx_data, rx_data, len);
        return SPI_OK;
    }
}
#endif
