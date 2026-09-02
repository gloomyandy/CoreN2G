//Hardware SPI
#include "HardwareSPI.h"
#include <DmacManager.h>
#include <hardware/dma.h>

#ifdef RTOS
#include <RTOSIface/RTOSIface.h>
#include <CoreNotifyIndices.h>
#endif


// Create SPI devices the actual configuration is set later
HardwareSPI HardwareSPI::HWSPI0(spi0);
HardwareSPI HardwareSPI::HWSPI1(spi1);

void HardwareSPI::initPins(Pin clk, Pin miso, Pin mosi, NvicPriority priority) noexcept
{
    if (clk != NoPin && miso != NoPin && mosi != NoPin)
    {
        SetPinFunction(clk, GpioPinFunction::Spi);
        SetPinFunction(miso, GpioPinFunction::Spi);
        SetPinFunction(mosi, GpioPinFunction::Spi);
    }
}

void HardwareSPI::initDma(DmaChannel txChan, DmaChannel rxChan) noexcept
{
    dma_channel_claim(rxChan);
    dma_channel_claim(txChan);
    useDma = true;
    this->rxChan = rxChan;
    this->txChan = txChan;
    DmacManager::SetInterruptCallback(rxChan, DmaCompleteCallback, CallbackParameter(this));
    DmacManager::EnableCompletedInterrupt(rxChan);
    dma_channel_config c = dma_channel_get_default_config(txChan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(dev, true));
    dma_channel_configure(txChan, &c, &spi_get_hw(dev)->dr, nullptr, 0, false);
    c = dma_channel_get_default_config(rxChan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(dev, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(rxChan, &c, nullptr, &spi_get_hw(dev)->dr, 0, false);
}

//setup the master device.
void HardwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    if (curBitRate != bitRate)
    {
        spi_init(dev, bitRate);
        curBitRate = bitRate;
    }
    if (clockMode != curClockMode)
    {
        spi_set_format(dev, 8, ((uint8_t)clockMode & 2) ? SPI_CPOL_1 : SPI_CPOL_0, ((uint8_t)clockMode & 1) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
        curClockMode = clockMode;
    }
}


HardwareSPI::HardwareSPI(spi_inst_t *hardware) noexcept : dev(hardware)
{
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
    useDma = false;
}

#if RP2350
spi_status_t __time_critical_func(HardwareSPI::transceivePacket)(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs) noexcept
#else
spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs) noexcept
#endif
{
    spi_status_t ret = SPI_OK;
    if (cs != NoPin) fastDigitalWriteLow(cs);
    if (useDma && tx_data != nullptr && rx_data != nullptr)
    {
        dma_channel_set_read_addr(txChan, tx_data, false);
        dma_channel_set_transfer_count(txChan, len, false);
        dma_channel_set_write_addr(rxChan, rx_data, false);
        dma_channel_set_transfer_count(rxChan, len, false);
        taskToWake = TaskBase::GetCallerTaskHandle();
        dma_start_channel_mask((1u << txChan) | (1u << rxChan));
        //dma_channel_wait_for_finish_blocking(rxChan);
        do {
            if (!TaskBase::TakeIndexed(NotifyIndices::Spi, 10))
            {
                ret = SPI_ERROR;
                taskToWake = nullptr;
                break;
            }
        } while (taskToWake != nullptr);
    }
    else
    {
        const int bytesTransferred = (rx_data == nullptr) ? spi_write_blocking(dev, tx_data, len)
                                        : (tx_data == nullptr) ? spi_read_blocking(dev, 0xFF, rx_data, len)
                                        : spi_write_read_blocking(dev, tx_data, rx_data, len);
        ret = bytesTransferred == (int)len ? SPI_OK : SPI_ERROR;
    }

    if (cs != NoPin) fastDigitalWriteHigh(cs);
    return ret;
}

#if RP2350
void __time_critical_func(HardwareSPI::DmaCompleteCallback)(CallbackParameter cp, DmaCallbackReason reason) noexcept
#else
void HardwareSPI::DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept
#endif
{
	TaskBase::GiveFromISR(static_cast<HardwareSPI *>(cp.vp)->taskToWake, NotifyIndices::Spi);
    static_cast<HardwareSPI *>(cp.vp)->taskToWake = nullptr;
}
