
#if USE_SDIO
#include "HardwareSDIO.h"
#include "CoreImp.h"
#ifdef RTOS
#include "FreeRTOS.h"
#include <CoreNotifyIndices.h>
#include "semphr.h"
#include "task.h"
#endif

#if STM32H7
#define SDIO_CLOCK_EDGE_RISING SDMMC_CLOCK_EDGE_RISING
#define SDIO_CLOCK_POWER_SAVE_DISABLE SDMMC_CLOCK_POWER_SAVE_DISABLE
#define SDIO_BUS_WIDE_1B SDMMC_BUS_WIDE_1B
#define SDIO_HARDWARE_FLOW_CONTROL_DISABLE SDMMC_HARDWARE_FLOW_CONTROL_DISABLE
#define SDIO_HARDWARE_FLOW_CONTROL_ENABLE SDMMC_HARDWARE_FLOW_CONTROL_ENABLE
#define SDIO_BUS_WIDE_4B SDMMC_BUS_WIDE_4B
#else
#endif

#define USE_SD_HIGHSPEED 0

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

HardwareSDIO HardwareSDIO::SDIO1;

extern "C" void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsdio)
{
  HardwareSDIO::SDIO1.ioComplete = true;
#ifdef RTOS
  TaskBase::GiveFromISR(HardwareSDIO::SDIO1.waitingTask, NotifyIndices::Sdio);
#endif
}

extern "C" void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsdio)
{
  HardwareSDIO::SDIO1.ioComplete = true;
#ifdef RTOS
  TaskBase::GiveFromISR(HardwareSDIO::SDIO1.waitingTask, NotifyIndices::Sdio);
#endif
}    

#if STM32H7
extern "C" void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
}

extern "C" void SDMMC1_IRQHandler()
{
  HAL_SD_IRQHandler(&(HardwareSDIO::SDIO1.hsd));
}
#else

extern "C" void DMA2_Stream3_IRQHandler()
{
  HAL_DMA_IRQHandler(&(HardwareSDIO::SDIO1.dmaTxRx));    
}

extern "C" void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
}

extern "C" void SDIO_IRQHandler()
{
  HAL_SD_IRQHandler(&(HardwareSDIO::SDIO1.hsd));
}

void HardwareSDIO::initDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, IRQn_Type irq, NvicPriority prio, uint32_t dir, uint32_t minc) noexcept
{
  hdma.Instance                 = inst;
  
  hdma.Init.Channel             = chan;

  hdma.Init.Direction           = dir;
  hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma.Init.MemInc              = minc;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma.Init.Mode                = DMA_PFCTRL;
  hdma.Init.Priority            = DMA_PRIORITY_LOW;
  hdma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
  hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma.Init.MemBurst            = DMA_MBURST_INC4;
  hdma.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  if (HAL_DMA_Init(&hdma) != HAL_OK)
  {
    debugPrintf("SDIO Failed to init DMA\n");
    delay(5000);
  }
  NVIC_SetPriority(irq, prio);
  NVIC_EnableIRQ(irq);      
}
#endif

HardwareSDIO::HardwareSDIO() noexcept
{   
}

bool HardwareSDIO::waitReady(uint32_t timeout) noexcept
{
  uint32_t start = millis();
  while((HAL_SD_GetCardState(&hsd) & 0xff) != HAL_SD_CARD_TRANSFER)
  {
    uint32_t elapsed = millis() - start;
    if (elapsed > 100)
    {
#if RTOS
      delay(1);
#endif
      if (elapsed > timeout)
      {
        return false;
      }
    }
  }
  return true;
}

uint8_t HardwareSDIO::initCard() noexcept
{
  uint8_t sd_state = MSD_OK;

  int retryCnt = 0;
  do {
#if STM32H7
    __HAL_RCC_SDMMC1_FORCE_RESET();
    delay(10);
    __HAL_RCC_SDMMC1_RELEASE_RESET();
    hsd.Instance = SDMMC1;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = SDMMC_INIT_CLK_DIV;
#else
    __HAL_RCC_SDIO_FORCE_RESET();
    delay(10);
    __HAL_RCC_SDIO_RELEASE_RESET();
    hsd.Instance = SDIO;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = SDIO_INIT_CLK_DIV;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
#endif
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    HAL_SD_DeInit(&hsd);
    delay(5);
    sd_state = HAL_SD_Init(&hsd);
    // HAL_SD_Init does not report an error if there is no card to talk to, HAL_SD_InitCard
    // does, so call that to check.
    if (sd_state == MSD_OK)
    {
      sd_state = HAL_SD_InitCard(&hsd);
    }

    // Configure SD high speed and bus width
    if (sd_state == MSD_OK && retryCnt < 2)
    {
      // Use highspeed mode on STM32H7. The errata for STM32F4 basically says this does not work when using a 48MHz clock
      // which we need as it is shared with USB.
#if STM32H7
      sd_state = HAL_SD_ConfigSpeedBusOperation(&hsd, SDMMC_SPEED_MODE_AUTO);
      if (sd_state == HAL_OK)
      {
        // Switch clock speed to high speed
        hsd.Init.ClockDiv = SDMMC_HSPEED_CLK_DIV;
      }
      else
      {
        debugPrintf("Failed to select high speed mode. error %d code %x\n", sd_state, (unsigned)HAL_SD_GetError(&hsd));
        hsd.Init.ClockDiv = SDMMC_NSPEED_CLK_DIV;
      }
#else
      hsd.Init.ClockDiv = SDIO_TRANSFER_CLK_DIV;
#endif 
      // Enable wide operation which will also set the clock speed
      sd_state = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
      if (sd_state != HAL_OK) 
      {
        debugPrintf("Failed to select wide bus mode ret %x code %x\n", sd_state, (unsigned)HAL_SD_GetError(&hsd));
      }
    }
    if (sd_state == MSD_OK && !waitReady(1000))
    {
      sd_state = MSD_ERROR;
    }
  } while (sd_state != MSD_OK && retryCnt++ < 2);
  return sd_state == MSD_OK ? MSD_OK : MSD_ERROR;
}

void HardwareSDIO::InitPins(NvicPriority priority) noexcept
{
  this->priority = priority;
}

/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t HardwareSDIO::Init() noexcept
{
  uint8_t sd_state = MSD_OK;
  /* Check if the SD card is plugged in the slot */
  if (IsDetected() != SD_PRESENT) {
    return MSD_ERROR;
  }
#if STM32H7
  __HAL_RCC_SDMMC1_CLK_ENABLE();
#else
  __HAL_RCC_SDIO_CLK_ENABLE();
#endif
  pinmap_pinout(PC_8, PinMap_SD);
  pinmap_pinout(PC_9, PinMap_SD);
  pinmap_pinout(PC_10, PinMap_SD);
  pinmap_pinout(PC_11, PinMap_SD);
  pinmap_pinout(PC_12, PinMap_SD);
  pinmap_pinout(PD_2, PinMap_SD);
#if STM32H7
  NVIC_SetPriority(SDMMC1_IRQn, priority);
  NVIC_EnableIRQ(SDMMC1_IRQn);      
#else
  NVIC_SetPriority(SDIO_IRQn, priority);
  NVIC_EnableIRQ(SDIO_IRQn);
  // DMA setup
  __HAL_RCC_DMA2_CLK_ENABLE();
  initDmaStream(dmaTxRx, DMA2_Stream3, DMA_CHANNEL_4, DMA2_Stream3_IRQn, priority, DMA_PERIPH_TO_MEMORY, DMA_MINC_ENABLE);
  __HAL_LINKDMA(&hsd, hdmarx, dmaTxRx);
  __HAL_LINKDMA(&hsd, hdmatx, dmaTxRx);
#endif
#ifdef RTOS
  waitingTask = 0;
#endif
sd_state = initCard();
return sd_state;
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t HardwareSDIO::ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout) noexcept
{
  uint8_t sd_state = MSD_OK;
  if (!waitReady(Timeout))
  {
      return MSD_ERROR;
  }
  #ifdef RTOS
  waitingTask = TaskBase::GetCallerTaskHandle();
#else
  uint32_t start = millis();
#endif
  ioComplete = false;
  HAL_StatusTypeDef stat = HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  if (stat != HAL_OK) {
    debugPrintf("SDIO Read %d len %d error %d code %x\n", (int)ReadAddr, (int)NumOfBlocks, stat, (unsigned)HAL_SD_GetError(&hsd));
    return MSD_ERROR;
  }
  // The SBC code can sometimes spam us with task notifications, check that our operation has finished
  while (!ioComplete)
  {
#ifdef RTOS
    if(!TaskBase::TakeIndexed(NotifyIndices::Sdio, Timeout)) // timed out
#else
    if (millis() - start > Timeout)
#endif
    {
        sd_state = MSD_ERROR;
        debugPrintf("SDIO Read SD timeout\n");
        break;
    }
  }
  #ifdef RTOS
  waitingTask = 0;
  #endif
  return sd_state;
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t HardwareSDIO::WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout) noexcept
{
  uint8_t sd_state = MSD_OK;
  if (!waitReady(Timeout))
  {
      return MSD_ERROR;
  }
#ifdef RTOS
  waitingTask = TaskBase::GetCallerTaskHandle();
#else
  uint32_t start = millis();
#endif
  ioComplete = false;
  HAL_StatusTypeDef stat = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  if (stat != HAL_OK) {
    debugPrintf("SDIO Write %d len %d return %d error %x\n", (int)WriteAddr, (int)NumOfBlocks, stat, (unsigned)HAL_SD_GetError(&hsd));
    return MSD_ERROR;
  }
  // The SBC code can sometimes spam us with task notifications, check that our operation has finished
  while (!ioComplete)
  {
#ifdef RTOS
    if(!TaskBase::TakeIndexed(NotifyIndices::Sdio, Timeout)) // timed out
#else
    if (millis() - start > Timeout)
#endif
    {
        sd_state = MSD_ERROR;
        debugPrintf("SDIO Write SD timeout\n");
        break;
    }

  }
  #ifdef RTOS
  waitingTask = 0;
  #endif

  return sd_state;
}

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t HardwareSDIO::GetCardState(void) noexcept
{
  return ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ?
    SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval clock frequency
  */
uint32_t HardwareSDIO::GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo) noexcept
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd, CardInfo);
#if STM32H7
  return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC)/(hsd.Init.ClockDiv == 0 ? 1 : 2*hsd.Init.ClockDiv);
#else
  return (48*1000000)/(hsd.Init.ClockBypass == SDIO_CLOCK_BYPASS_ENABLE ? 1 : hsd.Init.ClockDiv + 2);
#endif
}


/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
uint8_t HardwareSDIO::IsDetected(void) noexcept
{
  __IO uint8_t status = SD_PRESENT;
  return status;
}
#endif