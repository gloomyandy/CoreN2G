/*
  Copyright (c) 2017 Daniel Fekete

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  Copyright (c) 2019 STMicroelectronics
  Heavily modified for RRF by Andy (c) 2024
*/

#include <CoreImp.h>
#include "HardwareTimer.h"

#ifdef HAL_TIM_MODULE_ENABLED

/* Private Defines */
#define PIN_NOT_USED 0xFF
#define MAX_RELOAD ((1 << 16) - 1) // Currently even 32b timers are used as 16b to have generic behavior
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

/**
  * @brief  Enable the timer clock
  * @param  htim: TIM handle
  * @retval None
  */
static void enableTimerClock(TIM_HandleTypeDef *htim) noexcept
{
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
#endif
#if defined(TIM2_BASE)
  if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
#endif
#if defined(TIM3_BASE)
  if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
#endif
#if defined(TIM4_BASE)
  if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
#endif
#if defined(TIM5_BASE)
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
  }
#endif
#if defined(TIM6_BASE)
  if (htim->Instance == TIM6) {
    __HAL_RCC_TIM6_CLK_ENABLE();
  }
#endif
#if defined(TIM7_BASE)
  if (htim->Instance == TIM7) {
    __HAL_RCC_TIM7_CLK_ENABLE();
  }
#endif
#if defined(TIM8_BASE)
  if (htim->Instance == TIM8) {
    __HAL_RCC_TIM8_CLK_ENABLE();
  }
#endif
#if defined(TIM9_BASE)
  if (htim->Instance == TIM9) {
    __HAL_RCC_TIM9_CLK_ENABLE();
  }
#endif
#if defined(TIM10_BASE)
  if (htim->Instance == TIM10) {
    __HAL_RCC_TIM10_CLK_ENABLE();
  }
#endif
#if defined(TIM11_BASE)
  if (htim->Instance == TIM11) {
    __HAL_RCC_TIM11_CLK_ENABLE();
  }
#endif
#if defined(TIM12_BASE)
  if (htim->Instance == TIM12) {
    __HAL_RCC_TIM12_CLK_ENABLE();
  }
#endif
#if defined(TIM13_BASE)
  if (htim->Instance == TIM13) {
    __HAL_RCC_TIM13_CLK_ENABLE();
  }
#endif
#if defined(TIM14_BASE)
  if (htim->Instance == TIM14) {
    __HAL_RCC_TIM14_CLK_ENABLE();
  }
#endif
#if defined(TIM15_BASE)
  if (htim->Instance == TIM15) {
    __HAL_RCC_TIM15_CLK_ENABLE();
  }
#endif
#if defined(TIM16_BASE)
  if (htim->Instance == TIM16) {
    __HAL_RCC_TIM16_CLK_ENABLE();
  }
#endif
#if defined(TIM17_BASE)
  if (htim->Instance == TIM17) {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }
#endif
#if defined(TIM18_BASE)
  if (htim->Instance == TIM18) {
    __HAL_RCC_TIM18_CLK_ENABLE();
  }
#endif
#if defined(TIM19_BASE)
  if (htim->Instance == TIM19) {
    __HAL_RCC_TIM19_CLK_ENABLE();
  }
#endif
#if defined(TIM20_BASE)
  if (htim->Instance == TIM20) {
    __HAL_RCC_TIM20_CLK_ENABLE();
  }
#endif
#if defined(TIM21_BASE)
  if (htim->Instance == TIM21) {
    __HAL_RCC_TIM21_CLK_ENABLE();
  }
#endif
#if defined(TIM22_BASE)
  if (htim->Instance == TIM22) {
    __HAL_RCC_TIM22_CLK_ENABLE();
  }
#endif
}

#if 0
/**
  * @brief  Disable the timer clock
  * @param  htim: TIM handle
  * @retval None
  */
static void disableTimerClock(TIM_HandleTypeDef *htim) noexcept
{
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
#endif
#if defined(TIM2_BASE)
  if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
#endif
#if defined(TIM3_BASE)
  if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
#endif
#if defined(TIM4_BASE)
  if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_DISABLE();
  }
#endif
#if defined(TIM5_BASE)
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_DISABLE();
  }
#endif
#if defined(TIM6_BASE)
  if (htim->Instance == TIM6) {
    __HAL_RCC_TIM6_CLK_DISABLE();
  }
#endif
#if defined(TIM7_BASE)
  if (htim->Instance == TIM7) {
    __HAL_RCC_TIM7_CLK_DISABLE();
  }
#endif
#if defined(TIM8_BASE)
  if (htim->Instance == TIM8) {
    __HAL_RCC_TIM8_CLK_DISABLE();
  }
#endif
#if defined(TIM9_BASE)
  if (htim->Instance == TIM9) {
    __HAL_RCC_TIM9_CLK_DISABLE();
  }
#endif
#if defined(TIM10_BASE)
  if (htim->Instance == TIM10) {
    __HAL_RCC_TIM10_CLK_DISABLE();
  }
#endif
#if defined(TIM11_BASE)
  if (htim->Instance == TIM11) {
    __HAL_RCC_TIM11_CLK_DISABLE();
  }
#endif
#if defined(TIM12_BASE)
  if (htim->Instance == TIM12) {
    __HAL_RCC_TIM12_CLK_DISABLE();
  }
#endif
#if defined(TIM13_BASE)
  if (htim->Instance == TIM13) {
    __HAL_RCC_TIM13_CLK_DISABLE();
  }
#endif
#if defined(TIM14_BASE)
  if (htim->Instance == TIM14) {
    __HAL_RCC_TIM14_CLK_DISABLE();
  }
#endif
#if defined(TIM15_BASE)
  if (htim->Instance == TIM15) {
    __HAL_RCC_TIM15_CLK_DISABLE();
  }
#endif
#if defined(TIM16_BASE)
  if (htim->Instance == TIM16) {
    __HAL_RCC_TIM16_CLK_DISABLE();
  }
#endif
#if defined(TIM17_BASE)
  if (htim->Instance == TIM17) {
    __HAL_RCC_TIM17_CLK_DISABLE();
  }
#endif
#if defined(TIM18_BASE)
  if (htim->Instance == TIM18) {
    __HAL_RCC_TIM18_CLK_DISABLE();
  }
#endif
#if defined(TIM19_BASE)
  if (htim->Instance == TIM19) {
    __HAL_RCC_TIM19_CLK_DISABLE();
  }
#endif
#if defined(TIM20_BASE)
  if (htim->Instance == TIM20) {
    __HAL_RCC_TIM20_CLK_DISABLE();
  }
#endif
#if defined(TIM21_BASE)
  if (htim->Instance == TIM21) {
    __HAL_RCC_TIM21_CLK_DISABLE();
  }
#endif
#if defined(TIM22_BASE)
  if (htim->Instance == TIM22) {
    __HAL_RCC_TIM22_CLK_DISABLE();
  }
#endif
}
#endif

/**
  * @brief  This function return the timer clock source.
  * @param  tim: timer instance
  * @retval 1 = PCLK1 or 2 = PCLK2
  */
static uint8_t getTimerClkSrc(TIM_TypeDef *tim) noexcept
{
  uint8_t clkSrc = 0;

  if (tim != (TIM_TypeDef *)NC)
#ifdef STM32F0xx
    /* TIMx source CLK is PCKL1 */
    clkSrc = 1;
#else
  {
    /* Get source clock depending on TIM instance */
    switch ((uint32_t)tim) {
#if defined(TIM2_BASE)
      case TIM2_BASE:
#endif
#if defined(TIM3_BASE)
      case TIM3_BASE:
#endif
#if defined(TIM4_BASE)
      case TIM4_BASE:
#endif
#if defined(TIM5_BASE)
      case TIM5_BASE:
#endif
#if defined(TIM6_BASE)
      case TIM6_BASE:
#endif
#if defined(TIM7_BASE)
      case TIM7_BASE:
#endif
#if defined(TIM12_BASE)
      case TIM12_BASE:
#endif
#if defined(TIM13_BASE)
      case TIM13_BASE:
#endif
#if defined(TIM14_BASE)
      case TIM14_BASE:
#endif
#if defined(TIM18_BASE)
      case TIM18_BASE:
#endif
        clkSrc = 1;
        break;
#if defined(TIM1_BASE)
      case TIM1_BASE:
#endif
#if defined(TIM8_BASE)
      case TIM8_BASE:
#endif
#if defined(TIM9_BASE)
      case TIM9_BASE:
#endif
#if defined(TIM10_BASE)
      case TIM10_BASE:
#endif
#if defined(TIM11_BASE)
      case TIM11_BASE:
#endif
#if defined(TIM15_BASE)
      case TIM15_BASE:
#endif
#if defined(TIM16_BASE)
      case TIM16_BASE:
#endif
#if defined(TIM17_BASE)
      case TIM17_BASE:
#endif
#if defined(TIM19_BASE)
      case TIM19_BASE:
#endif
#if defined(TIM20_BASE)
      case TIM20_BASE:
#endif
#if defined(TIM21_BASE)
      case TIM21_BASE:
#endif
#if defined(TIM22_BASE)
      case TIM22_BASE:
#endif
        clkSrc = 2;
        break;
      default:
        _Error_Handler("TIM: Unknown timer instance", (int)tim);
        break;
    }
  }
#endif
  return clkSrc;
}

static uint32_t get_pwm_channel(PinName pin)
{
  uint32_t function = pinmap_function(pin, PinMap_PWM);
  uint32_t channel = 0;
  switch (STM_PIN_CHANNEL(function)) {
    case 1:
      channel = TIM_CHANNEL_1;
      break;
    case 2:
      channel = TIM_CHANNEL_2;
      break;
    case 3:
      channel = TIM_CHANNEL_3;
      break;
    case 4:
      channel = TIM_CHANNEL_4;
      break;
    default:
      channel = 0;
      break;
  }
  return channel;
}

static uint32_t get_timer_id(TIM_TypeDef *instance) noexcept
{

#if defined(TIM1_BASE)
  if (instance == TIM1) {
    return 1;
  }
#endif
#if defined(TIM2_BASE)
  if (instance == TIM2) {
    return 2;
  }
#endif
#if defined(TIM3_BASE)
  if (instance == TIM3) {
    return 3;
  }
#endif
#if defined(TIM4_BASE)
  if (instance == TIM4) {
    return 4;
  }
#endif
#if defined(TIM5_BASE)
  if (instance == TIM5) {
    return 5;
  }
#endif
#if defined(TIM6_BASE)
  if (instance == TIM6) {
    return 6;
  }
#endif
#if defined(TIM7_BASE)
  if (instance == TIM7) {
    return 7;
  }
#endif
#if defined(TIM8_BASE)
  if (instance == TIM8) {
    return 8;
  }
#endif
#if defined(TIM9_BASE)
  if (instance == TIM9) {
    return 9;
  }
#endif
#if defined(TIM10_BASE)
  if (instance == TIM10) {
    return 10;
  }
#endif
#if defined(TIM11_BASE)
  if (instance == TIM11) {
    return 11;
  }
#endif
#if defined(TIM12_BASE)
  if (instance == TIM12) {
    return 12;
  }
#endif
#if defined(TIM13_BASE)
  if (instance == TIM13) {
    return 13;
  }
#endif
#if defined(TIM14_BASE)
  if (instance == TIM14) {
    return 14;
  }
#endif
#if defined(TIM15_BASE)
  if (instance == TIM15) {
    return 15;
  }
#endif
#if defined(TIM16_BASE)
  if (instance == TIM16) {
    return 16;
  }
#endif
#if defined(TIM17_BASE)
  if (instance == TIM17) {
    return 17;
  }
#endif
#if defined(TIM18_BASE)
  if (instance == TIM18) {
    return 18;
  }
#endif
#if defined(TIM19_BASE)
  if (instance == TIM19) {
    return 19;
  }
#endif
#if defined(TIM20_BASE)
  if (instance == TIM20) {
    return 20;
  }
#endif
#if defined(TIM21_BASE)
  if (instance == TIM21) {
    return 21;
  }
#endif
#if defined(TIM22_BASE)
  if (instance == TIM22) {
    return 22;
  }
#endif
  return UNKNOWN_TIMER;
}


/**
  * @brief  HardwareTimer constructor: set default configuration values
  * @param  Timer instance ex: TIM1, ...
  * @retval None
  */
HardwareTimer::HardwareTimer(TIM_TypeDef *instance) noexcept
{
  uint32_t index = get_timer_id(instance);
  if (index == UNKNOWN_TIMER) {
    Error_Handler();
  }

   handle.Instance = instance;
   handle.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
   handle.hdma[0] = NULL;
   handle.hdma[1] = NULL;
   handle.hdma[2] = NULL;
   handle.hdma[3] = NULL;
   handle.hdma[4] = NULL;
   handle.hdma[5] = NULL;
   handle.hdma[6] = NULL;
   handle.Lock = HAL_UNLOCKED;
   handle.State = HAL_TIM_STATE_RESET;

   handle.Instance = instance;

  // Enable Timer clock
  enableTimerClock(&( handle));

  // Configure HAL structure for all channels
  for (int i = 0; i < TIMER_CHANNELS; i++) {
    OCMode[i] = TIMER_NOT_USED;
  }
}

/**
  * @brief  Pause HardwareTimer: stop timer
  * @param  None
  * @retval None
  */
void HardwareTimer::pause() noexcept
{
  HAL_TIM_Base_Stop_IT(&( handle));
}

/**
  * @brief  Start or resume HardwareTimer: all channels are resumed, interrupts are enabled if necessary
  * @param  None
  * @retval None
  */
void HardwareTimer::resume(void) noexcept
{
   handle.Init.CounterMode = TIM_COUNTERMODE_UP;
   handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
#if defined(TIM_RCR_REP)
   handle.Init.RepetitionCounter = 0;
#endif
   handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&( handle));

  HAL_TIM_Base_Start(&( handle));

  resumeChannel(1);
  resumeChannel(2);
  resumeChannel(3);
  resumeChannel(4);
}

/**
  * @brief  Convert arduino  into HAL channel
  * @param  Arduino channel [1..4]
  * @retval HAL channel. return -1 if arduino channel is invalid
  */
int HardwareTimer::getChannel(uint32_t channel) noexcept
{
  uint32_t return_value;

  switch (channel) {
    case 1:
      return_value = TIM_CHANNEL_1;
      break;
    case 2:
      return_value = TIM_CHANNEL_2;
      break;
    case 3:
      return_value = TIM_CHANNEL_3;
      break;
    case 4:
      return_value = TIM_CHANNEL_4;
      break;
    default:
      return_value = -1;
  }
  return return_value;
}

/**
  * @brief  Configure specified channel and resume/start timer
  * @param  Arduino channel [1..4]
  * @retval None
  */
void HardwareTimer::resumeChannel(uint32_t channel) noexcept
{
  int timChannel = getChannel(channel);
  if (timChannel == -1) {
    Error_Handler();
  }
  if (IS_TIM_PWM_MODE(OCMode[channel - 1])) {
    TIM_OC_InitTypeDef channelOC;
    memset(&channelOC, 0, sizeof(channelOC));
    switch (timChannel)
    {
    case TIM_CHANNEL_1:
      channelOC.Pulse =  handle.Instance->CCR1;
      break;
    case TIM_CHANNEL_2:
      channelOC.Pulse =  handle.Instance->CCR2;
      break;
    case TIM_CHANNEL_3:
      channelOC.Pulse =  handle.Instance->CCR3;
      break;
    case TIM_CHANNEL_4:
      channelOC.Pulse =  handle.Instance->CCR4;
      break;
    }
    channelOC.OCMode = OCMode[channel -1];
    HAL_TIM_PWM_ConfigChannel(&( handle), &channelOC, timChannel);
    HAL_TIM_PWM_Start(&( handle), timChannel);
  }
}


/**
  * @brief  Configure hardwareTimer prescaler
  * @param  prescaler factor
  * @retval None
  */
void HardwareTimer::setPrescaleFactor(uint32_t prescaler) noexcept
{
  // Hardware register correspond to prescaler-1. Example PSC register value 0 means divided by 1
  __HAL_TIM_SET_PRESCALER(& handle, prescaler - 1);
   handle.Init.Prescaler = prescaler - 1;
}


/**
  * @brief  Set overflow (rollover)
  * @param  overflow: depend on format parameter
  * @param  format of overflow parameter. If ommited default format is Tick
  *           TICK_FORMAT:     overflow is the number of tick for overflow
  *           MICROSEC_FORMAT: overflow is the number of microsecondes for overflow
  *           HERTZ_FORMAT:    overflow is the frequency in hertz for overflow
  * @retval None
  */
void HardwareTimer::setOverflow(uint32_t overflow, TimerFormat_t format) noexcept
{
  uint32_t ARR_RegisterValue;
  uint32_t Prescalerfactor;
  uint32_t period_cyc;
  // Remark: Hardware register correspond to period count-1. Example ARR register value 9 means period of 10 timer cycle
  switch (format) {
    case MICROSEC_FORMAT:
      period_cyc = overflow * (getTimerClkFreq() / 1000000);
      Prescalerfactor = (period_cyc / 0x10000) + 1;
      __HAL_TIM_SET_PRESCALER(& handle, Prescalerfactor - 1);
       handle.Init.Prescaler = Prescalerfactor - 1;
      ARR_RegisterValue = (period_cyc / Prescalerfactor) - 1;
      break;
    case HERTZ_FORMAT:
      period_cyc = getTimerClkFreq() / overflow;
      Prescalerfactor = (period_cyc / 0x10000) + 1;
      __HAL_TIM_SET_PRESCALER(& handle, Prescalerfactor - 1);
       handle.Init.Prescaler = Prescalerfactor - 1;
      ARR_RegisterValue = (period_cyc / Prescalerfactor) - 1;
      break;
    case TICK_FORMAT:
    default :
      ARR_RegisterValue = overflow - 1;
      break;
  }

  __HAL_TIM_SET_AUTORELOAD(& handle, ARR_RegisterValue);
   handle.Init.Period = ARR_RegisterValue;
}

/**
  * @brief  Retreive timer counter value
  * @param  format of returned value. If ommited default format is Tick
  * @retval overflow depending on format value:
  *           TICK_FORMAT:     return number of tick for counter
  *           MICROSEC_FORMAT: return number of microsecondes for counter
  *           HERTZ_FORMAT:    return frequency in hertz for counter
  */
uint32_t HardwareTimer::getCount(TimerFormat_t format) noexcept
{
  uint32_t CNT_RegisterValue = __HAL_TIM_GET_COUNTER(&( handle));
  uint32_t Prescalerfactor =  handle.Instance->PSC + 1;
  uint32_t return_value;
  switch (format) {
    case MICROSEC_FORMAT:
      return_value = (uint32_t)((CNT_RegisterValue * Prescalerfactor * 1000000.0) / getTimerClkFreq());
      break;
    case HERTZ_FORMAT:
      return_value = (uint32_t)(getTimerClkFreq() / (CNT_RegisterValue  * Prescalerfactor));
      break;
    case TICK_FORMAT:
    default :
      return_value = CNT_RegisterValue;
      break;
  }
  return return_value;
}

/**
  * @brief  Set timer counter value
  * @param  counter: depend on format parameter
  * @param  format of overflow parameter. If ommited default format is Tick
  *           TICK_FORMAT:     counter is the number of tick
  *           MICROSEC_FORMAT: counter is the number of microsecondes
  *           HERTZ_FORMAT:    counter is the frequency in hertz
  * @retval None
  */
void HardwareTimer::setCount(uint32_t counter, TimerFormat_t format) noexcept
{
  uint32_t CNT_RegisterValue;
  uint32_t Prescalerfactor =  handle.Instance->PSC + 1;
  switch (format) {
    case MICROSEC_FORMAT:
      CNT_RegisterValue = ((counter * (getTimerClkFreq() / 1000000)) / Prescalerfactor) - 1 ;
      break;
    case HERTZ_FORMAT:
      CNT_RegisterValue = (uint32_t)((getTimerClkFreq() / (counter * Prescalerfactor)) - 1);
      break;
    case TICK_FORMAT:
    default :
      CNT_RegisterValue = counter - 1;
      break;
  }
  __HAL_TIM_SET_COUNTER(&( handle), CNT_RegisterValue);
}

/**
  * @brief  Set channel mode
  * @param  channel: Arduino channel [1..4]
  * @param  mode: mode configuration for the channel (see TimerModes_t)
  * @param  pin: pin name, ex: PB_0
  * @retval None
  */
void HardwareTimer::setMode(uint32_t channel, TimerModes_t mode, PinName pin) noexcept
{
  if (getChannel(channel) == -1) {
    Error_Handler();
  }

  switch (mode) {
    case TIMER_OUTPUT_COMPARE_PWM1:
      OCMode[channel - 1] = TIM_OCMODE_PWM1;
      break;

    case TIMER_DISABLED:
      OCMode[channel - 1] = TIMER_NOT_USED;
      break;

    default:
      debugPrintf("Warning attempt to use timer mode %d\n", mode);
      break;
  }

  if (pin != NC) {
    if ((int)get_pwm_channel(pin) == getChannel(channel)) {
      /* Configure PWM GPIO pins */
      pinmap_pinout(pin, PinMap_PWM);
    } else {
      // Pin doesn't match with timer output channels
      Error_Handler();
    }
  }
}

/**
  * @brief  Set channel Capture/Compare register
  * @param  channel: Arduino channel [1..4]
  * @param  compare: compare value depending on format
  * @param  format of compare parameter. If ommited default format is Tick
  *           TICK_FORMAT:     compare is the number of tick
  *           MICROSEC_FORMAT: compare is the number of microsecondes
  *           HERTZ_FORMAT:    compare is the frequency in hertz
  * @retval None
  */
void HardwareTimer::setCaptureCompare(uint32_t channel, uint32_t compare, TimerCompareFormat_t format) noexcept
{
  int timChannel = getChannel(channel);
  uint32_t Prescalerfactor =  handle.Instance->PSC + 1;
  uint32_t CCR_RegisterValue;

  if (timChannel == -1) {
    Error_Handler();
  }

  switch (format) {
    case MICROSEC_COMPARE_FORMAT:
      CCR_RegisterValue = ((compare * (getTimerClkFreq() / 1000000)) / Prescalerfactor) - 1 ;
      break;
    case HERTZ_COMPARE_FORMAT:
      CCR_RegisterValue = (getTimerClkFreq() / (compare * Prescalerfactor)) - 1;
      break;
    case PERCENT_COMPARE_FORMAT:
      CCR_RegisterValue = ((__HAL_TIM_GET_AUTORELOAD(&( handle)) + 1) * compare) / 100;
      break;
    case RESOLUTION_8B_COMPARE_FORMAT:
      CCR_RegisterValue = ((__HAL_TIM_GET_AUTORELOAD(&( handle)) + 1) * compare) / 255 ;
      break;
    case RESOLUTION_12B_COMPARE_FORMAT:
      CCR_RegisterValue = ((__HAL_TIM_GET_AUTORELOAD(&( handle)) + 1) * compare) / 4095 ;
      break;
    case TICK_COMPARE_FORMAT:
    default :
      CCR_RegisterValue = compare - 1;
      break;
  }

  __HAL_TIM_SET_COMPARE(&( handle), timChannel, CCR_RegisterValue);
}


/**
  * @brief  HardwareTimer destructor
  * @retval None
  */
HardwareTimer::~HardwareTimer() noexcept
{
  enableTimerClock(&( handle));
}

/**
  * @brief  return timer index from timer handle
  * @param  htim : one of the defined timer
  * @retval None
  */


/**
  * @brief  This function return the timer clock frequency.
  * @param  tim: timer instance
  * @retval frequency in Hz
  */
uint32_t HardwareTimer::getTimerClkFreq() noexcept
{
  RCC_ClkInitTypeDef    clkconfig = {};
  uint32_t              pFLatency = 0U;
  uint32_t              uwTimclock = 0U, uwAPBxPrescaler = 0U;

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  switch (getTimerClkSrc(handle.Instance)) {
    case 1:
      uwAPBxPrescaler = clkconfig.APB1CLKDivider;
      uwTimclock = HAL_RCC_GetPCLK1Freq();
      break;
#if !defined(STM32F0xx) && !defined(STM32G0xx)
    case 2:
      uwAPBxPrescaler = clkconfig.APB2CLKDivider;
      uwTimclock = HAL_RCC_GetPCLK2Freq();
      break;
#endif
    default:
    case 0: // Unknown timer clock source
      Error_Handler();
      break;
  }

#if defined(STM32H7xx)
  /* When TIMPRE bit of the RCC_CFGR register is reset,
   *   if APBx prescaler is 1 or 2 then TIMxCLK = HCLK,
   *   otherwise TIMxCLK = 2x PCLKx.
   * When TIMPRE bit in the RCC_CFGR register is set,
   *   if APBx prescaler is 1,2 or 4, then TIMxCLK = HCLK,
   *   otherwise TIMxCLK = 4x PCLKx
   */
  RCC_PeriphCLKInitTypeDef PeriphClkConfig = {};
  HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkConfig);

  if (PeriphClkConfig.TIMPresSelection == RCC_TIMPRES_ACTIVATED) {
    switch (uwAPBxPrescaler) {
      default:
      case RCC_APB1_DIV1:
      case RCC_APB1_DIV2:
      case RCC_APB1_DIV4:
      /* case RCC_APB2_DIV1: */
      case RCC_APB2_DIV2:
      case RCC_APB2_DIV4:
        uwTimclock = HAL_RCC_GetHCLKFreq();
        break;
      case RCC_APB1_DIV8:
      case RCC_APB1_DIV16:
      case RCC_APB2_DIV8:
      case RCC_APB2_DIV16:
        uwTimclock *= 4;
        break;
    }
  } else {
    switch (uwAPBxPrescaler) {
      default:
      case RCC_APB1_DIV1:
      case RCC_APB1_DIV2:
      /* case RCC_APB2_DIV1: */
      case RCC_APB2_DIV2:
        // uwTimclock*=1;
        uwTimclock = HAL_RCC_GetHCLKFreq();
        break;
      case RCC_APB1_DIV4:
      case RCC_APB1_DIV8:
      case RCC_APB1_DIV16:
      case RCC_APB2_DIV4:
      case RCC_APB2_DIV8:
      case RCC_APB2_DIV16:
        uwTimclock *= 2;
        break;
    }
  }
#else
  /* When TIMPRE bit of the RCC_DCKCFGR register is reset,
   *   if APBx prescaler is 1, then TIMxCLK = PCLKx,
   *   otherwise TIMxCLK = 2x PCLKx.
   * When TIMPRE bit in the RCC_DCKCFGR register is set,
   *   if APBx prescaler is 1,2 or 4, then TIMxCLK = HCLK,
   *   otherwise TIMxCLK = 4x PCLKx
   */
#if defined(STM32F4xx) || defined(STM32F7xx)
#if !defined(STM32F405xx) && !defined(STM32F415xx) &&\
    !defined(STM32F407xx) && !defined(STM32F417xx)
  RCC_PeriphCLKInitTypeDef PeriphClkConfig = {};
  HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkConfig);

  if (PeriphClkConfig.TIMPresSelection == RCC_TIMPRES_ACTIVATED)
    switch (uwAPBxPrescaler) {
      default:
      case RCC_HCLK_DIV1:
      case RCC_HCLK_DIV2:
      case RCC_HCLK_DIV4:
        uwTimclock = HAL_RCC_GetHCLKFreq();
        break;
      case RCC_HCLK_DIV8:
      case RCC_HCLK_DIV16:
        uwTimclock *= 4;
        break;
    } else
#endif
#endif
    switch (uwAPBxPrescaler) {
      default:
      case RCC_HCLK_DIV1:
        // uwTimclock*=1;
        break;
      case RCC_HCLK_DIV2:
      case RCC_HCLK_DIV4:
      case RCC_HCLK_DIV8:
      case RCC_HCLK_DIV16:
        uwTimclock *= 2;
        break;
    }
#endif /* STM32H7xx */
  return uwTimclock;
}

TIM_HandleTypeDef* HardwareTimer::getHandle() noexcept
{
  return &handle;
}


uint32_t HardwareTimer::getTimerId() noexcept
{
  return get_timer_id(handle.Instance);
}

#endif // HAL_TIM_MODULE_ENABLED
