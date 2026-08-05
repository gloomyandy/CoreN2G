
#include "stm32_def.h"
#include "Interrupts.h"


/* Private Types */
#define RISING_EDGE           (0x00100000U)
#define FALLING_EDGE          (0x00200000U)
#if STM32H7
#define IMR IMR1
#define RTSR RTSR1
#define FTSR FTSR1
#endif

/*As we can have only one interrupt/pin id, don't need to get the port info*/
typedef struct {
  IRQn_Type irqnb;
  StandardCallbackFunction callback;
  CallbackParameter param;
  uint32_t mode;
  Pin pin;
} gpio_irq_conf_str;

/* Private_Defines */
#define NB_EXTI   (16)

/* Private Variables */
static gpio_irq_conf_str gpio_irq_conf[NB_EXTI] = {
#if defined (STM32F0xx) || defined (STM32G0xx) || defined (STM32L0xx)
  {.irqnb = EXTI0_1_IRQn}, //GPIO_PIN_0
  {.irqnb = EXTI0_1_IRQn}, //GPIO_PIN_1
  {.irqnb = EXTI2_3_IRQn}, //GPIO_PIN_2
  {.irqnb = EXTI2_3_IRQn}, //GPIO_PIN_3
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_4
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_5
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_6
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_7
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_8
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_9
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_10
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_11
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_12
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_13
  {.irqnb = EXTI4_15_IRQn}, //GPIO_PIN_14
  {.irqnb = EXTI4_15_IRQn}  //GPIO_PIN_15
#else
  {.irqnb = EXTI0_IRQn}, //GPIO_PIN_0
  {.irqnb = EXTI1_IRQn}, //GPIO_PIN_1
  {.irqnb = EXTI2_IRQn}, //GPIO_PIN_2
  {.irqnb = EXTI3_IRQn}, //GPIO_PIN_3
  {.irqnb = EXTI4_IRQn}, //GPIO_PIN_4
  {.irqnb = EXTI9_5_IRQn}, //GPIO_PIN_5
  {.irqnb = EXTI9_5_IRQn}, //GPIO_PIN_6
  {.irqnb = EXTI9_5_IRQn}, //GPIO_PIN_7
  {.irqnb = EXTI9_5_IRQn}, //GPIO_PIN_8
  {.irqnb = EXTI9_5_IRQn}, //GPIO_PIN_9
  {.irqnb = EXTI15_10_IRQn}, //GPIO_PIN_10
  {.irqnb = EXTI15_10_IRQn}, //GPIO_PIN_11
  {.irqnb = EXTI15_10_IRQn}, //GPIO_PIN_12
  {.irqnb = EXTI15_10_IRQn}, //GPIO_PIN_13
  {.irqnb = EXTI15_10_IRQn}, //GPIO_PIN_14
  {.irqnb = EXTI15_10_IRQn}  //GPIO_PIN_15
#endif
};


inline void EXTI_Callback(uint32_t bit, uint32_t entry)
{
  if (__HAL_GPIO_EXTI_GET_IT(bit) != 0x00U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(bit);

    if (gpio_irq_conf[entry].callback)
    {
      gpio_irq_conf[entry].callback(gpio_irq_conf[entry].param);
    }
  }
}



#ifdef __cplusplus
extern "C" {
#endif
/**
  * @brief This function handles external line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  EXTI_Callback(GPIO_PIN_0, 0);
}

/**
  * @brief This function handles external line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  EXTI_Callback(GPIO_PIN_1, 1);
}

/**
  * @brief This function handles external line 2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  EXTI_Callback(GPIO_PIN_2, 2);
}

/**
  * @brief This function handles external line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
  EXTI_Callback(GPIO_PIN_3, 3);
}

/**
  * @brief This function handles external line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
  EXTI_Callback(GPIO_PIN_4, 4);
}


/**
  * @brief This function handles external line 5 to 9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  uint32_t pin;
  uint32_t entry = 5;
  for (pin = GPIO_PIN_5; pin <= GPIO_PIN_9; pin = pin << 1) {
    EXTI_Callback(pin, entry);
    entry++;
  }
}

/**
  * @brief This function handles external line 10 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  uint32_t pin;
  uint32_t entry = 10;
  for (pin = GPIO_PIN_10; pin <= GPIO_PIN_15; pin = pin << 1) {
    EXTI_Callback(pin, entry);
    entry++;
  }
}

#ifdef __cplusplus
}
#endif

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static void setEXTI(Pin pin, uint32_t mode)
{
  uint32_t bit = STM_GPIO_PIN(pin);
  uint32_t temp;
  // enable/disable interrupt
  temp = EXTI->IMR;
  temp &= ~(bit);
  if (mode)
  {
    temp |= bit;
  }
  EXTI->IMR = temp;
  // Configure falling/rising edges
  temp = EXTI->RTSR;
  temp &= ~(bit);
  if ((mode & RISING_EDGE) == RISING_EDGE)
  {
    temp |= bit;
  }
  EXTI->RTSR = temp;

  temp = EXTI->FTSR;
  temp &= ~(bit);
  if ((mode & FALLING_EDGE) == FALLING_EDGE)
  {
    temp |= bit;
  }
  EXTI->FTSR = temp;
}

static void setSYSCFG(Pin pin)
{
  uint32_t port = STM_PORT(pin);
  uint32_t pos = STM_PIN(pin);
  uint32_t temp;

  temp = SYSCFG->EXTICR[pos >> 2U];
  temp &= ~(0x0FUL << (4U * (pos & 0x03U)));
  temp |= (port << (4U * (pos & 0x03U)));
  SYSCFG->EXTICR[pos >> 2U] = temp;
}

void initInterruptPins() noexcept
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  for(uint32_t i = 0; i < NB_EXTI; i++)
  {
    gpio_irq_conf[i].pin = NoPin;
    gpio_irq_conf[i].callback = NULL;
    setEXTI((Pin)i, 0);
    HAL_NVIC_EnableIRQ(gpio_irq_conf[i].irqnb);
  }
}

void EnablePinInterrupt(Pin pin) noexcept
{
  if (gpio_irq_conf[STM_PIN(pin)].pin != pin)
    return;
  // Enable EXTI on this pin
  setSYSCFG(pin);
  setEXTI(pin, gpio_irq_conf[STM_PIN(pin)].mode);
}

void DisablePinInterrupt(Pin pin) noexcept
{
  if (gpio_irq_conf[STM_PIN(pin)].pin != pin)
    return;
  // Disable EXTI on this pin
  setEXTI(pin, 0);

}

bool AttachPinInterrupt(Pin pin, StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param, bool enable) noexcept
{
  if (pin == NoPin)
  {
    return false;
  }
  uint32_t it_mode;
  switch (mode) {
    case InterruptMode::change :
      it_mode = GPIO_MODE_IT_RISING_FALLING;
      break;
    case InterruptMode::falling :
    case InterruptMode::low :
      it_mode = GPIO_MODE_IT_FALLING;
      break;
    case InterruptMode::rising :
    case InterruptMode::high :
      it_mode = GPIO_MODE_IT_RISING;
      break;
    default:
      it_mode = GPIO_MODE_IT_RISING;
      break;
  }
  if (gpio_irq_conf[STM_PIN(pin)].pin != NoPin && gpio_irq_conf[STM_PIN(pin)].pin != pin)
  {
    debugPrintf("Unable to attach interrupt for pin %c.%d interrupt already used by pin %c.%d\n", (int)('A'+STM_PORT(pin)), (int)STM_PIN(pin),
                     (int)('A'+STM_PORT(gpio_irq_conf[STM_PIN(pin)].pin)), (int)STM_PIN(gpio_irq_conf[STM_PIN(pin)].pin));
    return false;
  } 
  gpio_irq_conf[STM_PIN(pin)].callback = callback;
  gpio_irq_conf[STM_PIN(pin)].param = param;
  gpio_irq_conf[STM_PIN(pin)].pin = pin;
  gpio_irq_conf[STM_PIN(pin)].mode = it_mode;

  if (enable)
    EnablePinInterrupt(pin);
  return true;
}


void DetachPinInterrupt(Pin pin) noexcept
{
  if (gpio_irq_conf[STM_PIN(pin)].pin == pin)
  {
    DisablePinInterrupt(pin);
    gpio_irq_conf[STM_PIN(pin)].pin = NoPin;
    gpio_irq_conf[STM_PIN(pin)].callback = NULL;
  }
}

Pin getAttachedPin(uint32_t id) noexcept
{
  if (id < NB_EXTI)
    return gpio_irq_conf[id].pin;
  else
    return NoPin;
}
