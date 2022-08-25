/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PinAF_STM32F1.h"
#include "interrupt.h"
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
constexpr uint32_t MaxInterruptPins = 16;
Pin attachedPins[MaxInterruptPins];

void initInterruptPins() noexcept
{
  for(uint32_t i = 0; i < MaxInterruptPins; i++)
    attachedPins[i] = NoPin;
}

bool attachInterrupt(Pin pin, StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param) noexcept
{
#if !defined(HAL_EXTI_MODULE_DISABLED)
  uint32_t it_mode;
  PinName p = pin;
  GPIO_TypeDef *port = set_GPIO_Port_Clock(STM_PORT(p));
  if (!port) {
    return false;
  }
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

#ifdef STM32F1xx
  pinF1_DisconnectDebug(p);
#endif /* STM32F1xx */
  if (!stm32_interrupt_enable(port, STM_GPIO_PIN(p), callback, it_mode, param))
  {
    debugPrintf("Failed to attach interrupt to pin %c.%d\n", (int)('A'+STM_PORT(p)), (int)STM_PIN(p));
    if (attachedPins[STM_PIN(p)] != NoPin)
      debugPrintf("Old pin %c.%d new %c.%d\n", (int)('A'+STM_PORT(attachedPins[STM_PIN(p)])), (int)STM_PIN(attachedPins[STM_PIN(p)]), (int)('A'+STM_PORT(p)), (int)STM_PIN(p));
    return false;
  }
  attachedPins[STM_PIN(p)] = pin;
#else
  UNUSED(pin);
  UNUSED(callback);
  UNUSED(mode);
#endif
return true;
}


void detachInterrupt(Pin pin) noexcept
{
#if !defined(HAL_EXTI_MODULE_DISABLED)
  PinName p = pin;
  GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));
  if (!port) {
    return;
  }
  if (attachedPins[STM_PIN(p)] == pin)
  {
    stm32_interrupt_disable(port, STM_GPIO_PIN(p));
    attachedPins[STM_PIN(p)] = NoPin;
  }
#else
  UNUSED(pin);
#endif
}

Pin getAttachedPin(uint32_t id) noexcept
{
  if (id < MaxInterruptPins)
    return attachedPins[id];
  else
    return NoPin;
}
