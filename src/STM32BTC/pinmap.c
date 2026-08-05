/*
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//Based on mbed-os/hal/mbed_pinmap_common.c
// Heavily modifed for use with RRF by Andy.

#include "pinmap.h"
#include "pinconfig.h"
#if defined(__STM32H7__)
# include "stm32h7xx_ll_gpio.h"
#else
# include "stm32f4xx_ll_gpio.h"
#endif

/* Map STM_PIN to LL */
const uint32_t pin_map_ll[16] = {
  LL_GPIO_PIN_0,
  LL_GPIO_PIN_1,
  LL_GPIO_PIN_2,
  LL_GPIO_PIN_3,
  LL_GPIO_PIN_4,
  LL_GPIO_PIN_5,
  LL_GPIO_PIN_6,
  LL_GPIO_PIN_7,
  LL_GPIO_PIN_8,
  LL_GPIO_PIN_9,
  LL_GPIO_PIN_10,
  LL_GPIO_PIN_11,
  LL_GPIO_PIN_12,
  LL_GPIO_PIN_13,
  LL_GPIO_PIN_14,
  LL_GPIO_PIN_15
};


/**
 * Configure pin (mode, speed, output type and pull-up/pull-down)
 */
void pin_function(PinName pin, int function)
{
  /* Get the pin informations */
  uint32_t mode  = STM_PIN_FUNCTION(function);
  uint32_t afnum = STM_PIN_AFNUM(function);
  uint32_t port = STM_PORT(pin);
  uint32_t ll_pin  = STM_LL_GPIO_PIN(pin);
  uint32_t ll_mode = 0;

  if (pin == (PinName)NC) {
    Error_Handler();
  }

  /* Enable GPIO clock */
  GPIO_TypeDef *gpio = set_GPIO_Port_Clock(port);

  /*  Set default speed to high.
   *  For most families there are dedicated registers so it is
   *  not so important, register can be set at any time.
   *  But for families like F1, speed only applies to output.
   */
#if defined (STM32F1xx)
  if (mode == STM_PIN_OUTPUT) {
#endif
#if 1
    LL_GPIO_SetPinSpeed(gpio, ll_pin, LL_GPIO_SPEED_FREQ_MEDIUM);
#else

#ifdef LL_GPIO_SPEED_FREQ_VERY_HIGH
    LL_GPIO_SetPinSpeed(gpio, ll_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
#else
    LL_GPIO_SetPinSpeed(gpio, ll_pin, LL_GPIO_SPEED_FREQ_HIGH);
#endif
#endif
#if defined (STM32F1xx)
  }
#endif


  switch (mode) {
    case STM_PIN_INPUT:
      ll_mode = LL_GPIO_MODE_INPUT;
      break;
    case STM_PIN_OUTPUT:
      ll_mode = LL_GPIO_MODE_OUTPUT;
      break;
    case STM_PIN_ALTERNATE:
      ll_mode = LL_GPIO_MODE_ALTERNATE;
      /* In case of ALT function, also set the afnum */
      pin_SetAFPin(gpio, pin, afnum);
      break;
    case STM_PIN_ANALOG:
      ll_mode = LL_GPIO_MODE_ANALOG;
      break;
    default:
      Error_Handler();
      break;
  }
  LL_GPIO_SetPinMode(gpio, ll_pin, ll_mode);

#if defined(GPIO_ASCR_ASC0)
  /* For families where Analog Control ASC0 register is present */
  if (STM_PIN_ANALOG_CONTROL(function)) {
    LL_GPIO_EnablePinAnalogControl(gpio, ll_pin);
  } else {
    LL_GPIO_DisablePinAnalogControl(gpio, ll_pin);
  }
#endif

  if ((mode == STM_PIN_OUTPUT) || (mode == STM_PIN_ALTERNATE)) {
    if (STM_PIN_OD(function)) {
      LL_GPIO_SetPinOutputType(gpio, ll_pin, LL_GPIO_OUTPUT_OPENDRAIN);
    } else {
      LL_GPIO_SetPinOutputType(gpio, ll_pin, LL_GPIO_OUTPUT_PUSHPULL);
    }
  }

  pin_PullConfig(gpio, ll_pin, STM_PIN_PUPD(function));

  pin_DisconnectDebug(pin);
}

/**
 * Configure pin speed
 */
void pin_speed(PinName pin, int speed)
{
  /* Get the pin informations */
  uint32_t port = STM_PORT(pin);
  uint32_t ll_pin  = STM_LL_GPIO_PIN(pin);

  if (pin == (PinName)NC) {
    Error_Handler();
  }

  /* Enable GPIO clock */
  GPIO_TypeDef *gpio = set_GPIO_Port_Clock(port);

  LL_GPIO_SetPinSpeed(gpio, ll_pin, speed);
}

const PinMap *pinmap_find_entry(void *peripheral, PinName pin, const PinMap *map)
{
  // Search for an entry matching both pin and peripheral. Null values act as wildcard
  if (map == NULL || (pin == NC && peripheral == NULL)) {
    return NULL;
  }
  while (map->pin != NC) {
    if ((pin == NC || map->pin == pin) && (peripheral == NULL || map->peripheral == peripheral)) {
      return map;
    }
    map++;
  }
  return NULL;
}
  
void pinmap_pinout(PinName pin, const PinMap *map)
{
  const PinMap * entry = pinmap_find_entry(NULL, pin, map);
  if (entry != NULL) {
      pin_function(pin, entry->function);
      return;
  }
  Error_Handler();
}

bool pinmap_pinout2(void *peripheral, PinName pin, const PinMap *map)
{
  if (pin == NC) {
    return true;
  }
  const PinMap * entry = pinmap_find_entry(peripheral, pin, map);
  if (entry != NULL) {
      pin_function(pin, entry->function);
      return true;
  }
  return false;
}

void *pinmap_peripheral(PinName pin, const PinMap *map)
{
  const PinMap * entry = pinmap_find_entry(NULL, pin, map);
  if (entry != NULL) {
    return entry->peripheral;
  }
  return NP;
}

uint32_t pinmap_function(PinName pin, const PinMap *map)
{
  const PinMap * entry = pinmap_find_entry(NULL, pin, map);
  if (entry != NULL) {
    return entry->function;
  }
  return (uint32_t)NC;
}

// Merge peripherals
void *pinmap_merge_peripheral(void *a, void *b)
{
  // both are the same (inc both NP)
  if (a == b) {
    return a;
  }

  // one (or both) is not set
  if (a == NP) {
    return b;
  }
  if (b == NP) {
    return a;
  }

  // mis-match error case
  return NP;
}
