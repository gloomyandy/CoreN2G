/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#pragma once

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

#ifdef STM32F405RX
  #define STM32F4X_PIN_NUM  64  //64 pins mcu, 51 gpio
  #define STM32F4X_GPIO_NUM 51
  #define STM32F4X_ADC_NUM  16
#elif defined(STM32F407_5VX)
  #define STM32F4X_PIN_NUM  100  //100 pins mcu, 82 gpio
  #define STM32F4X_GPIO_NUM 82
  #define STM32F4X_ADC_NUM  16
#elif defined(STM32F407_5ZX)
  #define STM32F4X_PIN_NUM  144  //144 pins mcu, 114 gpio
  #define STM32F4X_GPIO_NUM 114
  #define STM32F4X_ADC_NUM  24
#elif defined(STM32F407IX)
  #define STM32F4X_PIN_NUM  176  //176 pins mcu, 140 gpio
  #define STM32F4X_GPIO_NUM 140
  #define STM32F4X_ADC_NUM  24
#else
  #error "no match MCU defined"
#endif

#define HSE_VALUE 8000000

// This must be a literal
#define NUM_DIGITAL_PINS        (STM32F4X_GPIO_NUM)
// This must be a literal with a value less than or equal to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       (STM32F4X_ADC_NUM)


#define SWDIO_PIN PA_13
#define SWCLK_PIN PA_14


