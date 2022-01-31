#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

// This must be a literal with the same value as PEND
#define NUM_DIGITAL_PINS        140

// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       24

#define HSE_VALUE 25000000

#define SWDIO_PIN PA_13
#define SWCLK_PIN PA_14

#endif /* _VARIANT_ARDUINO_STM32_ */
