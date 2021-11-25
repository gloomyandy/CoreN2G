/*
 * CoreTypes.h
 *
 *  Created on: 29 Oct 2021
 *      Author: David
 */

#ifndef SRC_CORETYPES_H_
#define SRC_CORETYPES_H_

#include <stdint.h>
#if defined(__STM32H7__)
# include <stm32h7.h>
#elif defined(__STM32F4__)
# include <stm32f4.h>
#elif defined(__LPC17xx__)
# include <lpc17xx.h>
#endif

// Core types used in interfaces and associated constants

typedef uint8_t DmaChannel;			///< A type that represents a DMA channel number
typedef uint8_t DmaPriority;		///< A type that represents a DMA priority
#if STM32F4
typedef PinName Pin;                ///< A type that represents an I/O pin on the microcontroller
#elif LPC17xx
typedef gpioPins_et Pin;            ///< A type that represents an I/O pin on the microcontroller
#else
typedef uint8_t Pin;				///< A type that represents an I/O pin on the microcontroller
#endif
typedef uint16_t PwmFrequency;		///< A type that represents a PWM frequency. 0 sometimes means "default".
typedef uint8_t CanAddress;			///< A type that represents the 7-bit CAN address of a board
typedef uint32_t NvicPriority;		///< A type that represents an interrupt priority
typedef uint8_t ExintNumber;		///< A type that represents an EXINT number (used on SAME5x/SAMC21 only)
typedef uint8_t EventNumber;		///< A type that represents an event number (used on SAME5x/SAMC21 only)
typedef __fp16 float16_t;			///< A 16-bit floating point type

#if STM32F4
static const Pin NoPin = NC;		///< A number that represents no I/O pin
#elif LPC17xx
static const Pin NoPin = P_NC; 		//which =0xff
#else
static const Pin NoPin = 0xFF;		///< A number that represents no I/O pin
static const Pin Nx = 0xFF;			///< A number that represents no I/O EXINT number
#endif

#endif /* SRC_CORETYPES_H_ */
