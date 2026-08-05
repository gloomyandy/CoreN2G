#ifndef STM32H7_H
#define STM32H7_H
#include <stm32_def.h>
#include <PinNames.h>
// Address of main RAM bank (excluding the area used for DMA buffers)
#define IRAM_ADDR 0x24000000
#define IRAM_SIZE 0x70000
#endif