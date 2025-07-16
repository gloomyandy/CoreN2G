/*
 * GA: This information extracted from the file: CoreN2G\src\RP2040\pico-sdk\src\rp2_common\cmsis\stub\CMSIS\Device\RP2040\Include\RP2040.h
 * to avoid clashes with other files called RP2040.h when used in Core.h
*/


typedef enum {
/* =======================================  ARM Cortex-M0+ Specific Interrupt Numbers  ======================================= */
  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn              =  -1,              /*!< -1 System Tick Timer                                                      */
/* ===========================================  RP2040 Specific Interrupt Numbers  =========================================== */
  TIMER_IRQ_0_IRQn          =   0,              /*!< 0  TIMER_IRQ_0                                                            */
  TIMER_IRQ_1_IRQn          =   1,              /*!< 1  TIMER_IRQ_1                                                            */
  TIMER_IRQ_2_IRQn          =   2,              /*!< 2  TIMER_IRQ_2                                                            */
  TIMER_IRQ_3_IRQn          =   3,              /*!< 3  TIMER_IRQ_3                                                            */
  PWM_IRQ_WRAP_IRQn         =   4,              /*!< 4  PWM_IRQ_WRAP                                                           */
  USBCTRL_IRQ_IRQn          =   5,              /*!< 5  USBCTRL_IRQ                                                            */
  XIP_IRQ_IRQn              =   6,              /*!< 6  XIP_IRQ                                                                */
  PIO0_IRQ_0_IRQn           =   7,              /*!< 7  PIO0_IRQ_0                                                             */
  PIO0_IRQ_1_IRQn           =   8,              /*!< 8  PIO0_IRQ_1                                                             */
  PIO1_IRQ_0_IRQn           =   9,              /*!< 9  PIO1_IRQ_0                                                             */
  PIO1_IRQ_1_IRQn           =  10,              /*!< 10 PIO1_IRQ_1                                                             */
  DMA_IRQ_0_IRQn            =  11,              /*!< 11 DMA_IRQ_0                                                              */
  DMA_IRQ_1_IRQn            =  12,              /*!< 12 DMA_IRQ_1                                                              */
  IO_IRQ_BANK0_IRQn         =  13,              /*!< 13 IO_IRQ_BANK0                                                           */
  IO_IRQ_QSPI_IRQn          =  14,              /*!< 14 IO_IRQ_QSPI                                                            */
  SIO_IRQ_PROC0_IRQn        =  15,              /*!< 15 SIO_IRQ_PROC0                                                          */
  SIO_IRQ_PROC1_IRQn        =  16,              /*!< 16 SIO_IRQ_PROC1                                                          */
  CLOCKS_IRQ_IRQn           =  17,              /*!< 17 CLOCKS_IRQ                                                             */
  SPI0_IRQ_IRQn             =  18,              /*!< 18 SPI0_IRQ                                                               */
  SPI1_IRQ_IRQn             =  19,              /*!< 19 SPI1_IRQ                                                               */
  UART0_IRQ_IRQn            =  20,              /*!< 20 UART0_IRQ                                                              */
  UART1_IRQ_IRQn            =  21,              /*!< 21 UART1_IRQ                                                              */
  ADC_IRQ_FIFO_IRQn         =  22,              /*!< 22 ADC_IRQ_FIFO                                                           */
  I2C0_IRQ_IRQn             =  23,              /*!< 23 I2C0_IRQ                                                               */
  I2C1_IRQ_IRQn             =  24,              /*!< 24 I2C1_IRQ                                                               */
  RTC_IRQ_IRQn              =  25               /*!< 25 RTC_IRQ                                                                */
} IRQn_Type;

#define __CM0PLUS_REV                 0x0001U   /*!< CM0PLUS Core Revision                                                     */
#define __NVIC_PRIO_BITS               2        /*!< Number of Bits used for Priority Levels                                   */
#define __Vendor_SysTickConfig         0        /*!< Set to 1 if different SysTick Config is used                              */
#define __VTOR_PRESENT                 1        /*!< Set to 1 if CPU supports Vector Table Offset Register                     */
#define __MPU_PRESENT                  1        /*!< MPU present                                                               */
#define __FPU_PRESENT                  0        /*!< FPU present                                                               */
