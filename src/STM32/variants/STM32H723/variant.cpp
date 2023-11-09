#include <CoreImp.h>
#ifdef __cplusplus
extern "C" {
#endif
#if 0
// Pin number
const PinName digitalPin[] = {
  PE_2,  // D0
  PE_3,  // D1
  PE_4,  // D2
  PE_5,  // D3
  PE_6,  // D4
  PI_8,  // D5
  PC_13, // D6
  PC_14, // D7
  PC_15, // D8
  PI_9,  // D9
  PI_10, // D10
  PI_11, // D11
  PF_0,  // D12
  PF_1,  // D13
  PF_2,  // D14
  PH_0,  // D15
  PH_1,  // D16
  PB_2,  // D17
  PF_15, // D18
  PG_0,  // D19
  PG_1,  // D20
  PE_7,  // D21
  PE_8,  // D22
  PE_9,  // D23
  PE_10, // D24
  PE_11, // D25
  PE_12, // D26
  PE_13, // D27
  PE_14, // D28
  PE_15, // D29
  PB_10, // D30
  PB_11, // D31
  PH_6,  // D32
  PH_7,  // D33
  PH_8,  // D34
  PH_9,  // D35
  PH_10, // D36
  PH_11, // D37
  PH_12, // D38
  PB_12, // D39
  PB_13, // D40
  PB_14, // D41
  PB_15, // D42
  PD_8,  // D43
  PD_9,  // D44
  PD_10, // D45
  PD_11, // D46
  PD_12, // D47
  PD_13, // D48
  PD_14, // D49
  PD_15, // D50
  PG_2,  // D51
  PG_3,  // D52
  PG_4,  // D53
  PG_5,  // D54
  PG_6,  // D55
  PG_7,  // D56
  PG_8,  // D57
  PC_6,  // D58
  PC_7,  // D59
  PC_8,  // D60
  PC_9,  // D61
  PA_8,  // D62
  PA_9,  // D63
  PA_10, // D64
  PA_11, // D65
  PA_12, // D66
  PA_13, // D67
  PH_13, // D68
  PH_14, // D69
  PH_15, // D70
  PI_0,  // D71
  PI_1,  // D72
  PI_2,  // D73
  PI_3,  // D74
  PA_14, // D75
  PA_15, // D76
  PC_10, // D77
  PC_11, // D78
  PC_12, // D79
  PD_0,  // D80
  PD_1,  // D81
  PD_2,  // D82
  PD_3,  // D83
  PD_4,  // D84
  PD_5,  // D85
  PD_6,  // D86
  PD_7,  // D87
  PG_9,  // D88
  PG_10, // D89
  PG_11, // D90
  PG_12, // D91
  PG_13, // D92
  PG_14, // D93
  PG_15, // D94
  PB_3,  // D95
  PB_4,  // D96
  PB_5,  // D97
  PB_6,  // D98
  PB_7,  // D99
  PB_8,  // D100
  PB_9,  // D101
  PE_0,  // D102
  PE_1,  // D103
  PI_4,  // D104
  PI_5,  // D105
  PI_6,  // D106
  PI_7,  // D107
  PA_0,  // D108 / A0
  PA_1,  // D109 / A1
  PA_2,  // D110 / A2
  PA_3,  // D111 / A3
  PA_4,  // D112 / A4
  PA_5,  // D113 / A5
  PA_6,  // D114 / A6
  PA_7,  // D115 / A7
  PB_0,  // D116 / A8
  PB_1,  // D117 / A9
  PH_2,  // D118 / A10
  PH_3,  // D119 / A11
  PH_4,  // D120 / A12
  PH_5,  // D121 / A13
  PC_0,  // D122 / A14
  PC_1,  // D123 / A15
  PC_2,  // D124 / A16
  PC_3,  // D125 / A17
  PC_4,  // D126 / A18
  PC_5,  // D127 / A19
  PF_3,  // D128 / A20
  PF_4,  // D129 / A21
  PF_5,  // D130 / A22
  PF_6,  // D131 / A23
  PF_7,  // D132 / A24
  PF_8,  // D133 / A25
  PF_9,  // D134 / A26
  PF_10, // D135 / A27
  PF_11, // D136 / A28
  PF_12, // D137 / A29
  PF_13, // D138 / A30
  PF_14, // D139 / A31
};
#endif
#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t _szero_nocache;
extern uint32_t _ezero_nocache;

void SystemClockStartupInit() {
  // Confirm is called only once time to avoid hang up caused by repeated calls in USB wakeup interrupt
  static bool first_call = true;
  if (!first_call) return;
  first_call = false;

  // Clear all clock setting register
  RCC->CR = 0x00000001;
  RCC->CFGR = 0x00000000;
  RCC->D1CFGR = 0x00000000;
  RCC->D2CFGR = 0x00000000;
  RCC->D3CFGR = 0x00000000;
  RCC->PLLCKSELR = 0x00000000;
  RCC->PLLCFGR = 0x00000000;
  RCC->CIER = 0x00000000;

  // AXI_TARG7_FN_MOD for SRAM
  *((volatile uint32_t*)0x51008108)=0x00000001;

  // Enable L1-Cache
  SCB_EnableICache();
  SCB->CACR |= 1<<2;
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
   *  Run mode (VOS0 to VOS3)
   *  Scale 0: boosted performance (available only with LDO regulator)
   *  Scale 1: high performance
   *  Scale 2: medium performance and consumption
   *  Scale 3: optimized performance and low-power consumption
   *
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  /* PLL1 pclk is sysclk 480 Mhz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // 480 Mhz
  RCC_OscInitStruct.PLL.PLLM = 5; // M div 5
  RCC_OscInitStruct.PLL.PLLN = 96; // N mul 96
  RCC_OscInitStruct.PLL.PLLP = 1; // P div 1
  RCC_OscInitStruct.PLL.PLLQ = 10; // Q div 10 - CANFD 48 Mhz
  RCC_OscInitStruct.PLL.PLLR = 10; // R unused
  /*
   * RCC_PLL1VCIRANGE_0  Clock range frequency between 1 and 2 MHz
   * RCC_PLL1VCIRANGE_1  Clock range frequency between 2 and 4 MHz
   * RCC_PLL1VCIRANGE_2  Clock range frequency between 4 and 8 MHz
   * RCC_PLL1VCIRANGE_3  Clock range frequency between 8 and 16 MHz */
  /* PLLRGE: RCC_PLL1VCIRANGE_2 Clock range frequency between 4 and 8 MHz  */
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  /* VCOSEL PLL1VCOWIDE 2-16 Mhz */
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection =   RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_ADC
                                             | RCC_PERIPHCLK_USART16
                                             | RCC_PERIPHCLK_USART234578
                                             | RCC_PERIPHCLK_SPI123
                                             | RCC_PERIPHCLK_SPI45 | RCC_PERIPHCLK_SPI6
                                             | RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 5;  // M DIV 5 vco 25 / 5 = 5MHz
  PeriphClkInitStruct.PLL2.PLL2N = 80; // N MUL 80 = 400
  PeriphClkInitStruct.PLL2.PLL2P = 4;  // P div 4 = 100
  PeriphClkInitStruct.PLL2.PLL2Q = 4;  // Q div 4 = 100
  PeriphClkInitStruct.PLL2.PLL2R = 2;  // R div 2 = 200
  // RCC_PLL1VCIRANGE_0  Clock range frequency between 4 and 8 MHz
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
#if 0
  PeriphClkInitStruct.PLL3.PLL3M = 15; // M DIV 15 vco 25 / 15 ~ 1.667 Mhz
  PeriphClkInitStruct.PLL3.PLL3N = 96; // N MUL 96
  PeriphClkInitStruct.PLL3.PLL3P = 2;  // P div 2
  PeriphClkInitStruct.PLL3.PLL3Q = 2;  // Q div 2
  PeriphClkInitStruct.PLL3.PLL3R = 2;  // R div 2
  // RCC_PLL1VCIRANGE_0  Clock range frequency between 1 and 2 MHz
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
#endif
  // ADC from PLL2 pclk
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  // USB from PLL1 qclk
  //PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  // QSPI from PLL1 qclk
  //PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL;
  // SDMMC from PLL2 rclk
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  // LPUART from PLL2 qclk
  //PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PLL2;
  // USART from Bus
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  // USART from Bus
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  // I2C123 from PLL3 rclk
  //PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_PLL3;
  // I2C4 from PLL3 rclk
  //PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PLL3;
  // SPI123 from PLL2 pclk
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  // SPI45 from PLL2 qclk
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  // SPI6 from PLL2 qclk
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
  // FDCAN from Q clock
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // USB clock, (use HSI48 clock)
  RCC->CR |= 1 << 12;   // HSI48 clock enabl
  while((RCC->CR & (1 << 13)) == 0);// 1: HSI48 clock is ready
  RCC->APB1HENR |= 1 << 1;      // CRS peripheral clock enabled
  RCC->APB1HRSTR |= 1 << 1;     // Resets CRS
  RCC->APB1HRSTR &= ~(1 << 1);  // Does not reset CRS
  CRS->CFGR &= ~(3 << 28);      // USB2 SOF selected as SYNC signal source
  CRS->CR |= 3 << 5;            // Automatic trimming and Frequency error counter enabled
  RCC->D2CCIP2R &= ~(3 << 20);  // Clear USBOTG 1 and 2 kernel clock source selection
  RCC->D2CCIP2R |= 3 << 20;     // HSI48_ck clock is selected as kernel clock
}

uint8_t MPU_Convert_Bytes_To_POT(uint32_t nbytes)
{
  uint8_t count = 0;
  while(nbytes != 1)
  {
    nbytes >>= 1;
    count++;
  }
  return count;
}

uint8_t MPU_Set_Protection(uint32_t baseaddr, uint32_t size, uint32_t rnum, uint8_t ap, uint8_t sen, uint8_t cen, uint8_t ben)
{
  uint32_t tempreg = 0;
  uint8_t rnr = 0;
  if ((size % 32) || size == 0) return 1;
  rnr = MPU_Convert_Bytes_To_POT(size) - 1;
  SCB->SHCSR &= ~(1 << 16);	        //disable MemManage
  MPU->CTRL &= ~(1 << 0);		        //disable MPU
  MPU->RNR = rnum;
  MPU->RBAR = baseaddr;
  tempreg |= 0 << 28;
  tempreg |= ((uint32_t)ap) << 24;
  tempreg |= 0 << 19;
  tempreg |= ((uint32_t)sen) << 18;
  tempreg |= ((uint32_t)cen) << 17;
  tempreg |= ((uint32_t)ben) << 16;
  tempreg |= 0 << 8;
  tempreg |= rnr << 1;
  tempreg |= 1 << 0;
  MPU->RASR = tempreg;
  MPU->CTRL = (1 << 2) | (1 << 0);  //enable PRIVDEFENA
  SCB->SHCSR |= 1 << 16;		        //enable MemManage
  return 0;
}

void MPU_Memory_Protection(void)
{
  // DTCM currently no used
	MPU_Set_Protection(0x20000000, 128 * 1024, 1, MPU_REGION_FULL_ACCESS, 0, 1, 1);       // protect DTCM 128k,  Sharing is prohibited, cache is allowed, and buffering is allowed
  // AXI RAM used for stacks and heap
	MPU_Set_Protection(0x24000000, (320 - 32) * 1024, 2, MPU_REGION_FULL_ACCESS, 0, 1, 1);       // protect AXI SRAM,  Sharing is prohibited, cache is allowed, and buffering is allowed
	MPU_Set_Protection(0x24048000, 32 * 1024, 3, MPU_REGION_FULL_ACCESS, 1, 0, 0);       // protect AXI SRAM,  Sharing is enabled, no cache, no buffering
  // SRAM1~SRAM3 used for DMA buffers
	MPU_Set_Protection(0x30000000, 32 * 1024, 4, MPU_REGION_FULL_ACCESS, 1, 0, 0);       // protect SRAM1~SRAM3, Sharing is enabled, no cache, no buffering
  // SRAM4 currently not used
	MPU_Set_Protection(0x38000000, 16 * 1024, 5, MPU_REGION_FULL_ACCESS, 0, 1, 1);        // protect SRAM4, Sharing is prohibited, cache is allowed, and buffering is allowed

  // We don't use any of these
	//MPU_Set_Protection(0x60000000, 64 * 1024 * 1024, 5, MPU_REGION_FULL_ACCESS, 0, 0, 0);   // protect LCD FMC  64M, No sharing, no cache, no buffering
	//MPU_Set_Protection(0XC0000000, 32 * 1024 * 1024, 6, MPU_REGION_FULL_ACCESS, 0, 1, 1);   // protect SDRAM  32M, Sharing is prohibited, cache is allowed, and buffering is allowed
	//MPU_Set_Protection(0X80000000, 256 * 1024 * 1024, 7, MPU_REGION_FULL_ACCESS, 0, 0, 0);  // protect NAND FLASH 256M, No sharing, no cache, no buffering
}

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  SystemClockStartupInit();

  MPU_Memory_Protection();

  /* Update current SystemCoreClock value */
  SystemCoreClockUpdate();

  /* Configure the Systick interrupt time */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

  /* Configure the Systick */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Zero uncahced memory */
	for (uint32_t *pDest = &_szero_nocache; pDest < &_ezero_nocache;)
	{
		*pDest++ = 0;
	}

}

#ifdef __cplusplus
}
#endif
