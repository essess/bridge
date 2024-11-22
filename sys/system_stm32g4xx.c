/**
  ******************************************************************************
  * @file    system_stm32g4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32g4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *   After each device reset the HSI (16 MHz) is used as system clock source.
  *   Then SystemInit() function is called, in "startup_stm32g4xx.s" file, to
  *   configure the system clock before to branch to main program.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32g4xx_system
  * @{
  */

/** @addtogroup STM32G4xx_System_Private_Includes
  * @{
  */


#include <assert.h>
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_bus.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE     8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
/******************************************************************************/
/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Private_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
  uint32_t SystemCoreClock = HSI_VALUE;

  const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
  const uint8_t APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */

void SystemInit(void)
{
  /* FPU settings ---------------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
  #endif

  /* we should have arrived here via some hardware reset mechanism ------------------- */
  /*                                                                                   */
  if(!(RCC->CSR & (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF |
                   RCC_CSR_SFTRSTF  | RCC_CSR_BORRSTF  | RCC_CSR_PINRSTF  |
                   RCC_CSR_OBLRSTF)))
    NVIC_SystemReset();

  /** get SYSCLK up to 170MHz first thing -------------------------------------------- */
  /*                                                                                   */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
  LL_RCC_LSE_Enable();
  LL_RCC_HSE_Enable();
  LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, /*< PLLSRC := HSE                  */
                              LL_RCC_PLLM_DIV_2,    /*< PLLM   := /2  ->   4MHz        */
                              85,                   /*< PLLN   := *85 -> 340MHz VCO    */
                              LL_RCC_PLLR_DIV_2);   /*< PLLR   := /2  -> 170MHZ SYSCLK */
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, /*< PLLSRC := HSE                  */
                              LL_RCC_PLLM_DIV_2,    /*< PLLM   := /2  ->   4MHz        */
                              85,                   /*< PLLN   := *85 -> 340MHz VCO    */
                              LL_RCC_PLLP_DIV_2);   /*< PLLP   := /2  -> 170MHZ        */
  LL_RCC_PLL_EnableDomain_ADC();
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, /*< PLLSRC := HSE                  */
                              LL_RCC_PLLM_DIV_2,    /*< PLLM   := /2  ->   4MHz        */
                              85,                   /*< PLLN   := *85 -> 340MHz VCO    */
                              LL_RCC_PLLQ_DIV_2);   /*< PLLQ   := /2  -> 170MHZ        */
  LL_RCC_PLL_EnableDomain_48M();
  LL_RCC_PLL_Enable();

  unsigned loops = 500*3;                                  /**< ~3x margin (empirical) */
  do {
    if(LL_RCC_PLL_IsReady()) {                     /**< when pll locked - switch it in */
      LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
      LL_FLASH_EnablePrefetch();
      LL_FLASH_EnableInstCache();
      LL_FLASH_EnableDataCache();

      LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);     /**< AHB  := /1 -> 170MHz HCLK  */
      LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);      /**< APB1 := /1 -> 170MHz PCLK1 */
      LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);      /**< APB2 := /1 -> 170MHz PCLK2 */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
      break;
    }
  } while(--loops);
  assert(loops);

  NVIC_SetPriorityGrouping(0);                            /**< 16 levels @see:PRIGROUP */
  /*                  .. leave the rest up to the application                          */
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(***)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (**) HSI_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              24 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  LL_RCC_ClocksTypeDef clks;
  LL_RCC_GetSystemClocksFreq(&clks);
  SystemCoreClock = clks.HCLK_Frequency;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

