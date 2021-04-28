/**
  ******************************************************************************
  * @file    USB_Device/DFU_Standalone/Src/main.c
  * @author  MCD Application Team
  * @brief   USB device DFU demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *#pragma once

#define M2K_USB_MFG "Zaunkoenig"
#define M2K_USB_NAME "M2K (F730 Proto Rev0)"
#define M2K_USB_PID 0xA3CF
#define M2K_USB_VID 0x0483
#ifdef 0
// TODO this is proto
// SPI periph pin clocks
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                do {RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;} while(0)
#define SPIx_SCK_GPIO_CLK_ENABLE()       do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_MISO_GPIO_CLK_ENABLE()      do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_MOSI_GPIO_CLK_ENABLE()      do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_SS_GPIO_CLK_ENABLE()	     do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;} while(0)

// SPI
#define SPIx_SCK_GPIO_PORT  GPIOA
#define SPIx_SCK_PIN_Pos    5
#define SPIx_SCK_PIN        (1 << SPIx_SCK_PIN_Pos)
#define SPIx_SCK_AF         5 //GPIO_AF5_SPI1

#define SPIx_MISO_GPIO_PORT GPIOA
#define SPIx_MISO_PIN_Pos   6
#define SPIx_MISO_PIN       (1 << SPIx_MISO_PIN_Pos)
#define SPIx_MISO_AF        5 //GPIO_AF5_SPI1

#define SPIx_MOSI_GPIO_PORT GPIOA
#define SPIx_MOSI_PIN_Pos   7
#define SPIx_MOSI_PIN       (1 << SPIx_MOSI_PIN_Pos)
#define SPIx_MOSI_AF        5 //GPIO_AF5_SPI1

#define SPIx_SS_PORT        GPIOB
#define SPIx_SS_PIN_Pos     6
#define SPIx_SS_PIN         (1 << SPIx_SS_PIN_Pos)

// lmb
#define LMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define LMB_NO_PORT         GPIOD
#define LMB_NO_PIN_Pos      10
#define LMB_NO_PIN          (1 << LMB_NO_PIN_Pos)

#define LMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define LMB_NC_PORT         GPIOD
#define LMB_NC_PIN_Pos      8
#define LMB_NC_PIN          (1 << LMB_NC_PIN_Pos)

// rmb
#define RMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define RMB_NO_PORT         GPIOA
#define RMB_NO_PIN_Pos      3
#define RMB_NO_PIN          (1 << RMB_NO_PIN_Pos)

#define RMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;} while(0)
#define RMB_NC_PORT         GPIOH
#define RMB_NC_PIN_Pos      13
#define RMB_NC_PIN          (1 << RMB_NC_PIN_Pos)

// mmb
#define MMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;} while(0)
#define MMB_NO_PORT         GPIOB
#define MMB_NO_PIN_Pos      2
#define MMB_NO_PIN          (1 << MMB_NO_PIN_Pos)

#define MMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;} while(0)
#define MMB_NC_PORT         GPIOG
#define MMB_NC_PIN_Pos      1
#define MMB_NC_PIN          (1 << MMB_NC_PIN_Pos)

// wheel
#define WHL_P_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;} while(0)
#define WHL_P_PORT         GPIOF
#define WHL_P_PIN_Pos      10
#define WHL_P_PIN          (1 << WHL_P_PIN_Pos)

#define WHL_N_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;} while(0)
#define WHL_N_PORT         GPIOH
#define WHL_N_PIN_Pos      12
#define WHL_N_PIN          (1 << WHL_N_PIN_Pos)
#else
// TODO this is prod
#pragma once

#define M2K_USB_MFG "Zaunkoenig"
#define M2K_USB_NAME "M2K"
#define M2K_USB_PID 0xA3CF
#define M2K_USB_VID 0x0483

// SPI periph pin clocks
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                do {RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;} while(0)
#define SPIx_SCK_GPIO_CLK_ENABLE()       do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_MISO_GPIO_CLK_ENABLE()      do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_MOSI_GPIO_CLK_ENABLE()      do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_SS_GPIO_CLK_ENABLE()	     do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;} while(0)

// SPI
#define SPIx_SCK_GPIO_PORT  GPIOA
#define SPIx_SCK_PIN_Pos    5
#define SPIx_SCK_PIN        (1 << SPIx_SCK_PIN_Pos)
#define SPIx_SCK_AF         5 //GPIO_AF5_SPI1

#define SPIx_MISO_GPIO_PORT GPIOA
#define SPIx_MISO_PIN_Pos   6
#define SPIx_MISO_PIN       (1 << SPIx_MISO_PIN_Pos)
#define SPIx_MISO_AF        5 //GPIO_AF5_SPI1

#define SPIx_MOSI_GPIO_PORT GPIOA
#define SPIx_MOSI_PIN_Pos   7
#define SPIx_MOSI_PIN       (1 << SPIx_MOSI_PIN_Pos)
#define SPIx_MOSI_AF        5 //GPIO_AF5_SPI1

#define SPIx_SS_PORT        GPIOB
#define SPIx_SS_PIN_Pos     6
#define SPIx_SS_PIN         (1 << SPIx_SS_PIN_Pos)

// lmb
#define LMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define LMB_NO_PORT         GPIOD
#define LMB_NO_PIN_Pos      10
#define LMB_NO_PIN          (1 << LMB_NO_PIN_Pos)

#define LMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define LMB_NC_PORT         GPIOD
#define LMB_NC_PIN_Pos      8
#define LMB_NC_PIN          (1 << LMB_NC_PIN_Pos)

// rmb
#define RMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;} while(0)
#define RMB_NO_PORT         GPIOE
#define RMB_NO_PIN_Pos      4
#define RMB_NO_PIN          (1 << RMB_NO_PIN_Pos)

#define RMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;} while(0)
#define RMB_NC_PORT         GPIOE
#define RMB_NC_PIN_Pos      15
#define RMB_NC_PIN          (1 << RMB_NC_PIN_Pos)

// mmb
#define MMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;} while(0)
#define MMB_NO_PORT         GPIOF
#define MMB_NO_PIN_Pos      11
#define MMB_NO_PIN          (1 << MMB_NO_PIN_Pos)

#define MMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;} while(0)
#define MMB_NC_PORT         GPIOF
#define MMB_NC_PIN_Pos      14
#define MMB_NC_PIN          (1 << MMB_NC_PIN_Pos)

// wheel
#define WHL_P_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;} while(0)
#define WHL_P_PORT         GPIOE
#define WHL_P_PIN_Pos      1
#define WHL_P_PIN          (1 << WHL_P_PIN_Pos)

#define WHL_N_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define WHL_N_PORT         GPIOD
#define WHL_N_PIN_Pos      12
#define WHL_N_PIN          (1 << WHL_N_PIN_Pos)
#endif
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "m2k_resource.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DFU_TIMEOUT 3 // seconds of holding L+R for DFU mode
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
pFunction JumpToApplication;
uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
//static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	// check if LMB and RMB are both pressed
	LMB_NO_CLK_ENABLE();
	MODIFY_REG(LMB_NO_PORT->PUPDR,
			0b11 << (2*LMB_NO_PIN_Pos),
			0b01 << (2*LMB_NO_PIN_Pos));
	RMB_NO_CLK_ENABLE();
	MODIFY_REG(RMB_NO_PORT->PUPDR,
			0b11 << (2*RMB_NO_PIN_Pos),
			0b01 << (2*RMB_NO_PIN_Pos));

	// LMB or RMB not pressed
	if ((LMB_NO_PORT->IDR & LMB_NO_PIN) != 0 ||
			(RMB_NO_PORT->IDR & RMB_NO_PIN) != 0) {
		JumpAddress = *(__IO uint32_t*) (USBD_DFU_APP_DEFAULT_ADD + 4);
		JumpToApplication = (pFunction) JumpAddress;
		__set_MSP(*(__IO uint32_t*) USBD_DFU_APP_DEFAULT_ADD);
		JumpToApplication();
	}

	// LMB and RMB pressed. continue to DFU mode if held for DFU_TIMEOUT seconds.
	// default HSI clock is 16MHz. set SysTick to reload every 1ms.
	SysTick->LOAD = (16000000/1000) - 1;
	SysTick->VAL  = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	for (int i = 0; i < DFU_TIMEOUT*1000; i++) { // loops every 1ms
		while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
			if ((LMB_NO_PORT->IDR & LMB_NO_PIN) != 0 ||
					(RMB_NO_PORT->IDR & RMB_NO_PIN) != 0) { // LMB or RMB released
				// keep SysTick enabled. application will know from this that
				// both LMB and RMB were originally pressed.
				JumpAddress = *(__IO uint32_t*) (USBD_DFU_APP_DEFAULT_ADD + 4);
				JumpToApplication = (pFunction) JumpAddress;
				__set_MSP(*(__IO uint32_t*) USBD_DFU_APP_DEFAULT_ADD);
				JumpToApplication();
			}
		}
	}

  /* Otherwise enters DFU mode to allow user programming his application */

  /* Enable the CPU Cache */
//  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();
    
//  /* Configure Key Button */
//  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
//
//  /* Check if the KEY Button is pressed */
//  if(BSP_PB_GetState(BUTTON_USER) == 0x00)
//  {
//    /* Test if user code is programmed starting from USBD_DFU_APP_DEFAULT_ADD address */
//    if(((*(__IO uint32_t*)USBD_DFU_APP_DEFAULT_ADD) & 0x2FFE0000 ) == 0x20000000)
//    {
//      /* Jump to user application */
//      JumpAddress = *(__IO uint32_t*) (USBD_DFU_APP_DEFAULT_ADD + 4);
//      JumpToApplication = (pFunction) JumpAddress;
//
//      /* Initialize user application's Stack Pointer */
//      __set_MSP(*(__IO uint32_t*) USBD_DFU_APP_DEFAULT_ADD);
//      JumpToApplication();
//    }
//  }

  /* Init Device Library */
  USBD_Init(&USBD_Device, &DFU_Desc, 0);
  
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_DFU_CLASS);
  
  /* Add DFU Media interface */
  USBD_DFU_RegisterMedia(&USBD_Device, &USBD_DFU_Flash_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
  
  /* Run Application (Interrupt mode) */
  while (1)
  {
    __WFI();
  }
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on SysTick counter flag.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */

void HAL_Delay(__IO uint32_t Delay)
{
  while(Delay) 
  {
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) 
    {
      Delay--;
    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 24000000
  *            PLL_M                          = 12
  *            PLL_N                          = 96
  *            PLL_P                          = 6
  *            PLL_Q                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
//  /* Activate the OverDrive to reach the 216 Mhz Frequency */
//  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
//  {
//    Error_Handler();
//  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
//static void CPU_CACHE_Enable(void)
//{
//  /* Enable I-Cache */
//  SCB_EnableICache();
//
//  /* Enable D-Cache */
//  SCB_EnableDCache();
//}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
