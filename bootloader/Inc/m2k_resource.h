#pragma once

// copy this file to any MCU init project's includes and change the mappings for the particular platform
// 'safety error' to stop someone from importing this file without doing the init first
//#error You need to customize m2k_resource_template.h!

#define M2K_USB_MFG "Zaunkoenig"
#define M2K_USB_NAME "M2K (F730 Proto Rev0)"
#define M2K_USB_PID 0xA3CF
#define M2K_USB_VID 0x0483

//#define ALWAYS_SEND_FRAME 1
//#define SEND_TEMP

// debug GPIO(s)

//#define USE_DEBUG_0
#ifdef USE_DEBUG_0
#define DBG_GPIO_0_CLK_ENABLE()	do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define DBG_GPIO_0_PORT 		GPIOA
#define DBG_GPIO_0_PIN	 		(1 << 10)
#define DBG_GPIO_0_HIGH			DBG_GPIO_0_PORT->ODR |= DBG_GPIO_0_PIN;
#define DBG_GPIO_0_LOW			DBG_GPIO_0_PORT->ODR &= ~DBG_GPIO_0_PIN;
#endif

//#define USE_DEBUG_1
#ifdef USE_DEBUG_1
#define DBG_GPIO_1_CLK_ENABLE()	do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;} while(0)
#define DBG_GPIO_1_PORT 		GPIOH
#define DBG_GPIO_1_PIN	 		GPIO_PIN_2
#define DBG_GPIO_1_HIGH			DBG_GPIO_1_PORT->ODR |= DBG_GPIO_1_PIN;
#define DBG_GPIO_1_LOW			DBG_GPIO_1_PORT->ODR &= ~DBG_GPIO_1_PIN;
#endif

// SPI periph pin clocks
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                do {RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;} while(0)
#define SPIx_SCK_GPIO_CLK_ENABLE()       do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_MISO_GPIO_CLK_ENABLE()      do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define SPIx_MOSI_GPIO_CLK_ENABLE()      do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;} while(0)
#define SPIx_CS_CLK_ENABLE() 		     do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;} while(0)

#define SPIx_FORCE_RESET()               do {RCC->APB2RSTR |= (RCC_APB2RSTR_SPI1RST);} while(0)
#define SPIx_RELEASE_RESET()             do {RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);} while(0)

// SPI
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_PIN                     (1 << 5)
#define SPIx_SCK_AF                      5 //GPIO_AF5_SPI1
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_PIN                    (1 << 6)
#define SPIx_MISO_AF                     5 //GPIO_AF5_SPI1
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_PIN                    (1 << 7)
#define SPIx_MOSI_AF                     5 //GPIO_AF5_SPI1
#define SPIx_SS_PORT					 GPIOB
#define SPIx_SS_PIN						 (1 << 6)

// lmb
#define LMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define LMB_NO_PORT         GPIOD
#define LMB_NO_PIN          (1 << 10)
#define LMB_NO_EXTI_GPIO EXTI_GPIOD
#define LMB_NO_EXTI_LINE EXTI_LINE_10

#define LMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define LMB_NC_PORT         GPIOD
#define LMB_NC_PIN          (1 << 8)
#define LMB_NC_EXTI_GPIO EXTI_GPIOD
#define LMB_NC_EXTI_LINE EXTI_LINE_8

// rmb
#define RMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;} while(0)
#define RMB_NO_PORT         GPIOA
#define RMB_NO_PIN          (1 << 3)
#define RMB_NO_EXTI_GPIO EXTI_GPIOA
#define RMB_NO_EXTI_LINE EXTI_LINE_3

#define RMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;} while(0)
#define RMB_NC_PORT         GPIOH
#define RMB_NC_PIN          (1 << 13)
#define RMB_NC_EXTI_GPIO EXTI_GPIOH
#define RMB_NC_EXTI_LINE EXTI_LINE_13

// mmb
#define MMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;} while(0)
#define MMB_NO_PORT         GPIOB
#define MMB_NO_PIN          (1 << 2)
#define MMB_NO_EXTI_GPIO EXTI_GPIOB
#define MMB_NO_EXTI_LINE EXTI_LINE_2

#define MMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;} while(0)
#define MMB_NC_PORT         GPIOG
#define MMB_NC_PIN          (1 << 1)
#define MMB_NC_EXTI_GPIO EXTI_GPIOG
#define MMB_NC_EXTI_LINE EXTI_LINE_1

// wheel
#define WHL_P_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;} while(0)
#define WHL_P_PORT         GPIOF
#define WHL_P_PIN          (1 << 10)

#define WHL_N_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;} while(0)
#define WHL_N_PORT         GPIOH
#define WHL_N_PIN          (1 << 12)
