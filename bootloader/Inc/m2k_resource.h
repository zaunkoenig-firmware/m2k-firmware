#pragma once


#if 0
#define M2K_USB_MFG "Zaunkoenig"
#define M2K_USB_NAME "M2K (F730 Proto Rev0)"
#define M2K_USB_PID 0xA3CF
#define M2K_USB_VID 0x0483
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
#define WHL_P_EXTICFG      SYSCFG_EXTICR1_EXTI1_PE

#define WHL_N_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define WHL_N_PORT         GPIOD
#define WHL_N_PIN_Pos      12
#define WHL_N_PIN          (1 << WHL_N_PIN_Pos)
#define WHL_N_EXTICFG      SYSCFG_EXTICR4_EXTI12_PD

#endif
