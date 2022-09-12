#undef M2K_FW_VERSION
#define M2K_FW_VERSION " fw v1.2 (LH)"

#undef LMB_NO_CLK_ENABLE
#undef LMB_NO_PORT
#undef LMB_NO_PIN_Pos
#undef LMB_NO_PIN
#undef LMB_NO_EXTICFG
#undef LMB_NC_CLK_ENABLE
#undef LMB_NC_PORT
#undef LMB_NC_PIN_Pos
#undef LMB_NC_PIN
#undef LMB_NC_EXTICFG

#undef RMB_NO_CLK_ENABLE
#undef RMB_NO_PORT
#undef RMB_NO_PIN_Pos
#undef RMB_NO_PIN
#undef RMB_NO_EXTICFG
#undef RMB_NC_CLK_ENABLE
#undef RMB_NC_PORT
#undef RMB_NC_PIN_Pos
#undef RMB_NC_PIN
#undef RMB_NC_EXTICFG

// lmb, but with pins for rmb
#define LMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;} while(0)
#define LMB_NO_PORT         GPIOE
#define LMB_NO_PIN_Pos      4
#define LMB_NO_PIN          (1 << RMB_NO_PIN_Pos)
#define LMB_NO_EXTICFG      SYSCFG_EXTICR2_EXTI4_PE

#define LMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;} while(0)
#define LMB_NC_PORT         GPIOE
#define LMB_NC_PIN_Pos      15
#define LMB_NC_PIN          (1 << RMB_NC_PIN_Pos)
#define LMB_NC_EXTICFG      SYSCFG_EXTICR4_EXTI15_PE

// rmb, but with pins for lmb
#define RMB_NO_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define RMB_NO_PORT         GPIOD
#define RMB_NO_PIN_Pos      10
#define RMB_NO_PIN          (1 << LMB_NO_PIN_Pos)
#define RMB_NO_EXTICFG      SYSCFG_EXTICR3_EXTI10_PD

#define RMB_NC_CLK_ENABLE() do {RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;} while(0)
#define RMB_NC_PORT         GPIOD
#define RMB_NC_PIN_Pos      8
#define RMB_NC_PIN          (1 << LMB_NC_PIN_Pos)
#define RMB_NC_EXTICFG      SYSCFG_EXTICR3_EXTI8_PD
