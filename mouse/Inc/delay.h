#pragma once

#include "stm32f7xx.h"

void delay_init(void);

#define DELAY_SLEEP
#ifdef DELAY_SLEEP
static inline void delay_us(const uint32_t us)
{
	TIM2->CNT = 32*us - 1; // assumes TIM2CLK = 32MHz
	TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
	while (TIM2->CR1 != 0) // can comment if only interrupt is TIM2_IRQHandler
		__WFI();
}
#else // delay with busy wait
static inline void delay_us(const uint32_t us)
{
	TIM2->CNT = 0;
	while (TIM2->CNT < 32*us); // assumes TIM2CLK = 32MHz
}
#endif

#define delay_ms(x) delay_us(1000*(x))
