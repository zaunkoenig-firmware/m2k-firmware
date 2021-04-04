#include "stm32f7xx.h"
#include "delay.h"

void delay_init(void)
{
	// see pg 132 of ref manual: TIM2CLK = PCLK1 = HCLK
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
#ifdef DELAY_SLEEP
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
#else
	TIM2->CR1 = TIM_CR1_CEN;
#endif
}

#ifdef DELAY_SLEEP
void TIM2_IRQHandler(void)
{
	TIM2->CR1 = 0; // disable counter
	TIM2->SR = 0; // clear status flag
}
#endif
