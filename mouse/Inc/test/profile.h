#pragma once

#include "stm32f7xx.h"

static void profile_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
}

static inline void profile_start(void)
{
	TIM5->CNT = 0;
	TIM5->CR1 = TIM_CR1_CEN;
}

static inline void profile_stop(void)
{
	TIM5->CR1 = 0;
}

static inline uint32_t profile_read(void)
{
	return TIM5->CNT;
}
