/* MIT License
 *
 * Copyright (c) 2022 Zaunkoenig GmbH
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
