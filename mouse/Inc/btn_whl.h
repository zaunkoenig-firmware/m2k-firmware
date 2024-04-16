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
#include "m2k_resource.h"

// if x is either 0 or (1 << a)
// this gives either 0 or (1 << b)
// trust compiler to optimize away conditionals if a and b are constant
#define SHIFT(x, a, b) ( \
		(a) > (b) ? (x) >> ((a) - (b)) : \
		(a) < (b) ? (x) << ((b) - (a)) : \
		(x))

// enable pull up registers for buttons, and edge change detection
static void btn_whl_init(void)
{
	// enable GPIO ports and pull-up resistors
	// assume they are all inputs (default after reset)
	LMB_NO_CLK_ENABLE();
	LMB_NC_CLK_ENABLE();
	RMB_NO_CLK_ENABLE();
	RMB_NC_CLK_ENABLE();
	MMB_NO_CLK_ENABLE();
	MMB_NC_CLK_ENABLE();
	WHL_P_CLK_ENABLE();
	WHL_N_CLK_ENABLE();
	MODIFY_REG(LMB_NO_PORT->PUPDR,
			0b11 << (2*LMB_NO_PIN_Pos),
			0b01 << (2*LMB_NO_PIN_Pos));
	MODIFY_REG(LMB_NC_PORT->PUPDR,
			0b11 << (2*LMB_NC_PIN_Pos),
			0b01 << (2*LMB_NC_PIN_Pos));
	MODIFY_REG(RMB_NO_PORT->PUPDR,
			0b11 << (2*RMB_NO_PIN_Pos),
			0b01 << (2*RMB_NO_PIN_Pos));
	MODIFY_REG(RMB_NC_PORT->PUPDR,
			0b11 << (2*RMB_NC_PIN_Pos),
			0b01 << (2*RMB_NC_PIN_Pos));
	MODIFY_REG(MMB_NO_PORT->PUPDR,
			0b11 << (2*MMB_NO_PIN_Pos),
			0b01 << (2*MMB_NO_PIN_Pos));
	MODIFY_REG(MMB_NC_PORT->PUPDR,
			0b11 << (2*MMB_NC_PIN_Pos),
			0b01 << (2*MMB_NC_PIN_Pos));
	MODIFY_REG(WHL_P_PORT->PUPDR,
			0b11 << (2*WHL_P_PIN_Pos),
			0b01 << (2*WHL_P_PIN_Pos));
	MODIFY_REG(WHL_N_PORT->PUPDR,
			0b11 << (2*WHL_N_PIN_Pos),
			0b01 << (2*WHL_N_PIN_Pos));

	// rising edge detection
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	SYSCFG->EXTICR[LMB_NO_PIN_Pos/4] |= LMB_NO_EXTICFG;
	SYSCFG->EXTICR[LMB_NC_PIN_Pos/4] |= LMB_NC_EXTICFG;
	SYSCFG->EXTICR[RMB_NO_PIN_Pos/4] |= RMB_NO_EXTICFG;
	SYSCFG->EXTICR[RMB_NC_PIN_Pos/4] |= RMB_NC_EXTICFG;
	SYSCFG->EXTICR[MMB_NO_PIN_Pos/4] |= MMB_NO_EXTICFG;
	SYSCFG->EXTICR[MMB_NC_PIN_Pos/4] |= MMB_NC_EXTICFG;
	SYSCFG->EXTICR[WHL_P_PIN_Pos/4] |= WHL_P_EXTICFG;
	SYSCFG->EXTICR[WHL_N_PIN_Pos/4] |= WHL_N_EXTICFG;

	const uint32_t pins_mask = (
			LMB_NO_PIN | LMB_NC_PIN |
			RMB_NO_PIN | RMB_NC_PIN |
			MMB_NO_PIN | MMB_NC_PIN |
			WHL_P_PIN | WHL_N_PIN
	);
	EXTI->RTSR = pins_mask;
	EXTI->IMR = pins_mask;
	EXTI->PR = pins_mask;
}

// returns (NC << 8) | NO
// where NO and NC have LMB in bit 0, RMB in bit 1, MMB in bit 2
// rising edge detection example:
// if LMB_NO was always 1: 1 in bit 0
// if LMB_NO was low at any point since last clear of EXTI->PR: 0 in bit 0
static inline uint16_t btn_read(void)
{
	const uint16_t now = (
			SHIFT(LMB_NO_PORT->IDR & LMB_NO_PIN, LMB_NO_PIN_Pos, 0) |
			SHIFT(RMB_NO_PORT->IDR & RMB_NO_PIN, RMB_NO_PIN_Pos, 1) |
			SHIFT(MMB_NO_PORT->IDR & MMB_NO_PIN, MMB_NO_PIN_Pos, 2) |
			SHIFT(LMB_NC_PORT->IDR & LMB_NC_PIN, LMB_NC_PIN_Pos, 0 + 8) |
			SHIFT(RMB_NC_PORT->IDR & RMB_NC_PIN, RMB_NC_PIN_Pos, 1 + 8) |
			SHIFT(MMB_NC_PORT->IDR & MMB_NC_PIN, MMB_NC_PIN_Pos, 2 + 8)
	);
	const uint32_t EXTI_PR_read = EXTI->PR;
	const uint16_t edge = (
			SHIFT(EXTI_PR_read & LMB_NO_PIN, LMB_NO_PIN_Pos, 0) |
			SHIFT(EXTI_PR_read & RMB_NO_PIN, RMB_NO_PIN_Pos, 1) |
			SHIFT(EXTI_PR_read & MMB_NO_PIN, MMB_NO_PIN_Pos, 2) |
			SHIFT(EXTI_PR_read & LMB_NC_PIN, LMB_NC_PIN_Pos, 0 + 8) |
			SHIFT(EXTI_PR_read & RMB_NC_PIN, RMB_NC_PIN_Pos, 1 + 8) |
			SHIFT(EXTI_PR_read & MMB_NC_PIN, MMB_NC_PIN_Pos, 2 + 8)
	);
	const uint32_t pins_mask = (
			LMB_NO_PIN | LMB_NC_PIN |
			RMB_NO_PIN | RMB_NC_PIN |
			MMB_NO_PIN | MMB_NC_PIN
	);
	EXTI->PR = pins_mask;
	return now & ~edge;
}

static inline int whl_read(void)
{
	const int now = (
			SHIFT(WHL_N_PORT->IDR & WHL_N_PIN, WHL_N_PIN_Pos, 0) |
			SHIFT(WHL_P_PORT->IDR & WHL_P_PIN, WHL_P_PIN_Pos, 1)
	);
	const uint32_t EXTI_PR_read = EXTI->PR;
	const int edge = (
			SHIFT(EXTI_PR_read & WHL_N_PIN, WHL_N_PIN_Pos, 0) |
			SHIFT(EXTI_PR_read & WHL_P_PIN, WHL_P_PIN_Pos, 1)
	);
	EXTI->PR = WHL_N_PIN | WHL_P_PIN;
	return now & ~edge;
}
