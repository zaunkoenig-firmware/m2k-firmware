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
static void btn_init(void)
{
	// enable GPIO ports and pull-up resistors
	// assume they are all inputs (default after reset)
	LMB_NO_CLK_ENABLE();
	LMB_NC_CLK_ENABLE();
	RMB_NO_CLK_ENABLE();
	RMB_NC_CLK_ENABLE();
	MMB_NO_CLK_ENABLE();
	MMB_NC_CLK_ENABLE();
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

	// rising edge detection
	const uint32_t pins_mask = (
			LMB_NO_PIN_Pos | LMB_NC_PIN_Pos |
			RMB_NO_PIN_Pos | RMB_NC_PIN_Pos |
			MMB_NO_PIN_Pos | MMB_NC_PIN_Pos
	);
	EXTI->RTSR |= pins_mask;
	EXTI->IMR |= pins_mask;
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
			LMB_NO_PIN_Pos | LMB_NC_PIN_Pos |
			RMB_NO_PIN_Pos | RMB_NC_PIN_Pos |
			MMB_NO_PIN_Pos | MMB_NC_PIN_Pos
	);
	EXTI->PR = pins_mask;
	return now & ~edge;
}

// enable pull up registers for wheel (mechanical quadrature encoder)
static void whl_init(void)
{
	WHL_P_CLK_ENABLE();
	WHL_N_CLK_ENABLE();
	MODIFY_REG(WHL_P_PORT->PUPDR,
			0b11 << (2*WHL_P_PIN_Pos),
			0b01 << (2*WHL_P_PIN_Pos));
	MODIFY_REG(WHL_N_PORT->PUPDR,
			0b11 << (2*WHL_N_PIN_Pos),
			0b01 << (2*WHL_N_PIN_Pos));
}

static inline int whl_read_p(void)
{
	return SHIFT(WHL_P_PORT->IDR & WHL_P_PIN, WHL_P_PIN_Pos, 0);
}

static inline int whl_read_n(void)
{
	return SHIFT(WHL_N_PORT->IDR & WHL_N_PIN, WHL_N_PIN_Pos, 0);
}
