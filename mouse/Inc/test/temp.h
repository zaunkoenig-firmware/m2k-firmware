#pragma once

#include "stm32f7xx.h"

static void init_temp(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable ADC1
	ADC1->SQR3 = 18; // channel 18 for temp sensor
	ADC1->SMPR1 = 0b111 << ADC_SMPR1_SMP18_Pos; // 480 cycles for sampling time > 10us
	// disable VBAT and enable temp sensor
	// PCLK2 = HCLK = 32 MHz. need ADC clock < 36MHz.
	// so set prescaler to 00 to give 32/2 = 16 MHz.
	MODIFY_REG(ADC->CCR,
			ADC_CCR_VBATE | ADC_CCR_ADCPRE,
			ADC_CCR_TSVREFE);
	ADC1->CR2 |= ADC_CR2_ADON; // turn on
	HAL_Delay(1); // only actually needs 3us to stabilize

	NVIC_EnableIRQ(ADC_IRQn);
}

// returns mult*(temp in Celsius)
static int16_t get_temp(void)
{
	const int mult = 10;

	// read temperature sensor calibration
	uint16_t t30 = *TEMPSENSOR_CAL1_ADDR_CMSIS; // 30 C
	uint16_t t110 = *TEMPSENSOR_CAL2_ADDR_CMSIS; // 110 C

	ADC1->CR1 = ADC_CR1_EOCIE;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while (ADC1->CR1 != 0)
		__WFI();
	uint32_t adc_dr = ADC1->DR;
	uint16_t temp = (uint16_t)(adc_dr & 0x00000fff); // lowest 12 bits
	return mult*30 + mult*((int16_t)(temp - t30))*(110-30)/(t110-t30);
}

void ADC_IRQHandler(void)
{
	ADC1->CR1 = 0;
	ADC1->SR = 0;
}
