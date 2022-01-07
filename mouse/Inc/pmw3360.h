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

#include <delay.h>
#include <stdint.h>
#include "stm32f7xx.h"
#include "config.h"
#include "m2k_resource.h"
#include "srom_3360_0x05.h"

static void spi_init(void)
{
	// GPIO config
	SPIx_SCK_GPIO_CLK_ENABLE();
	SPIx_MISO_GPIO_CLK_ENABLE();
	SPIx_MOSI_GPIO_CLK_ENABLE();
	SPIx_SS_GPIO_CLK_ENABLE();

	// PA5 SCK
	MODIFY_REG(SPIx_SCK_GPIO_PORT->MODER,
			0b11 << (2*SPIx_SCK_PIN_Pos),
			0b10 << (2*SPIx_SCK_PIN_Pos));
	MODIFY_REG(SPIx_SCK_GPIO_PORT->AFR[SPIx_SCK_PIN_Pos >= 8],
			0b1111 << ((4*SPIx_SCK_PIN_Pos)%32),
			SPIx_SCK_AF << ((4*SPIx_SCK_PIN_Pos)%32));
	// PA6 MISO
	MODIFY_REG(SPIx_MISO_GPIO_PORT->MODER,
			0b11 << (2*SPIx_MISO_PIN_Pos),
			0b10 << (2*SPIx_MISO_PIN_Pos));
	MODIFY_REG(SPIx_MISO_GPIO_PORT->PUPDR,
			0b11 << (2*SPIx_MISO_PIN_Pos),
			0b10 << (2*SPIx_MISO_PIN_Pos));
	MODIFY_REG(SPIx_MISO_GPIO_PORT->AFR[SPIx_MISO_PIN_Pos >= 8],
			0b1111 << ((4*SPIx_MISO_PIN_Pos)%32),
			SPIx_MISO_AF << ((4*SPIx_MISO_PIN_Pos)%32));
	// PA7 MOSI
	MODIFY_REG(SPIx_MOSI_GPIO_PORT->MODER,
			0b11 << (2*SPIx_MOSI_PIN_Pos),
			0b10 << (2*SPIx_MOSI_PIN_Pos));
	MODIFY_REG(SPIx_MOSI_GPIO_PORT->AFR[SPIx_MOSI_PIN_Pos >= 8],
			0b1111 << ((4*SPIx_MOSI_PIN_Pos)%32),
			SPIx_MOSI_AF << ((4*SPIx_MOSI_PIN_Pos)%32));
	// PB6 SS
	MODIFY_REG(SPIx_SS_PORT->MODER,
			0b11 << (2*SPIx_SS_PIN_Pos),
			0b01 << (2*SPIx_SS_PIN_Pos));

	// SPI config
	SPIx_CLK_ENABLE();
	SPIx->CR1 = SPI_CR1_SSM | SPI_CR1_SSI // software SS
			| (0b011 << SPI_CR1_BR_Pos) // assumes PCLK2 = 32MHz. divide by 16 for 2MHz
			| SPI_CR1_MSTR // master
			| SPI_CR1_CPOL // CPOL = 1
			| SPI_CR1_CPHA; // CPHA = 1
	SPIx->CR2 = SPI_CR2_FRXTH // 8-bit level for RXNE
			| (0b0111 << SPI_CR2_DS_Pos); // 8-bit data
	SPIx->CR1 |=  SPI_CR1_SPE; // enable SPI
}

static inline void ss_low(void)
{
	SPIx_SS_PORT->ODR &= ~SPIx_SS_PIN;
}

static inline void ss_high(void)
{
	SPIx_SS_PORT->ODR |= SPIx_SS_PIN;
}

static inline uint8_t spi_sendrecv(uint8_t b)
{
    while (!(SPI1->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI1->DR = b;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return *(__IO uint8_t *)&SPI1->DR;
}

#define spi_recv(x) spi_sendrecv(0)
#define spi_send(x) (void)spi_sendrecv(x)

static void spi_write(const uint8_t addr, const uint8_t data) {
	spi_send(addr | 0x80);
	spi_send(data);
	delay_us(180); // maximum of t_SWW, t_SWR
}

static uint8_t spi_read(const uint8_t addr) {
    spi_send(addr);
    delay_us(160); // t_SRAD
    uint8_t rd = spi_recv();
	delay_us(20);
	return rd;
}

static void pmw3360_init(const Config cfg)
{
	const uint8_t dpi = cfg.dpi;
	const uint8_t ang_snap = (cfg.flags & CONFIG_FLAGS_ANGLE_SNAP_ON) ? (1 << 7) : 0;
	const uint8_t lod = (cfg.flags & CONFIG_FLAGS_3MM_LOD) ? 0b11 : 0b10;

	ss_high();
	delay_ms(3);

	// shutdown first
	ss_low();
	spi_write(0x3b, 0xb6);
	ss_high();
	delay_ms(300);

	// drop and raise ncs to reset spi port
	ss_low();
	delay_us(40);
	ss_high();
	delay_us(40);

	// power up reset
	ss_low();
	spi_write(0x3a, 0x5a);
	ss_high();
	delay_ms(50);

	// read from 0x02 to 0x06
	ss_low();
	spi_read(0x02);
	spi_read(0x03);
	spi_read(0x04);
	spi_read(0x05);
	spi_read(0x06);

	spi_write(0x10, 0x00);
	spi_write(0x22, 0x00);

	// srom download
	spi_write(0x13, 0x1d);
	ss_high();
	delay_ms(10);
	ss_low();
	spi_write(0x13, 0x18);

	spi_send(0x62 | 0x80);
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		delay_us(16);
		spi_send(srom[i]);
	}
	delay_us(18);
	ss_high();
	delay_us(200);

	// check srom id
	ss_low();

	// configuration/settings
	spi_write(0x10, 0x00); // 0x20 (g502 default) enables rest mode after ~10s of inactivity
	spi_write(0x14, 0xff); // how long to wait before going to rest mode. 0xff is max (~10 seconds)
	spi_write(0x17, 0xff);
	spi_write(0x18, 0x00);
	spi_write(0x19, 0x00);
	spi_write(0x1b, 0x00);
	spi_write(0x1c, 0x00);

	spi_write(0x2c, 0x0a);
	spi_write(0x2b, 0x10);

	// configuration/settings
	spi_write(0x0f, dpi);
	spi_write(0x42, ang_snap); 	// angle snapping
	spi_write(0x63, lod); 	// angle snapping
	spi_write(0x0d, 0x60); 	// invert x,y
	ss_high();

	delay_us(100);

	// begin burst mode (write any value, i.e. 0x00 to burst mode register 0x50)
	ss_low();
	spi_write(0x50, 0x00);
	ss_high();
}

static void pmw3360_set_dpi(uint8_t dpi)
{
	ss_low();
	spi_write(0x0f, dpi);
	spi_write(0x50, 0x00); // return to burst mode
	ss_high();
}
