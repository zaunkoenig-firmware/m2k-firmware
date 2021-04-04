#include "config.h"
#include "stm32f7xx.h"

// use flash sector 1, the 2nd 16kb (0x4000) sector
#define CONFIG_SECTOR_NUM  1
#define CONFIG_SECTOR_BASE (FLASHAXI_BASE + CONFIG_SECTOR_NUM*0x4000) // 0x08004000
#define CONFIG_SECTOR      ((__IO uint16_t *)CONFIG_SECTOR_BASE)
#define CONFIG_SECTOR_SIZE (0x4000 * sizeof(uint8_t)/sizeof(uint16_t))

static int config_index = -1; // set on first call to read_config

static const Config config_default = {
	.dpi = (800/100 - 1),
	.flags = 0
};

static void flash_unlock(void)
{
	FLASH->KEYR = 0x45670123; // ref manual pg 71
    FLASH->KEYR = 0xCDEF89AB;
}

static void flash_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_busy_wait(void)
{
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
}

static void flash_prog_u16(__IO uint16_t *addr, const uint16_t data)
{
	flash_busy_wait();
	MODIFY_REG(FLASH->CR,
			FLASH_CR_PSIZE,
			_VAL2FLD(FLASH_CR_PSIZE, 0b01) | FLASH_CR_PG); // 0b01 for 16-bit
	*addr = data;
	__DSB();
	flash_busy_wait();
	FLASH->CR &= ~FLASH_CR_PG;
}

static void flash_sector_erase(int sector)
{
	flash_busy_wait();
	// assume voltage range 2.7 - 3.6V for PSIZE (see ref manual pg 71)
	MODIFY_REG(FLASH->CR,
			FLASH_CR_PSIZE | FLASH_CR_SNB,
			_VAL2FLD(FLASH_CR_PSIZE, 0b10) | _VAL2FLD(FLASH_CR_SNB, sector) | FLASH_CR_SER);
	FLASH->CR |= FLASH_CR_STRT;
	__DSB();
	flash_busy_wait();
	FLASH->CR &= ~(FLASH_CR_SNB | FLASH_CR_SER);
}

// assumes all programmed bytes of a are before the empty bytes.
// returns index of highest programmed address (i.e. not 0xFFFF)
// or 0 if nothing is programmed yet
static int index_highest(const __IO uint16_t *a, const int len)
{
	int start = 0;
	int end = len;
	while (start + 1 < end) { // binary search
		int mid = (start + end)/2;
		if (a[mid] != 0xFFFF)
			start = mid;
		else
			end = mid;
	}
	return start;
}

Config config_read(void)
{
	if (config_index == -1) { // first call to function
		config_index = index_highest(CONFIG_SECTOR, CONFIG_SECTOR_SIZE);
		// write default cfg if sector is completely empty
		if (config_index == 0 && CONFIG_SECTOR[config_index] == 0xFFFF) {
			flash_unlock();
			flash_prog_u16(&CONFIG_SECTOR[config_index], config_default.u16);
			flash_lock();
		}
	}
	return (Config){.u16 = CONFIG_SECTOR[config_index]};
}

void config_write(Config cfg)
{
	config_index++;
	flash_unlock();
	if (config_index == CONFIG_SECTOR_SIZE) {
		config_index = 0;
		flash_sector_erase(CONFIG_SECTOR_NUM);
	}
	flash_prog_u16(&CONFIG_SECTOR[config_index], cfg.u16);
	flash_lock();
}
