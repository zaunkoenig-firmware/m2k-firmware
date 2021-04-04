#pragma once

#include <assert.h>
#include <stdint.h>
#include "cmsis_compiler.h"

// flags bits
//		|7		|6		|5		|4		|3		|2		|1		|0		|
// 0	|		|		|		|	Interval	|FS USB	|AS off	|2mm lod|
// 1	|		|		|		|	Interval	|HS USB	|AS on	|3mm lod|

// USB report rate:
//          |FS USB |HS USB |
//  Interval+-------+-------|
//     0b00 |   1ms | 125us |
//     0b01 |   2ms | 250us |
//     0b10 |   4ms | 500us |
//     0b11 |   8ms |   1ms |

#define CONFIG_FLAGS_3MM_LOD       (1 << 0)
#define CONFIG_FLAGS_ANGLE_SNAP_ON (1 << 1)
#define CONFIG_FLAGS_HS_USB        (1 << 2)
#define CONFIG_FLAGS_INTERVAL_Pos  3
#define CONFIG_FLAGS_INTERVAL_Msk (0b11 << 3)
#define CONFIG_FLAGS_INTERVAL      CONFIG_FLAGS_INTERVAL_Msk

typedef union {
	struct __PACKED {
		uint8_t dpi; // dpi in the form written to the sensor's register
		uint8_t flags;
	};
	uint16_t u16;
} Config;
static_assert(sizeof(Config) == sizeof(uint16_t), "Config wrong size");

Config config_read(void);
void config_write(Config cfg);
