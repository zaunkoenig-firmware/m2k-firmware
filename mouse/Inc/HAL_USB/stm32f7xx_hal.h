#pragma once

#define HAL_PCD_MODULE_ENABLED

#define HSE_VALUE (24000000U) /*!< Value of the External oscillator in Hz */

#include "stm32f7xx_hal_pcd.h"

#include <delay.h>

#define HAL_Delay(x) delay_ms(x)
