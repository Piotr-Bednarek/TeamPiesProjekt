#ifndef INIT_H_
#define INIT_H_

#include "stm32f7xx_hal.h"

// Map sysTick_Time to HAL_GetTick()
#define sysTick_Time HAL_GetTick()

#endif /* INIT_H_ */
