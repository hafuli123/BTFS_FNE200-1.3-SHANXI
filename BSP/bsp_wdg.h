#ifndef _BSP_WDG_H
#define _BSP_WDG_H

#include "stdint.h"

uint8_t WDG_Init(void);
void WDG_Feed(int feedId);

#endif
