#ifndef _ALGO_VERIFY_H
#define _ALGO_VERIFY_H

#include "stdint.h"

uint8_t getBccCode(const uint8_t *buff, uint16_t StartIdx, int16_t len);
uint8_t CheckVin(char* vin);

#endif
