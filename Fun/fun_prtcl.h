#ifndef __FUN_PRTCL_H
#define __FUN_PRTCL_H

#include "stdint.h"
#include "pdu.h"

extern void funPrtclInit(void);
extern void funPrtclrun(void);
extern RealData* getRealCache(uint8_t dataIdx,uint8_t* outDataIdx);

#endif
