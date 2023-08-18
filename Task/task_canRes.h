#ifndef _TASK_CANRES_H
#define _TASK_CANRES_H

#include "stdint.h"
#include "HCXY_DST/protocol_GB_EX_HCXY.h"

void task_canRes_Create(void);
extern uint8_t isResCANData(uint8_t ch,CAN_msg *msg);


#endif
