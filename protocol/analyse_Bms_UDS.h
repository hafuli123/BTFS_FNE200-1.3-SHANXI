#ifndef _UDS_H
#define _UDS_H


#include "fun_can.h"

void udsRecvData(uint8_t ch,CAN_msg *msg);


uint8_t NcmTxSuppoted(void);
uint8_t NcmRxSuppoted(void);
uint8_t NmcmTxSuppoted(void);
uint8_t NmcmRxSuppoted(void);

#endif
