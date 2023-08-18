#ifndef __TERMINAL_SERVER_H
#define __TERMINAL_SERVER_H
#include "stdint.h"


void InitTermServer(void);
void termServerTask(void);

uint8_t extSetPara(uint8_t check,uint8_t setParaId,const uint8_t* setBuff);
uint8_t extReadPara(uint8_t readParaId,uint8_t* readBuff);
#endif
