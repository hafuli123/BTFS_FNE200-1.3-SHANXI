/*
 * newant_SM2.h
 *
 *  Created on: 2019Äê1ÔÂ29ÈÕ
 *      Author: xtgaxm
 */


#ifndef NEWANT_SM2_H_
#define NEWANT_SM2_H_

#include <RTX_CAN.h>
#include "pdu.h"
#include "stm32f2xx.h"
#include "bsp_sys.h"

void SM2_IOInit(void);
uint8_t SM2_GetID(uint8_t* buf);
uint8_t SM2_SetID(uint8_t* buf);
uint8_t SM2_GetPubKey(u8 select,u8* buf);
uint8_t* SM2_PrivKeySign(uint8_t *source,uint16_t courceLen);
uint8_t* SM2_PubKeyEncryp(uint8_t select,uint8_t *source,uint16_t courceLen);
uint8_t* SM2_PriDecryp(uint8_t *source,uint16_t courceLen);
uint8_t SM2_SetPubKey(uint8_t select,uint8_t * PubKey);
uint8_t SM2_GenerateKey(void);
#endif
