#ifndef __PROTOCOL_GB17691_H__ 
#define __PROTOCOL_GB17691_H__ 

#include "pdu.h"

#define FVHJ_SERVER 	3						//燃油车环境部链路
#define FV17691_SERVER 	4					//燃油车17691链路

#define SIGNATURE_BIT 1 << 5			//数据签名
#define PACK_AUX_BIT	1 << 6			//补充数据流位

/*
proc: 0:17691 1:环境部无签名加密 2:环境部数据签名 3:环境部数据加密
*/
void* gb17691Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t proc);
uint8_t gb17691Run(void* obj);
uint8_t gb17691GetSta(uint8_t id);
//获取数据缓存
RealData* getRealCache(uint8_t dataIdx,uint8_t* outDataIdx);

#endif
