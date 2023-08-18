#ifndef __PROTOCOL_GB17691_H__ 
#define __PROTOCOL_GB17691_H__ 

#include "pdu.h"

#define FVHJ_SERVER 	3						//ȼ�ͳ���������·
#define FV17691_SERVER 	4					//ȼ�ͳ�17691��·

#define SIGNATURE_BIT 1 << 5			//����ǩ��
#define PACK_AUX_BIT	1 << 6			//����������λ

/*
proc: 0:17691 1:��������ǩ������ 2:����������ǩ�� 3:���������ݼ���
*/
void* gb17691Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t proc);
uint8_t gb17691Run(void* obj);
uint8_t gb17691GetSta(uint8_t id);
//��ȡ���ݻ���
RealData* getRealCache(uint8_t dataIdx,uint8_t* outDataIdx);

#endif
