/*
�� ����protocol_GB_EX_SHWX.c
�� �ܣ��Ϻ����� - ���������Զ�����չ����
�� ��: 2022/12/7
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: HYQ
*/

#include "FSFC/protocol_GB_EX_FSFC.h"
#define MAX_YZT_LINK 1

uint8_t isChgVIN = 0;
uint8_t isSendZDY = 0;                                          //0������80�Զ������� 1������81�Զ�������
//�Ϻ������Զ�������
SelfData80* pSelfData80 = NULL;					
SelfData81* pSelfData81 = NULL;

static uint16_t p80offset;
static uint16_t p81offset;


typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;										//�Ƿ�ʹ��
	char *vin;											//���ܺ�
	uint8_t bLogouted;							//�ǳ���־λ 0:δ�ǳ� 1:�ѵǳ�
	uint8_t sendOverTimeCnt;				//���ʹ���
	//�ڲ�ʱ���
	uint32_t chgPowerStamp;					//�������ϱ�ʱ���
	//��������
	uint8_t* buff;									//�������ݻ�����
	uint16_t buffLen;								//�������ݻ���������
}GBSTA;

static GBSTA gbSta[MAX_YZT_LINK] = {0};

//��չЭ���ʼ������������״̬����
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bUse == 1 && gbSta[i].bLink == link)
		{
			oldLinkIdx = i;//���³�ʼ��
		}
		if(gbSta[i].bUse == 0 && objLinkIdx == 0xFF)
		{
			objLinkIdx = i;
		}
	}
	if(oldLinkIdx != 0xFF)
	{
		objLinkIdx = oldLinkIdx;
	}
	if(objLinkIdx == 0xFF)
		return NULL;
	//�ⲿ����
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].buff = buff;
	gbSta[objLinkIdx].buffLen = buffLen;
	//�ڲ�
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
    
	//�Զ������ݷ����ڴ棬����ƫ����
	if(isSendZDY == 0)
	{
		p80offset = 0;
		pSelfData80 = (SelfData80*)&gRealData.externData[p80offset];
	}
	else if(isSendZDY == 1)
	{
		p81offset = 0;
		pSelfData81 = (SelfData81*)&gRealData.externData[p81offset]; 
	}
	
	return &gbSta[objLinkIdx];
}

//���80�Զ�������
static uint16_t Pack80Data(RealData *pRealData,uint8_t *bPackBuf)
{	
	uint16_t PackLen = 0, index = 0, u16Val = 0;
	uint8_t u8Val = 0;
	index = 3;
	SelfData80*  p80Data = (SelfData80*)&pRealData->externData[p80offset];		
	if(bPackBuf != 0)
	{
	  u8Val = p80Data->OutWaterTem;						//ˮ���¶�
		if(u8Val == 0xFE || u8Val == 0xFF)
		{
			bPackBuf[index++] = u8Val;	
		}
		else
		{
			bPackBuf[index++] = (uint8_t)((p80Data->OutWaterTem+40)*1);
		}
		 
		u16Val = (uint16_t)(p80Data->AirComVol);	//�յ�ѹ������ѹ
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p80Data->AirComVol+0)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}

		u16Val = (uint16_t)(p80Data->AirComCur);	//�յ�ѹ��������
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p80Data->AirComCur+1000)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		u16Val = (uint16_t)(p80Data->HyCyclePumpVol);	//����ѭ���õ�ѹ
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p80Data->HyCyclePumpVol+0)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}

		u16Val = (uint16_t)(p80Data->HyCyclePumpCur);	//����ѭ���õ���
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p80Data->HyCyclePumpCur+1000)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		u16Val = (uint16_t)(p80Data->InWaterTem);	//ˮ���¶�
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p80Data->InWaterTem+40)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		u16Val = (uint16_t)(p80Data->HySurplus);	//����ʣ����
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)(p80Data->HySurplus*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		bPackBuf[index++] = p80Data->AirConControlCommand;		//�յ�ָ��
		bPackBuf[index++] = p80Data->WarmRiskControlCommand;	//ů��ָ��

		PackLen = index - 3;
		
		bPackBuf[0] = 0x80;										//�Զ�������ָ����							
		bPackBuf[1] = (uint8_t)(PackLen>>8);
		bPackBuf[2] = (uint8_t)(PackLen>>0);
	}		
	return index;
}

//���81�Զ�������
static uint16_t Pack81Data(RealData *pRealData,uint8_t *bPackBuf)
{	
	uint16_t PackLen = 0, index = 0, u16Val = 0;
	uint8_t u8Val = 0;
	index = 3;
	SelfData81*  p81Data = (SelfData81*)&pRealData->externData[p81offset];		
	if(bPackBuf != 0)
	{
		u16Val = (uint16_t)(p81Data->AirComVol);
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p81Data->AirComVol+0)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}

		u16Val = (uint16_t)(p81Data->AirComPow);
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p81Data->AirComPow)*100);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		u16Val = (uint16_t)(p81Data->HyCyclePumpVol);
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p81Data->HyCyclePumpVol+0)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}

		u16Val = (uint16_t)(p81Data->HyCyclePumpCur);
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p81Data->HyCyclePumpCur+1000)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		u16Val = (uint16_t)(p81Data->InWaterTem);
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)((p81Data->InWaterTem+40)*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}
		
		u8Val = (uint8_t)p81Data->OutWaterTem; 
		if(u8Val == 0xFE || u8Val == 0xFF)
		{
			bPackBuf[index++] = u8Val;	
		}
		else
		{
			bPackBuf[index++] = (uint8_t)((p81Data->OutWaterTem+40)*1);
		}

		u16Val = (uint16_t)(p81Data->HySurplus);
		if(u16Val == 0xFFFE || u16Val == 0xFFFF)
		{
			bPackBuf[index++] = (uint8_t)(u16Val>>8);
			bPackBuf[index++] = (uint8_t)(u16Val>>0);
		}
		else
		{
			u16Val = (uint16_t)(p81Data->HySurplus*10);
			bPackBuf[index++] = (uint8_t)(u16Val>>8);	
			bPackBuf[index++] = (uint8_t)(u16Val>>0); 	
		}

		PackLen = index - 3;
		
		bPackBuf[0] = 0x81;										//�Զ�������ָ����							
		bPackBuf[1] = (uint8_t)(index>>8);
		bPackBuf[2] = (uint8_t)(index>>0);
	}		
	return index;
}


//��չʵʱ���ݣ�ʵʱ���������Զ�������
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;
	if(isSendZDY == 0)
	{
		index += Pack80Data(pRealData,&bPackBuf[index]);
	}
	else if(isSendZDY == 1)
	{
		index += Pack81Data(pRealData,&bPackBuf[index]);
	}
	
	return index;
}
