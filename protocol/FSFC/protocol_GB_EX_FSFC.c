/*
文 件：protocol_GB_EX_SHWX.c
功 能：上海万象 - 解析上送自定义扩展数据
日 期: 2022/12/7
公 司：北理新源(佛山)信息科技有限公司
作 者: HYQ
*/

#include "FSFC/protocol_GB_EX_FSFC.h"
#define MAX_YZT_LINK 1

uint8_t isChgVIN = 0;
uint8_t isSendZDY = 0;                                          //0：上送80自定义数据 1：上送81自定义数据
//上海万象自定义数据
SelfData80* pSelfData80 = NULL;					
SelfData81* pSelfData81 = NULL;

static uint16_t p80offset;
static uint16_t p81offset;


typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;										//是否使用
	char *vin;											//车架号
	uint8_t bLogouted;							//登出标志位 0:未登出 1:已登出
	uint8_t sendOverTimeCnt;				//发送次数
	//内部时间戳
	uint32_t chgPowerStamp;					//充电电量上报时间戳
	//其他参数
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffLen;								//发送数据缓冲区长度
}GBSTA;

static GBSTA gbSta[MAX_YZT_LINK] = {0};

//扩展协议初始化，返回配置状态参数
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bUse == 1 && gbSta[i].bLink == link)
		{
			oldLinkIdx = i;//重新初始化
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
	//外部参数
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].buff = buff;
	gbSta[objLinkIdx].buffLen = buffLen;
	//内部
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
    
	//自定义数据分配内存，设置偏移量
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

//打包80自定义数据
static uint16_t Pack80Data(RealData *pRealData,uint8_t *bPackBuf)
{	
	uint16_t PackLen = 0, index = 0, u16Val = 0;
	uint8_t u8Val = 0;
	index = 3;
	SelfData80*  p80Data = (SelfData80*)&pRealData->externData[p80offset];		
	if(bPackBuf != 0)
	{
	  u8Val = p80Data->OutWaterTem;						//水出温度
		if(u8Val == 0xFE || u8Val == 0xFF)
		{
			bPackBuf[index++] = u8Val;	
		}
		else
		{
			bPackBuf[index++] = (uint8_t)((p80Data->OutWaterTem+40)*1);
		}
		 
		u16Val = (uint16_t)(p80Data->AirComVol);	//空调压缩机电压
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

		u16Val = (uint16_t)(p80Data->AirComCur);	//空调压缩机电流
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
		
		u16Val = (uint16_t)(p80Data->HyCyclePumpVol);	//氢气循环泵电压
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

		u16Val = (uint16_t)(p80Data->HyCyclePumpCur);	//氢气循环泵电流
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
		
		u16Val = (uint16_t)(p80Data->InWaterTem);	//水入温度
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
		
		u16Val = (uint16_t)(p80Data->HySurplus);	//氢气剩余量
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
		
		bPackBuf[index++] = p80Data->AirConControlCommand;		//空调指令
		bPackBuf[index++] = p80Data->WarmRiskControlCommand;	//暖风指令

		PackLen = index - 3;
		
		bPackBuf[0] = 0x80;										//自定义数据指令码							
		bPackBuf[1] = (uint8_t)(PackLen>>8);
		bPackBuf[2] = (uint8_t)(PackLen>>0);
	}		
	return index;
}

//打包81自定义数据
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
		
		bPackBuf[0] = 0x81;										//自定义数据指令码							
		bPackBuf[1] = (uint8_t)(index>>8);
		bPackBuf[2] = (uint8_t)(index>>0);
	}		
	return index;
}


//扩展实时数据，实时数据增加自定义数据
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
