#include "protocol_GB32960.h"
#include "stdio.h"
#include "string.h"
#include "user.h"
#include "bsp_rtc.h"
#include "Fun_Net.h"
#include "algo_verify.h"
#include "bsp_storage.h"

#define MAX_EXT_LINK 1

#define SWD_DATA_ADDR								0x08010000	/* 欣旺达基础数据， 64K	*/
#define SWD_DATA_ADDR1							0x080C0000	/* 欣旺达放电数据1，128K	*/
#define SWD_DATA_ADDR2							0x080E0000	/* 欣旺达充电数据2，128K	*/

static YJ_MESSAGE yjMsg;
static Flash_DATA1 cacheData;
static Flash_DATA2 cacheData1;

static uint8_t readSwdData(uint8_t idx);		//0 - 放电数据 1 - 充电数据
static uint8_t saveSwdData(uint8_t idx);		//0 - 放电数据 1 - 充电数据
static uint8_t chargeSta = 0xFF;						//0 - 放电 1 - 充电
static uint8_t runSta = 0;									//0 网络离线 1:网络在线 2:睡眠

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;									  //是否使用
	char *vin;										  //车架号
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffLen;								//发送数据缓冲区长度
	uint8_t isReady;								//允许上报电池安全预警数据
}GBSTA;

static GBSTA gbSta[MAX_EXT_LINK] = {0};

uint8_t saveSwdData(uint8_t idx)
{
	uint16_t storeFlag;
	uint16_t index = 0;
	uint8_t wbuf[4];
	uint32_t addr = 0;
	storeFlag = 0xAA55;
	
	wbuf[index++] = storeFlag >> 0;
	wbuf[index++] = storeFlag >> 8;
	wbuf[index++] = Cache_Version >> 0;
	wbuf[index++] = Cache_Version >> 8;
	
	Flash_Write(SWD_DATA_ADDR + addr,wbuf,4);
	addr += 4;
	
	Flash_Write(SWD_DATA_ADDR + addr,(uint8_t*)&cacheData,sizeof(Flash_DATA1) - (sizeof(Flash_DATA1) % 4) + 4);
	
	if(idx == 0)
	{
		Flash_Write(SWD_DATA_ADDR1,(uint8_t*)&cacheData1,sizeof(Flash_DATA2) - (sizeof(Flash_DATA2) % 4) + 4);
	}
	else if(idx == 1)
	{
		Flash_Write(SWD_DATA_ADDR2,(uint8_t*)&cacheData1,sizeof(Flash_DATA2) - (sizeof(Flash_DATA2) % 4) + 4);
	}
	return 0;
}

uint8_t readSwdData(uint8_t idx)
{
	uint16_t storeFlag,version;
	uint32_t addr = SWD_DATA_ADDR;
	memcpy(&storeFlag,(uint8_t*)addr,2);
	addr += 2;
	memcpy(&version,(uint8_t*)addr,2);
	addr += 2;
	if(storeFlag == 0xAA55 && version == Cache_Version)
	{
		memcpy(&cacheData,(uint8_t*)addr,sizeof(cacheData));
		if(idx == 0)
		{
			addr = SWD_DATA_ADDR1;
			memcpy(&cacheData1,(uint8_t*)addr,sizeof(cacheData1));
		}
		else if(idx == 1)
		{
			addr = SWD_DATA_ADDR2;
			memcpy(&cacheData1,(uint8_t*)addr,sizeof(cacheData1));
		}
	}
	else
	{
		saveSwdData(0);
		Flash_Write(SWD_DATA_ADDR2 ,(uint8_t*)&cacheData1,sizeof(Flash_DATA2) - (sizeof(Flash_DATA2) % 4) + 4);
	}
	return 0;
}

//添加数据报头及校验
static uint16_t MakeCmd(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *wbuf,uint16_t dataLen)
{
	wbuf[0] = 0x23;
	wbuf[1] = 0x23;
	wbuf[2] = cmd;
	wbuf[3] = rsp;
	memcpy(&wbuf[4],vin,17);
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

//打包欣旺达数据
static uint16_t packSwdData(char* vin,uint8_t *wbuf)
{
	uint16_t i,idx;
	
	idx = 24;
	wbuf[idx++] = (uint8_t)(g_system_dt.year - 2000);
	wbuf[idx++] = (uint8_t)(g_system_dt.month);
	wbuf[idx++] = (uint8_t)(g_system_dt.day);
	wbuf[idx++] = (uint8_t)(g_system_dt.hour);
	wbuf[idx++] = (uint8_t)(g_system_dt.minute);
	wbuf[idx++] = (uint8_t)(g_system_dt.second);
	//算法软件版本号
	wbuf[idx++] = 100;
	//算例life
	wbuf[idx++] = yjMsg.life1;
	wbuf[idx++] = yjMsg.life2;
	wbuf[idx++] = yjMsg.life3;
	//故障单体编号
	for(i = 0;i < 5;i++)
	{
		if(yjMsg.fault_cell_ID != NULL)
			wbuf[idx++] =  yjMsg.fault_cell_ID[i];
	}
	//故障类型编号
	for(i = 0;i < 5;i++)
	{
		if(yjMsg.fault_cell_Type != NULL)
			wbuf[idx++] =  yjMsg.fault_cell_Type[i];
	}
	//计算结果1
	wbuf[idx++] =  yjMsg.fault_OCVID;
	//计算结果2
	wbuf[idx++] = yjMsg.mean_OCV >> 8;
	wbuf[idx++] = yjMsg.mean_OCV >> 0;
	//计算结果3
	wbuf[idx++] = yjMsg.OCV_interval_CNT >> 8;
	wbuf[idx++] = yjMsg.OCV_interval_CNT >> 0;	
	//计算结果4
	wbuf[idx++] = cacheData.Cache_dSOC_Cycles_CNT >> 8;
	wbuf[idx++] = cacheData.Cache_dSOC_Cycles_CNT >> 0;	
	//计算结果5
	wbuf[idx++] = cacheData.Cache_dQ_Cycles_CNT >> 8;
	wbuf[idx++] = cacheData.Cache_dQ_Cycles_CNT >> 0;	
	//预留
	wbuf[idx++] = yjMsg.fault_cell_number;
	wbuf[idx++] = 0 >> 0;		
	return MakeCmd(0x09,0xFE,vin,wbuf,idx - 24);
}

void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_EXT_LINK;i++)
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
	gbSta[objLinkIdx].isReady = 0;
	//初始化电池预警数据
	memset(&cacheData,0,sizeof(cacheData));
	memset(&cacheData1,0,sizeof(cacheData1));
	memset(&yjMsg,0,sizeof(yjMsg));
	initialFunction(&cacheData,&cacheData1,&yjMsg);
	//默认启动读放电数据
	chargeSta = 0;
	readSwdData(0);
	//上电默认睡眠状态
	runSta = 2;
	return &gbSta[objLinkIdx];
}

uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	uint16_t dataLen = 0;
	//链路未配置
	if(pSta == NULL)
	{
		return 1;	//返回运行成功，允许上层协议登入
	}
	//睡眠时，存储电池预警数据
	if(ctrl == 2 && runSta != ctrl)
	{
		if(chargeSta == 0)
		{
			saveSwdData(0);
		}
		else
		{
			saveSwdData(1);
		}
	}
	runSta = ctrl;
	//上报或存储电池预警数据
	if(pSta->isReady == 1)
	{
		pSta->isReady = 0;
		dataLen = packSwdData(pSta->vin,pSta->buff);
		if(dataLen > 0)
		{
			if(sta == 1)
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			else
				saveHistory(pSta->bLink,0,pSta->buff,dataLen,0,REC);
		}
	}
	return 1;//返回运行成功，允许上层协议登入
}

uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	return 0;
}

uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	GBSTA *pSta = obj;
	if(pSta != NULL)
	{
		//上报实时数据，通知电池预警数据上报
		pSta->isReady = 1;
		yjMsg.UTC_TIME = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
		yjMsg.Nbat_s = gRealData.subSysData[0].singleVolCnt + gRealData.subSysData[1].singleVolCnt;
		yjMsg.SOC = gRealData.soc;
		yjMsg.Charge_state = (gRealData.chargeState == 1 ? 1 : 0);
		yjMsg.Current = gRealData.total_current;
		yjMsg.cellV = gRealData.single_vol;
		yjMsg.PackVoltage = gRealData.total_volt;
		if(chargeSta != yjMsg.Charge_state)
		{
			chargeSta = yjMsg.Charge_state;
			if(chargeSta == 0)
			{
				//存上一时刻车辆充电数据
				saveSwdData(1);
				//读取之前车辆启动数据
				readSwdData(0);
			}
			else
			{
				//存上一时刻车辆启动数据
				saveSwdData(0);
				//读取之前车辆充电数据
				readSwdData(1);
			}
		}
		chargeSta = yjMsg.Charge_state;
		battsafetyFunction1(&yjMsg);
		battsafetyFunction2(&cacheData,&cacheData1,&yjMsg);
		battsafetyFunction3(&cacheData,&yjMsg);
	}
	return 0;
}
