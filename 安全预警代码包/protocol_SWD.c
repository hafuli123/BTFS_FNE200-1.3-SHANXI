#include "protocol_GB32960.h"
#include "stdio.h"
#include "string.h"
#include "user.h"
#include "bsp_rtc.h"
#include "Fun_Net.h"
#include "algo_verify.h"
#include "bsp_storage.h"

#define MAX_EXT_LINK 1

#define SWD_DATA_ADDR								0x08010000	/* ������������ݣ� 64K	*/
#define SWD_DATA_ADDR1							0x080C0000	/* ������ŵ�����1��128K	*/
#define SWD_DATA_ADDR2							0x080E0000	/* ������������2��128K	*/

static YJ_MESSAGE yjMsg;
static Flash_DATA1 cacheData;
static Flash_DATA2 cacheData1;

static uint8_t readSwdData(uint8_t idx);		//0 - �ŵ����� 1 - �������
static uint8_t saveSwdData(uint8_t idx);		//0 - �ŵ����� 1 - �������
static uint8_t chargeSta = 0xFF;						//0 - �ŵ� 1 - ���
static uint8_t runSta = 0;									//0 �������� 1:�������� 2:˯��

typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;									  //�Ƿ�ʹ��
	char *vin;										  //���ܺ�
	uint8_t* buff;									//�������ݻ�����
	uint16_t buffLen;								//�������ݻ���������
	uint8_t isReady;								//�����ϱ���ذ�ȫԤ������
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

//������ݱ�ͷ��У��
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

//�������������
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
	//�㷨����汾��
	wbuf[idx++] = 100;
	//����life
	wbuf[idx++] = yjMsg.life1;
	wbuf[idx++] = yjMsg.life2;
	wbuf[idx++] = yjMsg.life3;
	//���ϵ�����
	for(i = 0;i < 5;i++)
	{
		if(yjMsg.fault_cell_ID != NULL)
			wbuf[idx++] =  yjMsg.fault_cell_ID[i];
	}
	//�������ͱ��
	for(i = 0;i < 5;i++)
	{
		if(yjMsg.fault_cell_Type != NULL)
			wbuf[idx++] =  yjMsg.fault_cell_Type[i];
	}
	//������1
	wbuf[idx++] =  yjMsg.fault_OCVID;
	//������2
	wbuf[idx++] = yjMsg.mean_OCV >> 8;
	wbuf[idx++] = yjMsg.mean_OCV >> 0;
	//������3
	wbuf[idx++] = yjMsg.OCV_interval_CNT >> 8;
	wbuf[idx++] = yjMsg.OCV_interval_CNT >> 0;	
	//������4
	wbuf[idx++] = cacheData.Cache_dSOC_Cycles_CNT >> 8;
	wbuf[idx++] = cacheData.Cache_dSOC_Cycles_CNT >> 0;	
	//������5
	wbuf[idx++] = cacheData.Cache_dQ_Cycles_CNT >> 8;
	wbuf[idx++] = cacheData.Cache_dQ_Cycles_CNT >> 0;	
	//Ԥ��
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
	gbSta[objLinkIdx].isReady = 0;
	//��ʼ�����Ԥ������
	memset(&cacheData,0,sizeof(cacheData));
	memset(&cacheData1,0,sizeof(cacheData1));
	memset(&yjMsg,0,sizeof(yjMsg));
	initialFunction(&cacheData,&cacheData1,&yjMsg);
	//Ĭ���������ŵ�����
	chargeSta = 0;
	readSwdData(0);
	//�ϵ�Ĭ��˯��״̬
	runSta = 2;
	return &gbSta[objLinkIdx];
}

uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	uint16_t dataLen = 0;
	//��·δ����
	if(pSta == NULL)
	{
		return 1;	//�������гɹ��������ϲ�Э�����
	}
	//˯��ʱ���洢���Ԥ������
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
	//�ϱ���洢���Ԥ������
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
	return 1;//�������гɹ��������ϲ�Э�����
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
		//�ϱ�ʵʱ���ݣ�֪ͨ���Ԥ�������ϱ�
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
				//����һʱ�̳����������
				saveSwdData(1);
				//��ȡ֮ǰ������������
				readSwdData(0);
			}
			else
			{
				//����һʱ�̳�����������
				saveSwdData(0);
				//��ȡ֮ǰ�����������
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
