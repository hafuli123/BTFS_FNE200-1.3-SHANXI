#include "bsp_sys.h"
#include "protocol_GB32960.h"
#include "bsp_storage.h"
#include "bsp_rtc.h"
#include "string.h"
#include "stdio.h"
#include "algo_verify.h"
#include "Fun_Net.h"
#include "bsp_uart_fifo.h"
#include "fun_can.h"
#include "fun_mon.h"



#define MAX_GB_LINK 3
#define SHDB_STA 0
enum COMMAND_CODE
{
	CMD_NONE = 0,
	CMD_LOGIN,                 			//��������
	CMD_REALDATA,              			//ʵʱ����
	CMD_REISSUEDATA,           			//��������
	CMD_LOGOUT,                			//�����ǳ�
	CMD_HEARTBEAT = 0x07,      			//����
	CMD_TIMING,               			//Уʱ
	CMD_QUERYPARAS = 0x80,     			//��ѯ����
	CMD_SETPARAS,              			//��������
	CMD_CTRL,                  			//��������
};

//���籣����·����
typedef struct _CFG
{
	uint32_t regTimeStamp;					//ע��ʱ�����־
	uint16_t regNo;									//ע����ˮ��
}CFG;

typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;										//�Ƿ�ʹ��
	uint8_t ctrl;										//��·�������

	char *vin;											//���ܺ�
	char* domain;										//��·����
	uint32_t* port;									//��·�˿ں�	
	uint8_t realHz;									//ʵʱ��������
	uint8_t heartbeatHz;						//��������
	//ƽ̨����״̬����
	uint8_t bLogined;       				//�����־λ 0:δ���� 1:�ѵ��� 0xFF:���͵�����
	uint8_t bLogouted;							//�ǳ���־λ 0:δ�ǳ� 1:�ѵǳ�
	uint8_t bTiming;								//Уʱ��־
	uint8_t sendOverTimeCnt;				//���ͳ�ʱ����
	uint8_t reissueSta;							//����״̬��0����Ҫ���� 1���������
	//ʵʱ��ʱ����
	uint8_t realSec;								//ʵʱ����
	uint8_t realSecCnt;							//ʵʱ�����
	//3����������
	uint8_t lastAlarmLevel;					//��һ�α����ȼ�
	uint8_t afterAlarm1hzCnt;				//������1��Ƶ������
	uint8_t afterAlarmSecCnt;			  //���������¼�ʱ������Ƶ������3��������ʵʱ30�벿�ֲ��ò���
	//�ڲ�ʱ���
	uint32_t heartbeatStamp;				//����ʱ���
	uint32_t reissueStamp;					//����ʱ���
	//��������
	uint8_t* buff;									//�������ݻ�����
	uint16_t buffSize;							//�������ݻ�������С
	//�����ļ���
	char reissueFileName[30];				//�����ļ���
	uint8_t day;										//��
	void *extCfg;										//��չ����
	uint8_t extSta;									//��չЭ��״̬
	//�ڲ����籣�����
	CFG cfg;
}GBSTA;

static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo);							//��������
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen);														//�ն�Уʱ
static uint16_t packHeartBeat(char* vin,uint8_t* buff,uint16_t maxLen);													//�ն�����
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo);							//�����ǳ�
static uint16_t buildRealData(GBSTA *pSta,uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,RealData *pRealData);//���ʵʱ����
static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen);															//�������

static GBSTA gbSta[MAX_GB_LINK] = {0};																													//��·������̬
static uint8_t printPackCmd = 0;																																//��ӡ��������
static uint8_t logoutTestCmd = 0;																																//�ǳ���������
static uint32_t ctrlOfflineStaTimeStamp = 0;																										//���߲���ʱ���¼
static uint8_t ctrlOfflineSta = 0;																															//���߲�������
static uint8_t ctrlAlarmSta = 0;																																//������������
static uint32_t ctrlAlarmStaTimeStamp = 0;																											//��������ʱ���¼

/*��ʼ������*/
void* gb32960Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t realHz,uint8_t heartbeatHz)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_GB_LINK;i++)
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
	realHz = (realHz == 0 || realHz > 60) ? 10 : realHz;
	heartbeatHz = (heartbeatHz < 10 || heartbeatHz > 120) ? 30 : heartbeatHz;
	//�ⲿ����
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].domain = domain;
	gbSta[objLinkIdx].port = port;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].heartbeatHz = heartbeatHz;
	gbSta[objLinkIdx].realHz = realHz;	
	//�ڲ�
	gbSta[objLinkIdx].bLogined = 0;
	gbSta[objLinkIdx].bLogouted = 0;
	gbSta[objLinkIdx].bTiming = 0;
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].reissueSta = 0;
	
	gbSta[objLinkIdx].realSec = 0xFF;
	gbSta[objLinkIdx].realSecCnt = 0;
	gbSta[objLinkIdx].lastAlarmLevel = 0;
	gbSta[objLinkIdx].afterAlarm1hzCnt = 0;
	gbSta[objLinkIdx].afterAlarmSecCnt = 60;
	gbSta[objLinkIdx].heartbeatStamp = 0;
	gbSta[objLinkIdx].reissueStamp = 0;
	
	gbSta[objLinkIdx].buff = pBuff;
	gbSta[objLinkIdx].buffSize = pBuffSize;
	gbSta[objLinkIdx].day = g_system_dt.day;
	//����洢�����ָ�
	readConfig(link,&gbSta[objLinkIdx].cfg,sizeof(gbSta[objLinkIdx].cfg),0);
	//ͬ����־�ļ�
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),LOG);
	//ͬ�������ļ�
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),REC);
	if(objLinkIdx == 0)
	{
		gbSta[objLinkIdx].extCfg = extInit(gbSta[objLinkIdx].bLink,vin,0,gbSta[objLinkIdx].buff,gbSta[objLinkIdx].buffSize);
	}
	else
		gbSta[objLinkIdx].extCfg = 0;
	//���ص�ǰ��·����״ָ̬��
	return &gbSta[objLinkIdx];
}

void cleanPara(void* obj)
{
	GBSTA *pSta = obj;
	if((gSysPara.linkSwitch & (1 << pSta->bLink)) == 0)
	{
		Fun_Gprs_Tcp_disconnect(pSta->bLink);
		pSta->ctrl = 0;
	}
	pSta->reissueSta = 0;
	pSta->bLogined = 0;
	pSta->bLogouted = 0;
	pSta->bTiming = 0;	
}

static uint16_t realDataReadyLen(GBSTA *pSta)
{
	RealData* pRealData;
	uint8_t outDataIdx;
	uint16_t dataLen = 0;
	if(pSta->realSec != gRealData.second)
	{
		pSta->bTiming = (pSta->bTiming >= 1 && pSta->bTiming < 0xFF) ? pSta->bTiming + 1: pSta->bTiming;
		pSta->realSec = gRealData.second;
		pSta->afterAlarmSecCnt = pSta->afterAlarmSecCnt < 60 ? pSta->afterAlarmSecCnt + 1 : 60;
		//������������
		if(ctrlAlarmSta == 1)
		{
			gRealData.alarmLevel = 3;
			gRealData.batHighTempAlert = 1;
		} 
		//������������
		if(gRealData.alarmLevel == 3 && pSta->lastAlarmLevel != gRealData.alarmLevel)
		{
			//���Ʊ�����������
			uint8_t reissueCnt = pSta->afterAlarmSecCnt - 30;
			//����״̬����
			pSta->reissueSta = 0;
			//�洢��������
			for(;reissueCnt > 0;reissueCnt--)
			{
				pRealData = getRealCache(reissueCnt,&outDataIdx);
				if(outDataIdx != 0xFF)
				{
					reissueCnt = outDataIdx;
					dataLen = buildRealData(pSta,CMD_REISSUEDATA,pSta->vin,pSta->buff,pSta->buffSize,pRealData);
					if(dataLen > 0)
					{
						//�洢����ǰ30�����ݣ����ڲ���
						saveHistory(pSta->bLink,0,pSta->buff,dataLen,0,REC);
					}
				}
				else
				{
					break;
				}
			}
			dataLen = 0;
			pSta->afterAlarm1hzCnt = 30;
			pSta->afterAlarmSecCnt = 0;
		}
		if(ctrlAlarmSta == 1)
		{
			gRealData.alarmLevel = 3;
			gRealData.batHighTempAlert = 1;
			if(osKernelGetTickCount() - ctrlAlarmStaTimeStamp > 180000)
			{
				ctrlAlarmSta = 0;
			}
		}
		pSta->lastAlarmLevel = gRealData.alarmLevel;
		//���ڿ��� | ����30��
		if(++pSta->realSecCnt >= pSta->realHz || pSta->afterAlarm1hzCnt > 0)
		{
			pSta->realSecCnt = 0;
			pSta->afterAlarm1hzCnt = (pSta->afterAlarm1hzCnt > 0 ? pSta->afterAlarm1hzCnt - 1 : 0 );
			pRealData = getRealCache(0,&outDataIdx);
			if(outDataIdx != 0xFF)
			{
				dataLen = buildRealData(pSta,pSta->bLogined != 1 ? CMD_REISSUEDATA : CMD_REALDATA,pSta->vin,pSta->buff,pSta->buffSize,pRealData);
				if(dataLen > 0 && pSta->bLogined != 1)
				{
					//�洢ʵʱ����,���ڲ���
					saveHistory(pSta->bLink,pSta->bLogined,pSta->buff,dataLen,0,REC);
				}
			}
			else
			{
				dataLen = 0;
			}
		}
	}
	return dataLen;
}

static uint8_t sendNetData(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	uint16_t sendLen = (buf[22] << 8 | buf[23] << 0) + 25;

//		comSendBuf(RS1_COM,buf,len);
	
	Fun_Gprs_Tcp_send(socket_fd,buf,sendLen);
	if(sendLen < len)
	{
		Fun_Gprs_Tcp_send(socket_fd,&buf[sendLen],len - sendLen);
	}
	if(buf[2] >= CMD_LOGIN && buf[2] <= CMD_LOGOUT)
	{
		saveHistory(socket_fd,1,buf,len,0,LOG);
	}
	return 0;
}
//uint32_t gRealDataSendCnt = 0;	//��¼ʵʱ����֡����
/*��������·״̬���� ���� 0:���Ե�¼ƽ̨ 1:�ǳ�ƽ̨*/
uint8_t gb32960Run(void* obj)
{
	static uint32_t startTicks = 0,endTicks = 0;
	uint8_t active = 0;
	uint32_t ticks = osKernelGetTickCount();
	uint16_t dataLen = 0;
	uint8_t svrSta = 0;
	GBSTA *pSta = obj;
	if(pSta->day != g_system_dt.day)
	{
		pSta->day = g_system_dt.day;
		//ͬ����־�ļ�
		syncHistory(pSta->bLink,pSta->reissueFileName,sizeof(pSta->reissueFileName),LOG);
		//ͬ�������ļ�
		syncHistory(pSta->bLink,pSta->reissueFileName,sizeof(pSta->reissueFileName),REC);		
	}
	if(Fun_Gprs_Tcp_Sta(pSta->bLink) == FUN_GPRS_CONNECTED && ctrlOfflineSta == 0 && Fun_Gprs_Csq()>14)
	{
		svrSta = 1;
	}
	if(extCtrlNetSta(pSta->extCfg) == 1 ||(fun_can_Get_State(BSP_CAN) > 0 && (gRealData.carState == 1 || gRealData.chargeState == 1)))
	{
		//����Ͽ���������������
		if(svrSta == 0)
		{
			Fun_Gprs_Tcp_connect(pSta->bLink,pSta->domain,*pSta->port);
			Fun_Gprs_Tcp_Set_RecvCallBack(pSta->bLink,unpack);
		}
		startTicks = ticks;
		if(ticks - endTicks >= 10000)
		{
			pSta->ctrl = 1;
		}
	}
	else
	{
		endTicks = ticks;
		if(gRealData.chargeState == 4)
		{
			if(ticks - startTicks >= 360000)
			{
				pSta->ctrl = 0;
			}
		}
		else
		{
			if(ticks - startTicks >= 60000)
			{
				pSta->ctrl = 0;
			}			
		}
		if(pSta->ctrl == 0 && svrSta == 1 && pSta->bLogined != 1)
		{
			Fun_Gprs_Tcp_disconnect(pSta->bLink);
		}
	}
	//���Ƶǳ�
	if(logoutTestCmd == 1)
	{
		pSta->ctrl = 0;
	}
	//����״̬��ѯ�����
	if(ticks - ctrlOfflineStaTimeStamp > 800000)
	{
		ctrlOfflineSta = 0;
	}
	//�޷���������·δʹ������ȫֹͣ����ر�������
	if(svrSta == 0 || pSta->bLogined == 0  || pSta->bLogouted == 1)
	{
		cleanPara(pSta);
	}
	//���������ն��Ѿ�������δ�ǳ�
	if(pSta->ctrl == 1 || (pSta->bLogined == 1 && pSta->bLogouted == 0))
	{
		active = 1;
		dataLen = realDataReadyLen(pSta);
		//����������ƽ̨����
		if(svrSta == 1)
		{
			//����3�γ�3����Ӧ��ʱ,�ز�
			if(pSta->sendOverTimeCnt >= 3 &&  ticks - pSta->heartbeatStamp >= 3000)
			{
				pSta->sendOverTimeCnt = 0;
				Fun_Gprs_Tcp_disconnect(pSta->bLink);
				cleanPara(pSta);		
			}
			//�����������������
			else if(pSta->bLogined == 1)
			{
				if(dataLen > 0)//����ʵʱ����
				{
					sendNetData(pSta->bLink,pSta->buff,dataLen);
//					gRealDataSendCnt++;
				}
				else if(pSta->bTiming == 0)//Уʱ
				{
					pSta->bTiming = 1;
					if((dataLen = packTiming(pSta->vin,pSta->buff,pSta->buffSize)) > 0)
						sendNetData(pSta->bLink,pSta->buff,dataLen);
				}
				else if(pSta->reissueFileName[0] != 0 && ticks - pSta->reissueStamp >= 500 && pSta->afterAlarm1hzCnt == 0)//��������
				{
					pSta->reissueStamp = ticks;
					dataLen = readHistory(pSta->bLink,pSta->reissueFileName,pSta->buff,pSta->buffSize,REC);
					if(dataLen > 0)
					{
						sendNetData(pSta->bLink,pSta->buff,dataLen);
					}
					else
						pSta->reissueFileName[0] = 0;//��ǰ�ļ��������
				}
				else if(pSta->reissueSta == 0 && pSta->reissueFileName[0] == 0 && ticks - pSta->reissueStamp >= 1000)//ͬ�������ļ�
				{
					if(syncHistory(pSta->bLink,pSta->reissueFileName,sizeof(pSta->reissueFileName),REC) == 0)
					{
						pSta->reissueSta = 1;
					}
				}
				else if(ticks - pSta->heartbeatStamp >= pSta->heartbeatHz * 1000)//����
				{
					pSta->heartbeatStamp = ticks;
					pSta->sendOverTimeCnt++;
					if((dataLen = packHeartBeat(pSta->vin,pSta->buff,pSta->buffSize)) > 0)
						sendNetData(pSta->bLink,pSta->buff,dataLen);
					
				}
				else if(pSta->ctrl == 0 && pSta->bLogouted == 0)//�ǳ�
				{
					pSta->bLogouted = 1;
					if((dataLen = packLogout(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo)) > 0)
					{
						sendNetData(pSta->bLink,pSta->buff,dataLen);
					}
				}
			}
			else if(pSta->extSta == 1 && ticks - pSta->heartbeatStamp >= 20000 && pSta->bLogined != 1)//����ƽ̨����
			{
				uint32_t regTimeStamp = g_system_dt.year << 16 | g_system_dt.month << 8 | g_system_dt.day << 0;
				pSta->heartbeatStamp = ticks;
				pSta->bLogined = 0xFF;
				//��ˮ��
				if(pSta->cfg.regTimeStamp != regTimeStamp)
				{
					pSta->cfg.regTimeStamp = regTimeStamp;
					pSta->cfg.regNo = 0;
				}
				pSta->cfg.regNo++;
				saveConfig(pSta->bLink,&pSta->cfg,sizeof(pSta->cfg),0);
				//���
				if((dataLen = packLogin(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo)) > 0)
				{
					sendNetData(pSta->bLink,pSta->buff,dataLen);
				}
				pSta->sendOverTimeCnt++;
			}
		}
	}
	//������չЭ��
	pSta->extSta = extRun(pSta->extCfg,(pSta->ctrl == 1 || (pSta->bLogined == 1 && pSta->bLogouted == 0)) ? svrSta : 2,(pSta->bLogined == 1 &&  pSta->bLogouted == 0) ? 1 : 0);
	return active;
}

/*��ȡ��·״̬*/
uint8_t gb32960GetSta(uint8_t id)
{
	if(id < MAX_GB_LINK)
		return (gbSta[id].bLogined == 1);
	else if(id == 0xFF)
	{
		for(id = 0;id < MAX_GB_LINK;id++)
		{
			if(gbSta[id].bLogined == 1)
			{
				return 1;
			}
		}
	}
	return 0;
}

uint8_t gb32960Ctrl(uint8_t ctrl)
{
	switch(ctrl)
	{
		case GB32960_TEST_LOGIN:logoutTestCmd = 0;break;
		case GB32960_TEST_LOGOUT:logoutTestCmd = 1;break;
		case GB32960_TEST_ONLINE:ctrlOfflineSta = 0;break;
		case GB32960_TEST_OFFLINE:ctrlOfflineSta = 1;ctrlOfflineStaTimeStamp = osKernelGetTickCount();break;
		case GB32960_TEST_UNALARM:ctrlAlarmSta = 0;break;
		case GB32960_TEST_ALARM:ctrlAlarmSta = 1;ctrlAlarmStaTimeStamp = osKernelGetTickCount();break;
		case GB32960_TEST_UNPRINT:printPackCmd = 0;break;
		case GB32960_TEST_PRINT:printPackCmd = 1;break;
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
	extGetVin(cmd,rsp,(char*)&wbuf[4],NULL);
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

//�����������
static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo)
{
	uint16_t DataAreLen = 0;
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	DataAreLen = 6;
	buff[30] = (regNo) >> 8;         	//ע����ˮ��H
	buff[31] = (regNo);
	DataAreLen += 2;
	Fun_Gprs_getICCID((char*)&buff[32],20);
	DataAreLen += 20;
	buff[52] = gRealData.subBatSysCnt;        			//�ɳ�索����ϵͳ����
	DataAreLen++;
	buff[53] = 0;
	DataAreLen++;
	if(gRealData.subBatSysCnt > 0 && gRealData.subBatSysCnt <= 2 && gRealData.rechargeSysCodeLen > 0	&&  gRealData.rechargeSysCodeLen <= 50)
	{
		uint8_t i = 0;
		buff[53] = gRealData.rechargeSysCodeLen; //�ɳ�索��ϵͳ���볤��
		for(i = 0;i < gRealData.subBatSysCnt;i++)
		{
			memcpy(&buff[54 + i * gRealData.rechargeSysCodeLen],gRealData.subSysData[i].rechargeSysCode,gRealData.rechargeSysCodeLen);
			DataAreLen += gRealData.rechargeSysCodeLen;
		}
	}
	return MakeCmd(CMD_LOGIN,0xFE,vin,buff,DataAreLen);
}

//���Уʱ����
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen)
{
	return MakeCmd(CMD_TIMING,0xFE,vin,buff,0);	
}

//�����������
static uint16_t packHeartBeat(char* vin,uint8_t* buff,uint16_t maxLen)
{
	return MakeCmd(CMD_HEARTBEAT,0xFE,vin,buff,0);
}

//����ǳ�����
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo)
{
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	buff[30] = (regNo) >> 8;
	buff[31] = (regNo);
	return MakeCmd(CMD_LOGOUT,0xFE,vin,buff,8);
}

/*����������� 21�ֽ�*/
static uint8_t PackVehicleData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t usVal = 0;
	uint32_t dwVal = 0;
	uint8_t Idx = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x01;
		bPackBuf[Idx++] = pRealData->carState;                     //����״̬
		bPackBuf[Idx++] = pRealData->chargeState;                  //���״̬
		bPackBuf[Idx++] = pRealData->operationState;               //����ģʽ
		
		usVal = (uint16_t)(pRealData->speed);                      //����
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->speed * 10);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
		dwVal = (uint32_t)(pRealData->totalMileage);               //���
		if(dwVal != 0xFFFFFFFF  && dwVal != 0xFFFFFFFE)
			dwVal = (uint32_t)(pRealData->totalMileage * 10);
		else
			dwVal = 0xFFFFFFFE;
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 24);
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 16);
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 0);
		
		usVal = (uint16_t)(pRealData->total_volt);                 //�ܵ�ѹ
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->total_volt * 10);          //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
		usVal = (uint16_t)(pRealData->total_current);               //�ܵ���
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->total_current * 10 + 10000);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);	
		
		bPackBuf[Idx++] = pRealData->soc;                           //SOC		
		bPackBuf[Idx++] = pRealData->dc2dcState;                    //DC-DC״̬	
		bPackBuf[Idx++] = pRealData->stall;                         //��λ		   
		bPackBuf[Idx++] = (uint8_t)(pRealData->mohm >> 8);          //��Ե����
		bPackBuf[Idx++] = (uint8_t)(pRealData->mohm >> 0);		
		bPackBuf[Idx++] = pRealData->acceleratorVal;                //����̤���г�ֵ
		bPackBuf[Idx++] = pRealData->brakingVal;                    //�ƶ�̤���г�ֵ
	}
	return Idx;
}

/*��������������*/
static uint16_t PackMotorData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0, usVal = 0;
	uint8_t i = 0;
	int16_t sVal = 0;
	
	if(bPackBuf != 0 && pRealData->motorCnt >= 1 && pRealData->motorCnt <= 2 && pRealData->chargeState != 1 && pRealData->chargeState != 4)
	{
		bPackBuf[Idx++] = 0x02;
		bPackBuf[Idx++] = pRealData->motorCnt;
		for(i = 0; i < pRealData->motorCnt; i++)
		{
			bPackBuf[Idx++] = pRealData->motorData[i].motorIdx;                    //����������
			bPackBuf[Idx++] = pRealData->motorData[i].motorState;                  //�������״̬
			
			sVal = pRealData->motorData[i].motorCtrTemp;                           //��������������¶�
			if(sVal < 0xFE)
				sVal = sVal + 40;
			bPackBuf[Idx++] = (uint8_t)sVal;
			
			usVal = pRealData->motorData[i].motorSpeed;                            //�������ת��
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = usVal + 20000;
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
			
			usVal = (uint16_t)pRealData->motorData[i].motorTorsion;                //�������ת��
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = (uint16_t)(pRealData->motorData[i].motorTorsion * 10 + 20000);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
			
			sVal = pRealData->motorData[i].motorTemp;                              //��������¶�
			if(sVal < 0xFE)
				sVal = sVal + 40;
			bPackBuf[Idx++] = (uint8_t)sVal;
			
			usVal = (uint16_t)(pRealData->motorData[i].motorVol);                  //��������������ѹ
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = (uint16_t)(pRealData->motorData[i].motorVol * 10);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
			usVal = (uint16_t)(pRealData->motorData[i].motorCur);                  //���������ֱ��ĸ�ߵ���
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = (uint16_t)(pRealData->motorData[i].motorCur * 10 + 10000);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		}
	}
	return Idx;
}

/*�������������*/
static uint16_t PackEngineData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0, usVal = 0;
	if(bPackBuf != 0 && pRealData->engineState < 0xFE && pRealData->fuelConsumption < 0xFFFE && pRealData->chargeState != 1 && pRealData->chargeState != 4)
	{
		bPackBuf[Idx++] = 0x04;
		bPackBuf[Idx++] = pRealData->engineState;                   //������״̬
		
		usVal = pRealData->crankshaftSpeed;                         //����ת��
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
		usVal = (uint16_t)(pRealData->fuelConsumption);             //ȼ��������
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->fuelConsumption * 100);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
	}
	return Idx;
}

/*���ȼ�ϵ������*/
static uint16_t PackFuelBatData(RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t Idx = 0, usVal = 0;
	uint8_t FuelBatTemCnt = 0, i = 0;
	int16_t sVal = 0;
	
	if(bPackBuf != 0 && pRealData->fuelBatVol < 0xFFFE && pRealData->maxHydrPressure < 0xFFFE && 18 + pRealData->fuelBatTemCnt < RemainLen )
	{
		bPackBuf[Idx++] = 0x03;
		usVal = (uint16_t)(pRealData->fuelBatVol);                  //ȼ�ϵ�ص�ѹ
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->fuelBatVol * 10);           //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		usVal = (uint16_t)(pRealData->fuelBatCur);                  //ȼ�ϵ�ص���
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->fuelBatCur * 10);          //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal / 256);
		bPackBuf[Idx++] = (uint8_t)(usVal % 256);
		
		usVal = (uint16_t)(pRealData->batFuelConsumption);         //���ȼ��������
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->batFuelConsumption * 100);//usVal * 100;
		
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		FuelBatTemCnt = ((pRealData->fuelBatTemCnt <= countof(pRealData->fuelBatTem)) ? pRealData->fuelBatTemCnt : 0); //ȼ�ϵ���¶�̽������
		bPackBuf[Idx++] = (uint8_t)(pRealData->fuelBatTemCnt/256);
		bPackBuf[Idx++] = (uint8_t)(pRealData->fuelBatTemCnt%256);
		
		for(i = 0; i < FuelBatTemCnt; i++)
		{
			sVal = pRealData->fuelBatTem[i];                              //ȼ�ϵ��̽���¶�ֵ
			if(sVal < 0xFE)
				sVal = sVal + 40;
			bPackBuf[Idx++] = (uint8_t)sVal;
		}
		
		usVal = pRealData->maxHydrSysTem;                               //��ϵͳ������¶�
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			sVal = (pRealData->maxHydrSysTem + 40) * 10;
		
		bPackBuf[Idx++] = (uint8_t)(sVal/256);
		bPackBuf[Idx++] = (uint8_t)(sVal%256);
		
		bPackBuf[Idx++] = pRealData->maxHydrSysTemIdx;                 //��ϵͳ������¶�̽�����
		
		usVal = pRealData->maxHydrThickness;                           //�������Ũ��
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		bPackBuf[Idx++] = pRealData->maxHydrThicknessIdx;              //�������Ũ�ȴ���������
		
		usVal = (uint16_t)(pRealData->maxHydrPressure);                //�������ѹ��
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->maxHydrPressure * 10);  //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		bPackBuf[Idx++] = pRealData->maxHydrPressureIdx;              //�������ѹ������������
		bPackBuf[Idx++] = pRealData->dc2dcState_highVol;              //��ѹDC/DC״̬
	}
	return Idx;
}

/*�����λ����*/
static uint16_t PacklocatorData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	uint32_t dwVal = 0;		
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x05;
		//��λ״̬
		bPackBuf[Idx++] = pRealData->locationState;						
		//����
		dwVal = (uint32_t)(pRealData->longd * 1000000);    
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		bPackBuf[Idx++] = (uint8_t)dwVal;
		//γ��
		dwVal = (uint32_t)(pRealData->latd * 1000000);  
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		bPackBuf[Idx++] = (uint8_t)dwVal;
	}
	return Idx;
}

/*�����ֵ����*/
static uint16_t PackExtremeVal(RealData *pRealData,uint8_t *bPackBuf,uint8_t packIdx)
{
	uint16_t Idx = 0,usVal = 0,VolIdx = 0;
	int16_t sVal = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x06;
		bPackBuf[Idx++] = pRealData->maxVolPack_index;                 	//��ߵ�ѹ�����ϵͳ��
		
		VolIdx = (pRealData->maxVol_index - 1) / 200;
		if(VolIdx == packIdx)
		{
			bPackBuf[Idx++] = (pRealData->maxVol_index - 1) % 200 + 1;			//��ߵ�ѹ��ص������
			usVal = (uint16_t)(pRealData->max_singleVol);                  	//��ص����ѹ���ֵ
			if(usVal < 0xFFFE)
					usVal = (uint16_t)(pRealData->max_singleVol * 1000);
			else
				usVal = 0xFFFE;
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		}
		else
		{
			if(packIdx < VolIdx)//��ֵ�ں�һ�ְ�
			{
				bPackBuf[Idx++] = 200;
			}
			else//��ֵ��ǰһ�ְ�
			{
				bPackBuf[Idx++] = 0;
			}
			bPackBuf[Idx++] = 0xFF;
			bPackBuf[Idx++] = 0xFF;			
		}
		
		
		bPackBuf[Idx++] = pRealData->minVolPack_index;                	//��͵�ѹ�����ϵͳ�� 
		VolIdx = (pRealData->minVol_index - 1) / 200;
		if(VolIdx == packIdx)
		{
			bPackBuf[Idx++] = (pRealData->minVol_index - 1) % 200 + 1;             
			usVal = (uint16_t)(pRealData->min_singleVol);                 //��ص����ѹ���ֵ
			if(usVal < 0xFFFE)
				usVal = (uint16_t)(pRealData->min_singleVol * 1000);
			else
				usVal = 0xFFFE;
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		}
		else
		{
			if(packIdx < VolIdx)//��ֵ�ں�һ�ְ�
				bPackBuf[Idx++] = 200;
			else//��ֵ��ǰһ�ְ�
				bPackBuf[Idx++] = 0;
			bPackBuf[Idx++] = 0xFF;
			bPackBuf[Idx++] = 0xFF;			
		}
		
		bPackBuf[Idx++] = pRealData->maxTemperPack_index;            //����¶���ϵͳ��
		bPackBuf[Idx++] = pRealData->maxTemper_index;                //����¶�̽�����
		sVal = pRealData->max_singleTemper;                          //����¶�ֵ
		if(sVal < 0xFE)
			sVal = sVal + 40;
		bPackBuf[Idx++] = (uint8_t)sVal;
		
		bPackBuf[Idx++] = pRealData->minTemperPack_index;            //����¶���ϵͳ��
		bPackBuf[Idx++] = pRealData->minTemper_index;                //����¶�̽�����
		sVal = pRealData->min_singleTemper;                          //����¶�ֵ
		if(sVal < 0xFE)
			sVal = sVal + 40;
		bPackBuf[Idx++] = (uint8_t)sVal;
	}
	return Idx;
}

/*�����������*/
static uint16_t PackAlertData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0, i = 0;
	uint8_t N1Cnt = 0, N2Cnt = 0, N3Cnt = 0, N4Cnt = 0, Tmp = 0;
	
	N1Cnt = ((pRealData->batFaultCnt <= sizeof(pRealData->batFault) / 4) ? pRealData->batFaultCnt : 0);
	N2Cnt = ((pRealData->motorFaultCnt <= sizeof(pRealData->motorFault) / 4) ? pRealData->motorFaultCnt : 0);
	N3Cnt = ((pRealData->engineFaultCnt <= sizeof(pRealData->engineFault) / 4) ? pRealData->engineFaultCnt : 0);
	N4Cnt = ((pRealData->otherFaultCnt <= sizeof(pRealData->otherFault) / 4) ? pRealData->otherFaultCnt : 0);
	
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x07;
		bPackBuf[Idx++] = pRealData->alarmLevel;                                   //�����ȼ�
		bPackBuf[Idx++] = 0;                                                      //Ԥ��
		Tmp = pRealData->batOverCharge > 0 ? 0x04 : 0x00;                          //���ش���װ�����͹���
		Tmp = Tmp | (pRealData->motorTempAlert > 0 ? 0x02 : 0x00);                 //��������¶ȱ���
		Tmp = Tmp | (pRealData->highPressInterlockStateAlert > 0 ? 0x01 : 0x00);   //��ѹ����״̬����
		bPackBuf[Idx++] = Tmp;
		
		Tmp = 0;
		Tmp = pRealData->motorCtrTemAlert > 0 ? 0x80 : 0x00;                 //��������������¶ȱ���
		Tmp = Tmp | (pRealData->dc2dcStateAlert > 0 ? 0x40 : 0x00);          //DC-DC״̬����
		Tmp = Tmp | (pRealData->brakingAlert > 0 ? 0x20 : 0x00);             //�ƶ�ϵͳ����
		Tmp = Tmp | (pRealData->dc2dcTemAlert > 0 ? 0x10 : 0x00);            //DC-DC�¶ȱ���
		Tmp = Tmp | (pRealData->insulationFailtAlert > 0 ? 0x08 : 0x00);     //��Ե����
		Tmp = Tmp | (pRealData->singleBatPoorConsisAlert > 0 ? 0x04 : 0x00); //��ص���һ���Բ��
		Tmp = Tmp | (pRealData->batNotMatchAlert > 0 ? 0x02 : 0x00);         //�ɳ�索��ϵͳ��ƥ�䱨��
		Tmp = Tmp | (pRealData->socHopAlert > 0 ? 0x01 : 0x00);              //SOC���䱨��
		bPackBuf[Idx++] = Tmp;
		
		Tmp = 0;
		Tmp = pRealData->socHighAlert > 0 ? 0x80 : 0x00;                    //SOC���߱���
		Tmp = Tmp | (pRealData->singleBattLowVolAlert > 0 ? 0x40 : 0x00);   //������Ƿѹ����
		Tmp = Tmp | (pRealData->singleBatHighVolAlert > 0 ? 0x20 : 0x00);   //�����ع�ѹ����
		Tmp = Tmp | (pRealData->socLowAlert > 0 ? 0x10 : 0x00);             //SOC�ͱ���
		Tmp = Tmp | (pRealData->batLowVolAlert > 0 ? 0x08 : 0x00);          //���ش���װ������Ƿѹ����
		Tmp = Tmp | (pRealData->batHighVolAlert > 0 ? 0x04 : 0x00);         //���ش���װ�����͹�ѹ����
		Tmp = Tmp | (pRealData->batHighTempAlert > 0 ? 0x02 : 0x00);        //��ظ��±���
		Tmp = Tmp | (pRealData->tempDiffAlert > 0 ? 0x01 : 0x00);           //�¶Ȳ��챨��
		bPackBuf[Idx++] = Tmp;
		
		bPackBuf[Idx++] = N1Cnt;//�ɳ�索��װ�ù�������N1
		for(i = 0; i < N1Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->batFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->batFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->batFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->batFault[i];
		}                

		bPackBuf[Idx++] = N2Cnt;//���������������N2
		for(i = 0; i < N2Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->motorFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->motorFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->motorFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->motorFault[i];
		}
		
		bPackBuf[Idx++] = N3Cnt;//��������������N3
		for(i = 0; i < N3Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->engineFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->engineFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->engineFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->engineFault[i];
		}
		
		bPackBuf[Idx++] = N4Cnt;//������������N4
		for(i = 0; i < N4Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->otherFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->otherFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->otherFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->otherFault[i];
		}
	}
	return Idx;
}

/*��������ѹ*/
static uint16_t PackMonomerBatterys(RealData *pRealData,uint8_t *bPackBuf,uint16_t RemainLen,uint8_t packIdx,uint8_t *pLastFlag)
{
	uint16_t Idx = 0, i = 0, j = 0;
	uint16_t usVal = 0, BatIdx = 0;
	uint8_t packVolCnt = 0,flag = 0;
	if(bPackBuf != 0 && pRealData->subBatSysCnt >= 1 && pRealData->subBatSysCnt <= 2 && pRealData->subSysData[0].singleVolCnt > 0 && (pRealData->subSysData[0].singleVolCnt + pRealData->subSysData[1].singleVolCnt) <= sizeof(pRealData->single_vol)/sizeof(pRealData->single_vol[0]))
	{
		if(RemainLen >= (2 + pRealData->subBatSysCnt * 10))
		{
			bPackBuf[Idx++] = 0x08;
			bPackBuf[Idx++] = pRealData->subBatSysCnt;
			for(i = 0; i < pRealData->subBatSysCnt; i++)
			{
				BatIdx = packIdx * 200;
				if(pRealData->subSysData[i].singleVolCnt - BatIdx > 200)
				{
					//�ֱ�δ��
					flag = 1;
					packVolCnt = 200;
				}
				else if(pRealData->subSysData[i].singleVolCnt - BatIdx > 0)
				{
					//�ְ����
					packVolCnt = pRealData->subSysData[i].singleVolCnt - BatIdx;
					flag = 0;
				}
				else
				{
					//�հ�
					packVolCnt = 0;
				}
				if(RemainLen > Idx + 10 + (packVolCnt * 2))
				{
					bPackBuf[Idx++] = pRealData->subSysData[i].subSysIdx;        //�ɳ�索����ϵͳ��
					usVal = (uint16_t)(pRealData->subSysData[i].subSysVol);      //�ɳ�索��װ�õ�ѹ
					if(usVal < 0xFFFE)
					{
						usVal = (uint16_t)(pRealData->subSysData[i].subSysVol * 10);
					}
					bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
					bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
					
					usVal = (uint16_t)(pRealData->subSysData[i].subSysCur);      //�ɳ�索��װ�õ���
					if(usVal < 0xFFFE)
					{
						usVal = (uint16_t)(pRealData->subSysData[i].subSysCur * 10 + 10000);
					}
					bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
					bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
					
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleVolCnt >> 8);//��ϵͳ����������
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleVolCnt >> 0);
					
					bPackBuf[Idx++] = (BatIdx + 1) >> 8;     									//��֡��ʼ������
					bPackBuf[Idx++] = (BatIdx + 1) >> 0;
					bPackBuf[Idx++] = packVolCnt;   													//��֡����������
					for(j = 0; j < packVolCnt && j < 200; j++)
					{
						BatIdx = pRealData->subSysData[i].singleVolStartIdx + j + (packIdx * 200);
						usVal = (uint16_t)(pRealData->single_vol[BatIdx]);			//�����ص�ѹ
						if(usVal < 0xFFFE && pRealData->single_vol[BatIdx] >= 0)
							usVal = (uint16_t)(pRealData->single_vol[BatIdx] * 1000);
						else
							usVal = 0xFFFE;
						bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
						bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
					}
				}
				else
				{
					Idx = 0;
					break;
				}
			}
		}
	}
	if(flag == 0)
	{
		*pLastFlag = 1;
	}
	return Idx;
}

/*����¶�����*/
static uint16_t PackMonomerTemperatures(RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t Idx = 0, TempIdx = 0;
  uint16_t i = 0, j = 0;
	int16_t sVal = 0;
	
	if(bPackBuf != 0)
	{
		if(pRealData->subBatSysCnt >= 1 && pRealData->subBatSysCnt <= 2 && pRealData->subSysData[0].singleTemCnt > 0 && (pRealData->subSysData[0].singleTemCnt + pRealData->subSysData[1].singleTemCnt) <= sizeof(pRealData->single_temper)/sizeof(pRealData->single_temper[0]))
		{
			if(RemainLen >= (2 + pRealData->subBatSysCnt * 3 + pRealData->subSysData[0].singleTemCnt + pRealData->subSysData[1].singleTemCnt))
			{
				bPackBuf[0] = 0x09;
				bPackBuf[1] = pRealData->subBatSysCnt;//��ϵͳ����
				Idx = 2;
				for(i = 0; i < pRealData->subBatSysCnt; i++)
				{
					bPackBuf[Idx++] = pRealData->subSysData[i].subSysIdx;//��ϵͳ���
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleTemCnt >> 8);//�¶�̽�����
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleTemCnt >> 0);
						
					for(j = 0; j < pRealData->subSysData[i].singleTemCnt; j++)
					{
						TempIdx = pRealData->subSysData[i].singleTemStartIdx + j;
						sVal = pRealData->single_temper[TempIdx];//̽���¶�ֵ
						if(sVal < 0xFE)
							sVal = sVal + 40;
						bPackBuf[Idx++] = (uint8_t)sVal;
					}
				}
			}
		}
	}
	return Idx;
}

/*
*  ����˵��: ���ʵʱ�������ڴ洢
*  ����wbuf: ���뻺����, byte0:���ݵ�Ԫ��Ԫ���ȸ��ֽ�,byte1:���ݵ�Ԫ���ȵ��ֽ�
*  �� �� ֵ: �������
*/
uint16_t buildRealData(GBSTA *pSta,uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,RealData *pRealData)
{
	uint16_t index,subLen,totalLen = 0;
	uint8_t i,lastFlag = 0,*pbuf;
	for(i = 0;i<3;i++)
	{
		pbuf = &buff[totalLen];
		index = 24;
		pbuf[index++] = (uint8_t)(pRealData->year - 2000);
		pbuf[index++] = (uint8_t)(pRealData->month);
		pbuf[index++] = (uint8_t)(pRealData->day);
		pbuf[index++] = (uint8_t)(pRealData->hour);
		pbuf[index++] = (uint8_t)(pRealData->minute);
		pbuf[index++] = (uint8_t)(pRealData->second);
		if(i == 0)
		{
			//��������
			index += PackVehicleData(pRealData,&pbuf[index]);
			//�����������
			index += PackMotorData(pRealData,&pbuf[index]);
			//����������
			index += PackEngineData(pRealData,&pbuf[index]);
			//ȼ�ϵ������
			index += PackFuelBatData(pRealData,&pbuf[index],maxLen - totalLen - index);
			//��λ����
			index += PacklocatorData(pRealData,&pbuf[index]);
		}
		//��ֵ���ݣ��൥����Ҫ�ְ�
		index += PackExtremeVal(pRealData,&pbuf[index],i);
		if(i == 0)
		{
			//��������
			index += PackAlertData(pRealData,&pbuf[index]);
		}
		//�����ѹ,�൥����Ҫ�ְ�
		index += PackMonomerBatterys(pRealData,&pbuf[index],maxLen - totalLen - index,i,&lastFlag);
		if(i == 0)
		{
			//̽���¶�
			index += PackMonomerTemperatures(pRealData,&pbuf[index],maxLen - totalLen - index);					
		}
		if(lastFlag)
		{
			//������չЭ��
			index += extReal(pSta->extCfg,pRealData,&pbuf[index],maxLen - totalLen - index);	
		}
        if(SHDB_STA)
        {
//            index += extHyFuelCellReal(pSta->extCfg,pRealData,&pbuf[index],maxLen - totalLen - index);
//            index += extSHDBReal(pSta->extCfg,pRealData,&pbuf[index],maxLen - totalLen - index);
        }
		subLen = MakeCmd(cmd,0xFE,vin,pbuf,index - 24);
		totalLen += subLen;
		//�ж��Ƿ����һ��
		if(lastFlag)
		{
			break;
		}
	}
	return totalLen;
}

static uint8_t uppackQueryParas(GBSTA* pSta,const uint8_t *rbuf,uint16_t rLen)
{
	uint8_t RspParasBuf[150] = {0};
	uint8_t index = 0;
	uint8_t ParasPackIndex = 31;
	uint8_t QueryParaCnt,ResPondParasCnt = 0,i;
	QueryParaCnt = rbuf[30];
	memcpy(&RspParasBuf[24],&rbuf[24],6);
	for(i = 0; i < QueryParaCnt;i++)
	{
		uint8_t ParaId = rbuf[31+i];
		switch(ParaId)
		{
			case 0x01: //�ն˱��ش洢ʱ������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x01;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.localSaveInterval / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.localSaveInterval % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x02: //����ʱ����Ϣ�ϱ�ʱ������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x02;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.realDataInterval / 256);
        RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.realDataInterval % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x03: //����ʱ����Ϣ�ϱ�ʱ������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x03;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.warnRealDataInterval / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.warnRealDataInterval % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x04: //Զ�̷��������ƽ̨��������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x04;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)strlen(gSysPara.domain[pSta->bLink]);
				ResPondParasCnt++;
			}
			break;
			case 0x05: //Զ�̷��������ƽ̨����
			{
				uint8_t UrlLen = strlen(gSysPara.domain[pSta->bLink]);
				RspParasBuf[ParasPackIndex + index++] = 0x05;
				if(UrlLen > 0 && UrlLen <= MAX_URL_LEN)
				{
					memcpy(&RspParasBuf[ParasPackIndex + index],gSysPara.domain[pSta->bLink],UrlLen);
					index += UrlLen;					
				}
				ResPondParasCnt++;
			}
			break;
			case 0x06: //�ۺ�ƽ̨�˿�
			{
				RspParasBuf[ParasPackIndex + index++] = 0x06;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[pSta->bLink] / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[pSta->bLink] % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x07: //Ӳ���汾
			{
				RspParasBuf[ParasPackIndex + index++] = 0x07;
				memcpy(&RspParasBuf[ParasPackIndex + index],HW_VERSION,5);
				index += 5;
				ResPondParasCnt++;
			}
			break;
			case 0x08: //�̼��汾
			{
				RspParasBuf[ParasPackIndex + index++] = 0x08;
				memcpy(&RspParasBuf[ParasPackIndex + index],SW_VERSION,5);
				index += 5;
				ResPondParasCnt++;
			}
			break;
			case 0x09: //�ն���������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x09;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.heartInterval);
				ResPondParasCnt++;
			}
			break;
			case 0x0A: //�ն�Ӧ��ʱʱ��
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0a;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.terminalRespTimeOut / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.terminalRespTimeOut % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x0B: //�ۺ�ƽ̨Ӧ��ʱʱ��
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0b;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.platformRespTimeOut / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.platformRespTimeOut % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x0C: //��һ�ֵ���ʱ����
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0c;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.nextLoginInterval);
				ResPondParasCnt++;
			}
			break;
			case 0x0D: //����ƽ̨��������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0d;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)strlen(gSysPara.domain[EV2_SERVER]);
				ResPondParasCnt++;
			}
			break;
			case 0x0E: //����ƽ̨����
			{
				int UrlLen = strlen(gSysPara.domain[EV2_SERVER]);
				RspParasBuf[ParasPackIndex + index++] = 0x0e;
				if(UrlLen > 0 && UrlLen <= MAX_URL_LEN)
				{
					memcpy(&RspParasBuf[ParasPackIndex + index],gSysPara.domain[EV2_SERVER],UrlLen);
					index += UrlLen;
				}
				ResPondParasCnt++;
			}
			break;
			case 0x0F: //����ƽ̨�˿�
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0f;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[EV2_SERVER] / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[EV2_SERVER] % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x10: //�����������
			{
				RspParasBuf[ParasPackIndex + index++] = 0x10;
				RspParasBuf[ParasPackIndex + index++] = (gSysPara.linkSwitch & (1 << EV2_SERVER)) ? 1 : 0;
				ResPondParasCnt++;
			}
			break;
		}
	}
	RspParasBuf[30] = ResPondParasCnt;
	index++;
	if(ResPondParasCnt >0)
	{
		uint8_t datalen = MakeCmd(CMD_QUERYPARAS,0x01,pSta->vin,RspParasBuf,index);
		Fun_Gprs_Tcp_send(pSta->bLink,RspParasBuf,datalen);
	}
	return 0;
}

static uint8_t uppackSetParas(GBSTA* pSta,const uint8_t *rbuf,uint16_t rLen)
{
	uint16_t ResponSetBufLen = 0;
	uint8_t RspParasBuf[150] = {0};
	uint16_t index;
	uint8_t i,j,paraCnt,packParaCnt;
	uint8_t *pData = (uint8_t*)&rbuf[24];
	uint8_t ServerUrlLen = 0, PublicUrlLen = 0;
	for(j = 0;j < 2;j++)//��һ��У�����ݣ��ڶ�������
	{
		//��������ʱ��
		index = 6;
		paraCnt = pData[index++];
		packParaCnt = 0;
		for(i = 0;i< paraCnt;i++)
		{
			packParaCnt++;
			switch(pData[index++])
			{
				case 0x01:
				{
					if(j == 1)//���ش洢ʱ������
					{
						gSysPara.localSaveInterval = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x02:
				{
					if(j == 1) //����ʱ����Ϣ�ϱ�ʱ������
					{
						gSysPara.realDataInterval = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x03://����ʱ����Ϣ�ϱ�ʱ������
				{
					if(j == 1)
					{
						gSysPara.warnRealDataInterval = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x04://Զ�̷��������ƽ̨��������
				{
					if(j == 1)
					{
						ServerUrlLen = pData[index];
					}
					index += 1;
				}
				break;
				case 0x05://Զ�̷��������ƽ̨����
				{
					if(j == 1 && ServerUrlLen > 0)
					{
						memset(gSysPara.domain[pSta->bLink],0,sizeof(gSysPara.domain[0]));
						strncpy(gSysPara.domain[pSta->bLink],(char*)&pData[index],ServerUrlLen);								
					}
					index += ServerUrlLen;
				}
				break;
				case 0x06://Զ�˷������˿ں�
				{
					if(j == 1)
					{
						gSysPara.port[pSta->bLink] = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x07://Ӳ���汾
				{
					index += 5;
				}
				break;
				case 0x08://�̼��汾
				{
					index += 5;
				}
				break;
				case 0x09://������������:
				{
					if(j == 1)
					{
						gSysPara.heartInterval = pData[index];
					}
					index += 1;	
				}
				break;
				case 0x0A://�ն�Ӧ��ʱʱ��
				{
					if(j == 1)
					{
						gSysPara.terminalRespTimeOut = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x0B://ƽ̨Ӧ��ʱʱ��
				{
					if(j == 1)
					{
						gSysPara.platformRespTimeOut = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x0C://��һ�ֵ�������
				{
					if(j == 1)
					{
						gSysPara.nextLoginInterval = pData[index];
					}
					index += 1;	
				}
				break;
				case 0x0D://����ƽ̨��������
				{
					if(j == 1)
					{
						PublicUrlLen = pData[index];
					}
					index += 1;
				}
				break;
				case 0x0E://����ƽ̨����
				{
					if(j == 1 && ServerUrlLen > 0)
					{
						memset(gSysPara.domain[EV2_SERVER],0,sizeof(gSysPara.domain[0]));
						strncpy(gSysPara.domain[EV2_SERVER],(char*)&pData[index],PublicUrlLen);								
					}
					index += ServerUrlLen;
				}
				break;
				case 0x0F://����ƽ̨�˿ں�
				{
					if(j == 1)
					{
						gSysPara.port[EV2_SERVER] = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x10://�������
				{
					if(j == 1)
					{
						gSysPara.linkSwitch &= ~ (1 << EV2_SERVER);
						gSysPara.linkSwitch |= (pData[index] > 0) << EV2_SERVER;
					}
					index += 1;
				}
				break;
				default:packParaCnt = 0;
					break;
			}
		}
		if(j == 0 && packParaCnt != paraCnt)//��������
		{
			break;
		}
	}
	if(packParaCnt == paraCnt)
	{
		//gUpdateParaFlag = 0x1F;
		System_Pare_Save();
	}
	memcpy(&RspParasBuf[24],&rbuf[24],6);
	ResponSetBufLen = 6;
	ResponSetBufLen = MakeCmd(CMD_SETPARAS,packParaCnt == paraCnt ? 0x01:0x02,pSta->vin,RspParasBuf,ResponSetBufLen);
	Fun_Gprs_Tcp_send(pSta->bLink,RspParasBuf,ResponSetBufLen);
	return 0;	
}

static uint8_t uppackCtrl(GBSTA* pSta,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[150];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	uint8_t isReconnet = 0,isReStart = 0,isFtpUpdate = 0,isPowerOff = 0;
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	if(rbuf[30] >= 0x01 && rbuf[30] <= 0x07)
	{
		switch(rbuf[30])
		{
			case 0x01: //Զ������
			{
//				char *pSplit = NULL;
				
//				if(strncmp((char*)&rbuf[31],"FTP://",6) == 0 || strncmp((char*)&rbuf[31],"ftp://",6)==0)
//				{
//					if(sscanf((char*)&rbuf[37],"%[^:]:%[^@]@%[^/;]/%[^;]",gFtpParas.ftpUser,gFtpParas.ftpPwd,gFtpParas.ftpUrl,gFtpParas.ftpDir) >= 3)
//					{
//						//���·�����ļ���
//						if((pSplit = strrchr(gFtpParas.ftpDir,'/')) != NULL){
//							pSplit++;
//						}else{
//							pSplit = gFtpParas.ftpDir;
//						}
//						strcpy(gFtpParas.ftpName,pSplit);
//						*pSplit = '\0';
//						//��ֵ�ַ�Ͷ˿�
//						if((pSplit = strstr(gFtpParas.ftpUrl,":")) != NULL){
//							sscanf(pSplit,":%hu",&gFtpParas.ftpPort);
//							*pSplit = '\0';
//						}else{
//							gFtpParas.ftpPort = 21;
//						}
//						replyFlag = 1;
//						isFtpUpdate = 1;
//					}
//				}
				
				char *hostname;
				uint16_t hostport;
				char *username = NULL;
				char *password = NULL;
				char *filePach = NULL;
				char* fileName = NULL;
				char *pSplit = NULL;
				//�û���
				username = strstr((char*)&rbuf[31],"FTP://");
				if(username == NULL)
				{
					username = strstr((char*)&rbuf[31],"ftp://");
				}
				if(username != NULL)
				{
					username += 6;
					//����
					password = strstr(username,":");
				}
				//URL
				if(password != NULL)
				{
					*password = 0;
					password++;
					hostname = strstr(password,"@");
				}
				//·��
				if(hostname != NULL)
				{
					*hostname = 0;
					hostname++;
					filePach = strstr(hostname,"/");
					//·���ָ����ݾ�Э��
					if(filePach == NULL)
					{
						filePach = strstr(hostname,";");
					}
					//ȡ���˿ں�
					hostport = 21;
					pSplit = strstr(hostname,":");
					if(pSplit != NULL)
					{
						*pSplit = 0;
						pSplit++;
						sscanf(pSplit,"%hu",&hostport);
					}
				}
				if(filePach != NULL)
				{
					*filePach = 0;
					filePach++;
					//���·�����ļ���
					fileName = "NEVT200-F4_Ftp.bin";
					if((pSplit = strstr(filePach,";")) != NULL)
					{
						*pSplit = 0;
					}
					if((pSplit = strrchr(filePach,'/')) != NULL)
					{
						*pSplit = 0;
						pSplit++;
						if(*pSplit != 0)
						{
							fileName = pSplit;
						}
					}
					else
					{
						fileName = filePach;
						filePach = NULL;
					}
					
				}
				replyFlag = 2;

				//��ʼ����			
				funMonSetFtpParas(hostname,hostport,username,password,filePach,fileName,NULL);
			}
			break;
			case 0x02: //�����ն˹ػ�
			{
				isPowerOff = 1;
			}
			break;
			case 0x03: //�����ն˸�λ
			{
				isReStart = 1;
			}
			break;
			case 0x04: //�ָ���������
			{
				System_Pare_Restore();
				isReconnet = 1;
			}
			break;
			case 0x05: //�Ͽ�����ͨ����·
			{
				//�ذ�
				isReconnet = 1;
			}
			break;
			case 0x06: //�����ն˱���
			{
//				gRealData.alarmLevel = rbuf[31];
//				if(gRealData.alarmLevel == 3)
//				{
//					gTerminalState.ctrlAlarmSta = 1;
//					gTerminalState.ctrlAlarmStaTimeStamp = osKernelGetTickCount();	
//				}
			}
			break;
			case 0x07: //�������
			{
				gSysPara.linkSwitch |= 1 << EV2_SERVER;
				//���������·
			}
			break;
		}
		DataAreaLen = MakeCmd(CMD_CTRL,replyFlag,pSta->vin,ResponCtrlBuf,DataAreaLen);
		sendNetData(pSta->bLink,ResponCtrlBuf,DataAreaLen);
		osDelay(50);
		if(isReconnet)
		{//����
			Fun_Gprs_Tcp_disconnect(pSta->bLink);;
		}
//	if(isFtpUpdate)
//	{//Զ������
//		gIsPendingUpdate = 0x55;
//	}
		if(isReStart)
		{//����
			BoardReset();
		}
		if(isPowerOff == 1)
		{//�ػ�
			osDelay(500);
		}
	}
	return 0;
}

static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i,exSta;
	char vin[17];
	GBSTA* pSta = NULL;
//	if(printPackCmd == 1)
//	{
//		comSendBuf(RS1_COM,szRecvBuf,rLen);
//	}
	for(i = 0;i < MAX_GB_LINK;i++)
	{
		if(gbSta[i].bLink == link && gbSta[i].bUse == 1)
		{
			pSta = &gbSta[i];
			break;
		}
	}
	if(pSta == NULL)//δ���ô���·
		return;
	else if(rLen < 25 )//���ݳ��ȴ���
  {
		return;
	}
	else if(szRecvBuf[0] != 0x23 || szRecvBuf[1] != 0x23)//��־����
	{
		return;
	}
	else if(getBccCode(szRecvBuf, 2, rLen - 3) != szRecvBuf[rLen - 1])//BCCУ��
	{
		return;
	}
//	if(pSta->extCfg != NULL)
//	{
//		//��չЭ�������жϳ��ܺ�
		exSta = extUnpack(link,szRecvBuf,rLen);
//		if(exSta == 1)
//			return;
//	}
//	memcpy(vin,pSta->vin,17);
//	extGetVin(szRecvBuf[2],szRecvBuf[3],vin,(uint8_t*)&szRecvBuf[4]);
//	if(memcmp(vin,&szRecvBuf[4],17) != 0)//���ܺŴ���
//	{
//		return;
//	}
	
	//���մ洢ƽ̨��Ӧ����
	saveHistory(link,1,(uint8_t*)szRecvBuf,rLen,0,LOG);
	
	switch(szRecvBuf[2])
	{
		case CMD_LOGIN:  //��Ӧ��������
		{
			if(szRecvBuf[3] == 1 && pSta->bLogined == 0xFF)
			{
				pSta->sendOverTimeCnt = 0;
				pSta->bLogined = 1;
			}
		}
		break;
		case CMD_REALDATA:  		//��Ӧʵʱ����
		case CMD_REISSUEDATA:   //��Ӧ��������
		case CMD_HEARTBEAT:  		//��Ӧ����
		{
			if(szRecvBuf[3] == 1)
				pSta->sendOverTimeCnt = 0;
		}
		break;
		case CMD_TIMING:  //��ӦУʱ
		{
			if(szRecvBuf[3] == 1)
			{
				pSta->sendOverTimeCnt = 0;
				if(rLen == 31 && pSta->bTiming > 0 && pSta->bTiming < 4)
				{
					//������ʱ�ڷ�Χ��
					RTC_INFOR sysTime;
					sysTime.year = 2000+szRecvBuf[24];
					sysTime.month = szRecvBuf[25];
					sysTime.day = szRecvBuf[26];
					sysTime.hour = szRecvBuf[27];
					sysTime.minute = szRecvBuf[28];
					sysTime.second = szRecvBuf[29];
					RTC_Time_Set(&sysTime);	
				}
				pSta->bTiming = 0xFF;				
			}
		}
		break;
		case CMD_LOGOUT:  //��Ӧ�ǳ�
		{
			if(szRecvBuf[3] == 1 && pSta->bLogouted != 0)
				pSta->bLogouted = 1;
		}
		break;
		case 	CMD_QUERYPARAS:
		{
			uppackQueryParas(pSta,szRecvBuf,rLen);
		}
		break;
		case CMD_SETPARAS:
		{
			uppackSetParas(pSta,szRecvBuf,rLen);
		}
		break;
		case CMD_CTRL:
		{
			uppackCtrl(pSta,szRecvBuf,rLen);
		}
		break;
	}
	return;
}



__WEAK void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	return NULL;
}

__WEAK uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	return 1;
}
__WEAK uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	return 0;
}

__WEAK uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	return 0;
}

__WEAK uint8_t extCtrlNetSta(void* obj)
{
	return 0;
}

__WEAK uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
 return 0;
}
