#include "bsp_sys.h"
#include "protocol_GB17691.h"
#include "bsp_storage.h"
#include "bsp_rtc.h"
#include "string.h"
#include "algo_verify.h"
#include "Fun_Net.h"
#include "se.h"
#include "types.h"
#include "fun_can.h"

#define MAX_GB_LINK 2

enum COMMAND_CODE
{
	CMD_NONE = 0,
	CMD_LOGIN,                 			//��������
	CMD_REALDATA,              			//ʵʱ����
	CMD_REISSUEDATA,           			//��������
	CMD_LOGOUT,                			//�����ǳ�
	CMD_TIMING,               			//Уʱ
	CMD_ANTITAMPER,						 			//����
	CMD_SETUP = 0x07,					 			//����
	CMD_RSP_SETUP = 0x08,			 			//����Ӧ��
	CMD_PARA_QUERY = 0x80,					//��������
};

//���籣����·����
typedef struct _CFG
{
	uint32_t regTimeStamp;					//ע��ʱ�����־
	uint32_t realTimeStamp;					//ʵʱ��Ϣʱ�����־
	uint32_t antiTimeStamp;					//������Ϣʱ�����־
	uint16_t regNo;									//ע����ˮ��
	uint16_t realNo;								//ʵʱ��ˮ��
	uint16_t antiNo;								//������ˮ��
}CFG;

typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;										//�Ƿ�ʹ��
	char *vin;											//���ܺ�
	char* domain;										//��·����
	uint32_t* port;									//��·�˿ں�
	uint8_t proc;										//ȼ�ͳ�ƽ̨Э������ 0:17691 1:��������ǩ������ 2:����������ǩ�� 3:���������ݼ���
	//ƽ̨����״̬����
	uint8_t bLogined;       				//�����־λ 0:δ���� 1:�ѵ��� 0xFF:���͵�����
	uint8_t bLogouted;							//�ǳ���־λ 0:δ�ǳ� 1:�ѵǳ�
	uint8_t bTiming;								//Уʱ��־
	uint8_t sendOverTimeCnt;				//���ͳ�ʱ����
	uint8_t sendActiCnt;						//���ͼ������
	uint8_t reissueSta;							//����״̬��0����Ҫ���� 1���������
	//ʵʱ��ʱ����
	uint8_t realSec;								//ʵʱ����
	uint8_t realSecCnt;							//ʵʱ�����
	uint8_t sendObdFlag;						//����OBD���ݱ�־ 0:������ 1��������һ�� 2��������2�� ������������
	//�ڲ�ʱ���
	uint32_t antiStamp;							//����ʱ���
	uint32_t reissueStamp;					//����ʱ���
	uint32_t heartbeatStamp;				//����ʱ���
	uint32_t actitStamp;						//����ʱ���
	//��������
	uint8_t* buff;									//�������ݻ�����
	uint16_t buffSize;							//�������ݻ�������С
	//�����ļ���
	char reissueFileName[30];				//�����ļ���
	uint8_t day;										//��
	//�ڲ����籣�����
	CFG cfg;
}GBSTA;

static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc);			//��������
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc);										//�ն�Уʱ
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc);		//�����ǳ�
static uint16_t packAntiTamper(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc);	//�ն˷���
static uint16_t packSetUp(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc);										//�ն˱�������
static uint16_t buildRealData(uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc,uint8_t obdFlag);	//���ʵʱ����
static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen);																		//�������

static GBSTA gbSta[MAX_GB_LINK] = {0};

/*��ʼ������*/
void* gb17691Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t proc)
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
	//�ⲿ����
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].domain = domain;
	gbSta[objLinkIdx].port = port;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].proc = proc;
	//�ڲ�
	gbSta[objLinkIdx].bLogined = 0;
	gbSta[objLinkIdx].bLogouted = 0;
	gbSta[objLinkIdx].bTiming = 0;
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].sendActiCnt = 0;
	gbSta[objLinkIdx].reissueSta = 0;
	
	gbSta[objLinkIdx].realSec = 0xFF;
	gbSta[objLinkIdx].realSecCnt = 0;
	gbSta[objLinkIdx].antiStamp = 0;
	gbSta[objLinkIdx].reissueStamp = 0;
	gbSta[objLinkIdx].heartbeatStamp = 0;
	gbSta[objLinkIdx].actitStamp = 0;
	gbSta[objLinkIdx].buff = pBuff;
	gbSta[objLinkIdx].buffSize = pBuffSize;
	gbSta[objLinkIdx].day = g_system_dt.day;
	//����洢�����ָ�
	readConfig(link,&gbSta[objLinkIdx].cfg,sizeof(gbSta[objLinkIdx].cfg),0);
	//ͬ����־�ļ�
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),LOG);
	//ͬ�������ļ�
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),REC);
	//���ص�ǰ��·����״ָ̬��
	return &gbSta[objLinkIdx];
}

/*��������·״̬���� ���� 0:���Ե�¼ƽ̨ 1:�ǳ�ƽ̨*/
uint8_t gb17691Run(void* obj)
{
	static uint8_t ctrl = 0;
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
	if(Fun_Gprs_Tcp_Sta(pSta->bLink) == FUN_GPRS_CONNECTED)
	{
		svrSta = 1;
	}
	//if(fun_can_Get_State(BSP_CAN) > 0 && (gRealData.carState == 1 || gRealData.chargeState == 1))
	//if(1)
	if(fun_can_Get_State(BSP_CAN) > 0 && ((gRealData.engineSpeed <= 10 || gRealData.engineSpeed >= 0xFFFE) && (gRealData.speed == 0 || gRealData.speed >= 0xFFFE)) == 0)
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
			ctrl = 1;
		}
	}
	else
	{
		endTicks = ticks;
		if(gRealData.chargeState == 4)
		{
			if(ticks - startTicks >= 360000)
			{
				ctrl = 0;
			}
		}
		else
		{
			if(ticks - startTicks >= 10000)
			{
				ctrl = 0;
			}			
		}
		if(ctrl == 0 && svrSta == 1 && pSta->bLogined != 1)
		{
			Fun_Gprs_Tcp_disconnect(pSta->bLink);
		}
	}
	if(pSta->realSec != gRealData.second)
	{
		pSta->bTiming = (pSta->bTiming >= 1 && pSta->bTiming < 0xFF) ? pSta->bTiming + 1: pSta->bTiming;
		pSta->realSec = gRealData.second;
		//���ڿ���
		if(++pSta->realSecCnt >= 10)
		{
			uint32_t realTimeStamp = g_system_dt.year << 16 | g_system_dt.month << 8 | g_system_dt.day << 0;
			//ʵʱ������ˮ��
			if(pSta->cfg.realTimeStamp != realTimeStamp)
			{
				pSta->cfg.realTimeStamp = realTimeStamp;
				pSta->cfg.realNo = 0;
			}
			pSta->cfg.realNo++;
			saveConfig(pSta->bLink,&pSta->cfg,sizeof(pSta->cfg),0);
			//����OBD������24Сʱ��������һ�Σ���ǰ��ȡ������2��
			//gTerminalState.obdState = 1;
			pSta->sendObdFlag = pSta->sendObdFlag < 3 ? pSta->sendObdFlag + 1:pSta->sendObdFlag;
			pSta->sendObdFlag = gTerminalState.obdState == 0 ? 0 : pSta->sendObdFlag;
			pSta->realSecCnt = 0;
			dataLen =	buildRealData(pSta->bLogined == 0 ? CMD_REISSUEDATA : CMD_REALDATA,pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.realNo,pSta->proc,pSta->sendObdFlag >= 1 && pSta->sendObdFlag <= 2);
			if(dataLen > 0)
			{
				if(pSta->bLogined == 0)
				{
					//�洢��������
					saveHistory(pSta->bLink,pSta->bLogined,pSta->buff,dataLen,0,REC);
				}
			}
		}
	}
	//ƽ̨����
	if(svrSta == 1 && (ctrl == 1 || (pSta->bLogined == 1 && pSta->bLogouted == 0)))//�������������������ն��Ѿ�������δ�ǳ�
	{
		active = 1;
		if(pSta->sendOverTimeCnt >= 3 &&  ticks - pSta->heartbeatStamp >= 3000)
		{
			pSta->sendOverTimeCnt = 0;
			//����3�γ�3����Ӧ��ʱ,�ز�
			Fun_Gprs_Tcp_disconnect(pSta->bLink);
		}
		else if(pSta->bLogined == 1)
		{
			//�����������������
			if(dataLen > 0)//����ʵʱ����
			{
				pSta->heartbeatStamp = ticks;
				pSta->sendOverTimeCnt++;
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
				saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
			}
			else if(pSta->bTiming == 0)//Уʱ
			{
				pSta->bTiming = 1;
				if((dataLen = packTiming(pSta->vin,pSta->buff,pSta->buffSize,pSta->proc)) > 0)
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			}
			else if(pSta->reissueFileName[0] != 0 && ticks - pSta->reissueStamp >= 500)//��������
			{
				pSta->reissueStamp = ticks;
				dataLen = readHistory(pSta->bLink,pSta->reissueFileName,pSta->buff,pSta->buffSize,REC);
				if(dataLen > 0)
				{
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
					saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
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
			else if(pSta->proc != 0 && gTerminalState.powerState == 0 && ticks - pSta->antiStamp >= 60000)//����
			{
				//��ˮ��
				if(pSta->cfg.antiTimeStamp != g_system_dt.year)
				{
					pSta->cfg.antiTimeStamp = g_system_dt.year;
					pSta->cfg.antiNo = 0;
				}
				pSta->cfg.antiNo++;
				saveConfig(pSta->bLink,&pSta->cfg,sizeof(pSta->cfg),0);
				pSta->antiStamp = ticks;
				if((dataLen = packAntiTamper(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.antiNo,pSta->proc)) > 0)
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			}
			else if(ctrl == 0 && pSta->bLogouted == 0)//�ǳ�
			{
				pSta->bLogouted = 1;
				if((dataLen = packLogout(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo,pSta->proc)) > 0)
				{
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
					saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
				}
			}
		}
		else if(ticks - pSta->heartbeatStamp >= 20000 && pSta->bLogined != 1)//����
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
			if((dataLen = packLogin(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo,pSta->proc)) > 0)
			{
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
				saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
			}
			pSta->sendOverTimeCnt++;
		}
		/*17691����Ӧ��*/
		if(pSta->proc == 0)
		{
			pSta->sendOverTimeCnt = 0;
			if(pSta->bLogined == 0xFF)
			{
				pSta->bLogined = 1;
			}
		}
	}
	//���߲������
	if(svrSta == 0 || pSta->bLogined == 0  || pSta->bLogouted == 1)
	{
		//�޷���������·δʹ������ȫֹͣ����ر�������
		pSta->reissueSta = 0;
		pSta->bLogined = 0;
		pSta->bLogouted = 0;
		pSta->bTiming = 0;
		pSta->sendActiCnt = 0;
		pSta->heartbeatStamp = 0;
	}
	//����ļ����
	if(pSta->proc == 2 && gTerminalState.obdState == 1 && CheckVin(gRealData.vin) == 1 && gFrimPara.setUpFlag != 0xAA)
	{
		const uint8_t setUpLink = 2;
		if(Fun_Gprs_Tcp_Sta(setUpLink) == FUN_GPRS_CONNECTED)
		{
			if(ticks - pSta->actitStamp >= 120000 && pSta->sendActiCnt < 3)
			{
				pSta->actitStamp = ticks;
				pSta->sendActiCnt++;
				if((dataLen = packSetUp(pSta->vin,pSta->buff,pSta->buffSize,pSta->proc)) > 0)
					Fun_Gprs_Tcp_send(setUpLink,pSta->buff,dataLen);		
			}
		}
		else
		{
			Fun_Gprs_Tcp_connect(setUpLink,"g6check.vecc.org.cn",19006);
			Fun_Gprs_Tcp_Set_RecvCallBack(setUpLink,unpack);
		}
	}
	return active;	
}

/*��ȡ��·״̬*/
uint8_t gb17691GetSta(uint8_t id)
{
	if(id < MAX_GB_LINK)
		return (gbSta[id].bLogined == 1);
	return 0;
}

//������ݱ�ͷ��У��
static uint16_t MakeCmd(uint8_t cmd,char * vin,uint8_t *wbuf,uint16_t dataLen,uint8_t proc)
{
	int rv = SUCCEED;
	uint32_t sm2Len = dataLen + 96;
		
//	if(cmd != CMD_SETUP)
//	{
		wbuf[0] = 0x23;
		wbuf[1] = 0x23;
		wbuf[2] = cmd;																										//���Ԫ
		memcpy(&wbuf[3],gSysPara.vinCode,17);         										//���ܺ�	
		wbuf[20] = SW_VERSION_NUM;																				//�ն�����汾��
//	}
//	else
//	{
//		wbuf[0] = 0x7E;
//		wbuf[1] = 0x7E;
//		wbuf[2] = cmd;																										//���Ԫ
//		wbuf[3] = 0xFE;																										//Ӧ���־
//		memcpy(&wbuf[4],gSysPara.vinCode,17);         										//���ܺ�	
//	}
	wbuf[21] = 0x01;//���ݼ��ܷ�ʽ
	if(proc == 3)
	{
		wbuf[21] = 0x03;//SM2����
		if(dataLen > 0)
		{
			if(cmd != CMD_SETUP)
			{
//				rv = tms_sm2_encrypt(&wbuf[24],dataLen,&wbuf[24],&sm2Len,0x0002);
			}
			else
			{
//				rv = tms_sm2_encrypt(&wbuf[24],dataLen,&wbuf[24],&sm2Len,0x0003);//��������ƽ̨��Կ
			}
			if(rv == SUCCEED)
			{
				dataLen += 96;
			}
			else
			{
				//����ʧ��
				return 0;
			}
		}
	}
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

//�����������
static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc)
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
	return MakeCmd(CMD_LOGIN,vin,buff,DataAreLen,proc);
}

//���Уʱ����
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc)
{
	return MakeCmd(CMD_TIMING,vin,buff,0,proc);	
}

//����ǳ�����
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc)
{
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	buff[30] = (regNo) >> 8;
	buff[31] = (regNo);
	return MakeCmd(CMD_LOGOUT,vin,buff,8,proc);
}

//�����������
static uint16_t packAntiTamper(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc)
{
	uint8_t index = 24;
	uint32_t n32Val;
	buff[index++] = (uint8_t)(g_system_dt.year - 2000);
	buff[index++] = (uint8_t)(g_system_dt.month);
	buff[index++] = (uint8_t)(g_system_dt.day);
	buff[index++] = (uint8_t)(g_system_dt.hour);
	buff[index++] = (uint8_t)(g_system_dt.minute);
	buff[index++] = (uint8_t)(g_system_dt.second);	
	buff[index++] = msgNo >> 8;         															//ע����ˮ��H
	buff[index++] = msgNo;
	buff[index++] = gRealData.locationState;													//��λ״̬
	n32Val = gRealData.longd * 1000000;																//����ת��
	buff[index++] = n32Val >> 24;																			
	buff[index++] = n32Val >> 16;																			
	buff[index++] = n32Val >> 8;																			
	buff[index++] = n32Val >> 0;																		
	n32Val = gRealData.latd * 1000000;																//γ��ת��
	buff[index++] = n32Val >> 24;																			
	buff[index++] = n32Val >> 16;																			
	buff[index++] = n32Val >> 8;																			
	buff[index++] = n32Val >> 0;						
	return MakeCmd(CMD_ANTITAMPER,vin,buff,index - 24,proc);
}

//�����������
static uint16_t packSetUp(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc)
{
	uint32_t sm2Len = 64;
	uint16_t DataAreLen = 0;
	uint8_t index = 24;
	uint8_t priKeyVal[64];
	/*ʱ��*/
	buff[index++] = (uint8_t)(g_system_dt.year - 2000);
	buff[index++] = (uint8_t)(g_system_dt.month);
	buff[index++] = (uint8_t)(g_system_dt.day);
	buff[index++] = (uint8_t)(g_system_dt.hour);
	buff[index++] = (uint8_t)(g_system_dt.minute);
	buff[index++] = (uint8_t)(g_system_dt.second);
	DataAreLen += 6;
	if(strlen(gFrimPara.scyId) == 16)																			//��ȡоƬID
	{
		memcpy(&buff[index],gFrimPara.scyId,16);
		index += 16;
		DataAreLen += 16;
//		if(tms_sm2_export_pubkey(0x0001,&buff[index],&sm2Len) == SUCCEED)																	//��ȡ��ԿA
//		{
//			index += sm2Len;
//			DataAreLen += sm2Len;
//			memcpy(&buff[index],vin,17);
//			index += 17;
//			DataAreLen += 17;
//			if(proc == 2 || proc == 3)
//			{
//				if(tms_sm2_with_sm3_signature(&buff[24],DataAreLen,priKeyVal,&sm2Len,0x0001,(uint8_t *)gFrimPara.scyId) == SUCCEED)
//				{
//					buff[index++] = 32;																				//ǩ��Rֵ����
//					DataAreLen++;
//					
//					memcpy(&buff[index],priKeyVal,32);												//ǩ��Rֵ
//					index	 += 32;
//					DataAreLen += 32;
//					
//					
//					buff[index++] = 32;																				//ǩ��Sֵ����
//					DataAreLen++;
//					
//					memcpy(&buff[index],&priKeyVal[32],32);										//ǩ��Sֵ
//					index += 32;
//					DataAreLen += 32;
//				}
//			}
//			return MakeCmd(CMD_SETUP,vin,buff,index - 24,proc);			
//		}
	}
	return 0;
}

/*���OBD����*/
static uint16_t PackObdData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = pRealData->obdDiagProt;							//OBD���Э��
		bPackBuf[Idx++] = pRealData->milState;								//MIL״̬
		/*���֧��״̬*/
		bPackBuf[Idx++] = pRealData->diagSpState.value >> 8;	//���֧��״̬���ֽ�
		bPackBuf[Idx++] = pRealData->diagSpState.value;				//���֧��״̬���ֽ�
		/*��Ͼ���״̬*/
		bPackBuf[Idx++] = pRealData->diagRdyState.value >> 8;	//��Ͼ���״̬���ֽ�
		bPackBuf[Idx++] = pRealData->diagRdyState.value;			//��Ͼ���״̬���ֽ�
		/*����ʶ���*/
		memcpy(&bPackBuf[Idx], pRealData->vin,17);						//ʵʱVIN
		Idx	+=	17;
		/*����궨ʶ���*/
		memcpy(&bPackBuf[Idx], pRealData->softCalId,18);			//����궨ʶ���
		Idx	+=	18;	
		/*�궨��֤�루CVN��*/
		memcpy(&bPackBuf[Idx], pRealData->cvn,18);						//CVN
		Idx	+=	18;	
		/*IUPRֵ*/
		memcpy(&bPackBuf[Idx], pRealData->iuprVal,36);				//IUPR
		Idx	+=	36;	
		if(pRealData->faultCodeCnt <= MAX_FAULT_NUM)
		{
			uint8_t i = 0;
			/*����������*/
			bPackBuf[Idx++] = pRealData->faultCodeCnt;					//����������
			//��������Ϣ�б�
			for(i=0;i<pRealData->faultCodeCnt;i++)
			{
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 24;	//������
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 16;	//������
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 8;		//������
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 0;		//������
			}
		}
		else
		{
			bPackBuf[Idx++] = 0xFE;															//������������Ч
		}
	}
	return Idx;
}

/*���������*/
static uint16_t PackVehicleData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	uint32_t n32Val = 0;
	if(bPackBuf != 0)
	{
		if(pRealData->speed != 0xFFFF)													//����ת��
		{
			n32Val = pRealData->speed * 256;					
			n32Val &= 0xFFC0;
		}
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;													//���ٸ��ֽ�
		bPackBuf[Idx++] = n32Val;																//���ٵ��ֽ�
		if(pRealData->barometric != 0xFF)
			bPackBuf[Idx++] = pRealData->barometric * 2;					//����ѹ��
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->engineTorque != 0xFF)
			bPackBuf[Idx++] = pRealData->engineTorque + 125;			//���������Ť��/ʵ��Ť��
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->frictionTorque != 0xFF)
			bPackBuf[Idx++] = pRealData->frictionTorque + 125;		//Ħ��Ť��
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->engineSpeed != 0xFFFF)
			n32Val = pRealData->engineSpeed * 8;									//���ͻ�ת��ת��
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;													//���ͻ�ת�ٸ��ֽ�
		bPackBuf[Idx++] = n32Val;																//���ͻ�ת�ٵ��ֽ�
		if(pRealData->engineFuelFlow != 0xFFFF)
			n32Val = pRealData->engineFuelFlow * 20;							//������ȼ������ת��
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;													//������ȼ���������ֽ�
		bPackBuf[Idx++] = n32Val;																//������ȼ���������ֽ�
		
		if(pRealData->technology == 0)									//����·��
		{
			if(pRealData->scrUpperNOxSensor != 0xFFFF)
				n32Val = (pRealData->scrUpperNOxSensor + 200) * 20;	//SCR����NOx���������ֵת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;												//SCR����NOx���������ֵ���ֽ�
			bPackBuf[Idx++] = n32Val;															//SCR����NOx���������ֵ���ֽ�	
			if(pRealData->scrLowerNOxSensor != 0xFFFF)
				n32Val = (pRealData->scrLowerNOxSensor + 200) * 20;	//SCR����NOx���������ֵת��
			else
				n32Val = 0xFFFF;		
			bPackBuf[Idx++] = n32Val >> 8;												//SCR����NOx���������ֵ���ֽ�
			bPackBuf[Idx++] = n32Val;															//SCR����NOx���������ֵ���ֽ�	
			if(pRealData->reagentSurplus != 0xFF)
				bPackBuf[Idx++] = (uint8_t)(pRealData->reagentSurplus * 2.5l);		//��Ӧ������
			else
				bPackBuf[Idx++] = 0xFF;
			if(pRealData->intakeFlowrate != 0xFFFF)
				n32Val = pRealData->intakeFlowrate * 20;													//������ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//���������ֽ�
			bPackBuf[Idx++] = n32Val;																						//���������ֽ�
			if(pRealData->scrInletTemp != 0xFFFF)
				n32Val = (pRealData->scrInletTemp + 273) * 32;										//SCR����¶�ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//SCR����¶ȸ��ֽ�
			bPackBuf[Idx++] = n32Val;																						//SCR����¶ȵ��ֽ�	
			if(pRealData->scrOutletTemp != 0xFFFF)
				n32Val = (pRealData->scrOutletTemp + 273) * 32;										//SCR�����¶�ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//SCR�����¶ȸ��ֽ�
			bPackBuf[Idx++] = n32Val;																						//SCR�����¶ȵ��ֽ�	
			if(pRealData->dpfPressDiff != 0xFFFF)
				n32Val = pRealData->dpfPressDiff * 10;														//dpfѹ��ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//dpfѹ����ֽ�
			bPackBuf[Idx++] = n32Val;																						//dpfѹ����ֽ�	
			if(pRealData->engineCoolantTemp != 0xFF)
				bPackBuf[Idx++] = pRealData->engineCoolantTemp + 40;							//��������ȴҺ�¶�
			else
				bPackBuf[Idx++] = 0xFF;
			if(pRealData->tankLevel != 0xFF)
				bPackBuf[Idx++] = (uint8_t)(pRealData->tankLevel * 2.5l);					//����Һλ
			else
				bPackBuf[Idx++] = 0xFF;		
		}
		else if(pRealData->technology == 1)									//����·��
		{
			if(gRealData.twcUpperOxySensor != 0xFFFF)
				n32Val = gRealData.twcUpperOxySensor / 0.0000305f;								//TWC����Oxy���������ֵ
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC����Oxy���������ֵ���ֽ�
			bPackBuf[Idx++] = n32Val;																						//TWC����Oxy���������ֵ���ֽ�	
			bPackBuf[Idx++] = (uint8_t)(gRealData.twcLowerOxySensor * 100);			//TWC����Oxy���������ֵ
			if(gRealData.intakeFlowrate != 0xFFFF)
				n32Val = gRealData.intakeFlowrate * 20;														//������ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//���������ֽ�
			bPackBuf[Idx++] = n32Val;																						//���������ֽ�
			if(gRealData.engineCoolantTemp != 0xFF)
				bPackBuf[Idx++] = gRealData.engineCoolantTemp + 40;								//��������ȴҺ�¶�
			else
				bPackBuf[Idx++] = 0xFF;
			if(gRealData.twcTemp != 0xFFFF)
				n32Val = (gRealData.twcTemp + 273) * 32;													//TWC�¶�ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC�¶ȸ��ֽ�
			bPackBuf[Idx++] = n32Val;																						//TWC�¶ȵ��ֽ�	
		}
		if(pRealData->technology == 2)
		{
			if(gRealData.twcUpperOxySensor != 0xFFFF)
				n32Val = gRealData.twcUpperOxySensor / 0.0000305f;								//TWC����Oxy���������ֵ
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC����Oxy���������ֵ���ֽ�
			bPackBuf[Idx++] = n32Val;																						//TWC����Oxy���������ֵ���ֽ�	
			bPackBuf[Idx++] = (uint8_t)(gRealData.twcLowerOxySensor * 100);			//TWC����Oxy���������ֵ
			
			if(gRealData.twcLowerNOxSensor != 0xFFFF)
				n32Val = (gRealData.twcLowerNOxSensor + 200) * 20;								//TWC����NOx���������ֵת��
			else
				n32Val = 0xFFFF;		
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC����NOx���������ֵ���ֽ�
			bPackBuf[Idx++] = n32Val;																						//TWC����NOx���������ֵ���ֽ�				
			
			if(gRealData.intakeFlowrate != 0xFFFF)
				n32Val = gRealData.intakeFlowrate * 20;														//������ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//���������ֽ�
			bPackBuf[Idx++] = n32Val;																						//���������ֽ�
			if(gRealData.engineCoolantTemp != 0xFF)
				bPackBuf[Idx++] = gRealData.engineCoolantTemp + 40;								//��������ȴҺ�¶�
			else
				bPackBuf[Idx++] = 0xFF;
			if(gRealData.twcTemp != 0xFFFF)
				n32Val = (gRealData.twcTemp + 273) * 32;													//TWC�¶�ת��
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC�¶ȸ��ֽ�
			bPackBuf[Idx++] = n32Val;																						//TWC�¶ȵ��ֽ�			
		}
		
		bPackBuf[Idx++] = pRealData->locationState;														//��λ״̬
		n32Val = pRealData->longd * 1000000;																	//����ת��
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;																		
		n32Val = pRealData->latd * 1000000;																		//γ��ת��
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;																		
		n32Val = pRealData->totalMileage * 10;																//�����ת��
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;
		
		if(gSysPara.carType == 1 && pRealData->soc != 0 && pRealData->soc != 0xFF)
		{
			/*��϶����綯��������������*/
			bPackBuf[Idx++] = 0x04;																			//��϶����綯��������������
			/*ʱ��*/
			bPackBuf[Idx++] = (uint8_t)(pRealData->year - 2000);
			bPackBuf[Idx++] = (uint8_t)(pRealData->month);
			bPackBuf[Idx++] = (uint8_t)(pRealData->day);
			bPackBuf[Idx++] = (uint8_t)(pRealData->hour);
			bPackBuf[Idx++] = (uint8_t)(pRealData->minute);
			bPackBuf[Idx++] = (uint8_t)(pRealData->second);
			bPackBuf[Idx++] = pRealData->motorData[0].motorSpeed >> 8;	//���ת��
			bPackBuf[Idx++] = pRealData->motorData[0].motorSpeed;				//���ת��
			bPackBuf[Idx++] = pRealData->motorData[0].motorLoad;				//������ɰٷֱ�
			n32Val = (uint16_t)(pRealData->total_volt);									//��ص�ѹ
			if(n32Val != 0xFFFF && n32Val != 0xFFFE)
				n32Val = (uint16_t)(pRealData->total_volt * 10);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 8);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 0);
			
			n32Val = (uint16_t)(pRealData->total_current);								//��ص���
			if(n32Val != 0xFFFF && n32Val != 0xFFFE)
				n32Val = (uint16_t)(pRealData->total_current * 10 + 10000);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 8);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 0);
			
			bPackBuf[Idx++] = (uint8_t)(pRealData->soc);									//SOC		
		}
	}
	return Idx;
}

/*��� ���� ������*/
#pragma diag_suppress 177
static uint16_t PackAuxData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	uint32_t n32Val = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = pRealData->engineTorqueMode;										//������Ť��ģʽ
		if(pRealData->acceleratorVal != 0xFF)
			bPackBuf[Idx++] = (uint8_t)(pRealData->acceleratorVal * 2.5l);	//����̤��
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->EngTotalFuelUsed == 0xFFFFFFFF)
			n32Val= 0xFFFFFFFF;
		else
			n32Val = pRealData->EngTotalFuelUsed * 2;												//�ۼ��ͺ�
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;				
		if(pRealData->ureaTankTemp != 0xFF)
			bPackBuf[Idx++] = pRealData->ureaTankTemp + 40;									//�������¶�
		else
			bPackBuf[Idx++] = 0xFF;		
		if(pRealData->actAreaInjectVal == 0xFFFFFFFF)
			n32Val= 0xFFFFFFFF;
		else
			n32Val = pRealData->actAreaInjectVal * 100;											//ʵ������������
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;			
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 24;									//�ۼ���������														
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 16;																			
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 8;																			
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 0;			
		if(pRealData->dpfExhaustTemp != 0xFFFF)
			n32Val = (pRealData->dpfExhaustTemp + 273) * 32;								//DPF�����¶�ת��
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;																		//DPF�����¶ȸ��ֽ�
		bPackBuf[Idx++] = n32Val;																					//DPF�����¶ȵ��ֽ�
	}
	return Idx;
}

/*
*  ����˵��: ���ʵʱ�������ڴ洢
*  ����wbuf: ���뻺���� mode:0 17691��ʽ 1:��������ʽ 2:��������ʽ��ǩ��
*  �� �� ֵ: �������
*/
static uint16_t buildRealData(uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc,uint8_t obdFlag)
{
	RealData* pRealData;
	uint8_t priKeyVal[64];
	uint16_t index,subLen,totalLen = 0;
	int8_t i;
	uint32_t sm2Len = 64;
	uint8_t outDataIdx;
	//����������������ʼ,Ԥ�����ķ���ʱ�����ˮ��
	index = 32;
	for(i = 9;i >= 0;i--)
	{
		pRealData = getRealCache(i,&outDataIdx);
		i = outDataIdx;
		if(proc == 0)
		{
			//GB 17691�����ʽ
			uint8_t *pbuf;
			pbuf = &buff[totalLen];
			//���ݷ���ʱ��
			index = 24;
			pbuf[index++] = (uint8_t)(pRealData->year - 2000);
			pbuf[index++] = (uint8_t)(pRealData->month);
			pbuf[index++] = (uint8_t)(pRealData->day);
			pbuf[index++] = (uint8_t)(pRealData->hour);
			pbuf[index++] = (uint8_t)(pRealData->minute);
			pbuf[index++] = (uint8_t)(pRealData->second);
			//����������
			pbuf[index++] = 0x02;
			//��ˮ��
			pbuf[index++] = ((msgNo * 10) + 9 - i) >> 8;																			
			pbuf[index++] = ((msgNo * 10) + 9 - i) >> 0;
			index += PackVehicleData(pRealData,&pbuf[index]);			//���������
			subLen = MakeCmd(cmd,vin,pbuf,index - 24,proc);
			if(obdFlag == 1 && i == 0)
			{
				totalLen += subLen;
				pbuf = &buff[totalLen];
				//���ݷ���ʱ��
				index = 24;
				pbuf[index++] = (uint8_t)(pRealData->year - 2000);
				pbuf[index++] = (uint8_t)(pRealData->month);
				pbuf[index++] = (uint8_t)(pRealData->day);
				pbuf[index++] = (uint8_t)(pRealData->hour);
				pbuf[index++] = (uint8_t)(pRealData->minute);
				pbuf[index++] = (uint8_t)(pRealData->second);
				//OBD����
				pbuf[index++] = 0x01;
				//��ˮ��
				pbuf[index++] = ((msgNo * 10) + i) >> 8;																			
				pbuf[index++] = ((msgNo * 10) + i) >> 0;
				index += PackObdData(pRealData,&pbuf[index]);
				subLen = MakeCmd(cmd,vin,pbuf,index - 24,proc);
			}
			totalLen += subLen;
			continue;
		}
		else
		{
			//���������ģʽ
			//����������
			if(pRealData->technology == 0)
			{
				/*��������Ϣ*/
				buff[index++] = 0x02;										//��������Ϣ
			}
			if(pRealData->technology == 1)
			{
				/*��������Ϣ*/
				buff[index++] = 0x03;										//��������Ϣ
			}
			if(pRealData->technology == 2)
			{
				/*��������Ϣ*/
				buff[index++] = 0x05;										//��������Ϣ
			}
			//���ݷ���ʱ��
			buff[index++] = (uint8_t)(pRealData->year - 2000);
			buff[index++] = (uint8_t)(pRealData->month);
			buff[index++] = (uint8_t)(pRealData->day);
			buff[index++] = (uint8_t)(pRealData->hour);
			buff[index++] = (uint8_t)(pRealData->minute);
			buff[index++] = (uint8_t)(pRealData->second);	
			//��Ϣ��
			index += PackVehicleData(pRealData,&buff[index]);
			if(obdFlag == 1 && i == 0)
			{
				//OBD����
				buff[index++] = 0x01;	
				//���ݷ���ʱ��
				buff[index++] = (uint8_t)(pRealData->year - 2000);
				buff[index++] = (uint8_t)(pRealData->month);
				buff[index++] = (uint8_t)(pRealData->day);
				buff[index++] = (uint8_t)(pRealData->hour);
				buff[index++] = (uint8_t)(pRealData->minute);
				buff[index++] = (uint8_t)(pRealData->second);	
				//��Ϣ��
				index += PackObdData(pRealData,&buff[index]);				
			}	
			if(gSysPara.linkSwitch & PACK_AUX_BIT)
			{
				//����������
				buff[index++] = 0x80;
				//���ݷ���ʱ��
				buff[index++] = (uint8_t)(pRealData->year - 2000);
				buff[index++] = (uint8_t)(pRealData->month);
				buff[index++] = (uint8_t)(pRealData->day);
				buff[index++] = (uint8_t)(pRealData->hour);
				buff[index++] = (uint8_t)(pRealData->minute);
				buff[index++] = (uint8_t)(pRealData->second);	
				//��Ϣ��
				index += PackAuxData(pRealData,&buff[index]);
			}

			if(i == 0)
			{
				//���ݷ���ʱ��
				buff[24] = (uint8_t)(pRealData->year - 2000);
				buff[25] = (uint8_t)(pRealData->month);
				buff[26] = (uint8_t)(pRealData->day);
				buff[27] = (uint8_t)(pRealData->hour);
				buff[28] = (uint8_t)(pRealData->minute);
				buff[29] = (uint8_t)(pRealData->second);
				//��ˮ��
				buff[30] = (uint8_t)(msgNo >> 8);
				buff[31] = (uint8_t)(msgNo >> 0);
				if(proc == 2 || proc == 3)
				{
					if(tms_sm2_with_sm3_signature(&buff[24],index - 24,priKeyVal,&sm2Len,0x0001,(uint8_t *)gFrimPara.scyId) == SUCCEED)
					{
						buff[index++] = 32;																				//ǩ��Rֵ����
						memcpy(&buff[index],priKeyVal,32);												//ǩ��Rֵ
						index	 += 32;
						buff[index++] = 32;																				//ǩ��Sֵ����
						memcpy(&buff[index],&priKeyVal[32],32);										//ǩ��Sֵ
						index += 32;
					}
				}
				totalLen = MakeCmd(cmd,vin,buff,index - 24,proc);
			}
		}
	}
	return totalLen;
}

uint8_t rspBuf[50];
static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i;
	GBSTA* pSta = NULL;
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
	if(rLen < 25 )//���ݳ��Ȳ���
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
	else if(rLen != (szRecvBuf[22] << 8 | szRecvBuf[23]) + 25)//���ĳ��ȴ���
  {
		return;
	}
	if(memcmp(pSta->vin,&szRecvBuf[3],17) != 0)//���ܺŴ���
	{
		return;
	}
	switch(szRecvBuf[2])
	{
		case CMD_LOGIN:  //��Ӧ��������
		{
			if(pSta->bLogined == 0xFF)
			{
				pSta->sendOverTimeCnt = 0;
				pSta->bLogined = 1;
			}
		}
		break;
		case CMD_REALDATA:  		//��Ӧʵʱ����
		case CMD_REISSUEDATA:   //��Ӧ��������
		case CMD_ANTITAMPER:
		{
			pSta->sendOverTimeCnt = 0;
		}
		break;
		case CMD_TIMING:  //��ӦУʱ
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
		break;
		case CMD_LOGOUT:  //��Ӧ�ǳ�
		{
			if(pSta->bLogouted != 0)
				pSta->bLogouted = 1;
		}
		break;
		case CMD_RSP_SETUP:  //��Ӧ����
		{
			//�����ɹ����ظ�����
			//01�ɹ� 0201�Ѿ����� 0202VIN���� 0203δ֪����
			if(szRecvBuf[24]==0x01 || (szRecvBuf[24]==0x02 && szRecvBuf[25]==0x01))
			{
				//�����ɹ�������ɹ���־��������ʽ��������
				gFrimPara.setUpFlag = 0xAA;
				Device_ID_Save();		
			}
		}
		break;
		case CMD_PARA_QUERY:  //����
		{
			uint16_t resLen = 0;
			rspBuf[24] = (uint8_t)(g_system_dt.year - 2000);
			rspBuf[25] = (uint8_t)(g_system_dt.month);
			rspBuf[26] = (uint8_t)(g_system_dt.day);
			rspBuf[27] = (uint8_t)(g_system_dt.hour);
			rspBuf[28] = (uint8_t)(g_system_dt.minute);
			rspBuf[29] = (uint8_t)(g_system_dt.second);
			resLen = MakeCmd(CMD_PARA_QUERY,pSta->vin,rspBuf,6,pSta->proc);
			Fun_Gprs_Tcp_send(pSta->bLink,rspBuf,resLen);
		}
		break;
		default:
		{
			;
		}
		break;
	}
	return;
}

