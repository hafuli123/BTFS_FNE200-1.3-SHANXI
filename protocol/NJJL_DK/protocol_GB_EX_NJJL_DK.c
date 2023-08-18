/*
�� ����protocol_GB_EX_NJJL_DK.c
�� �ܣ��Ͼ�����ͨ��Э�� - ���������Զ�����չ����
�� ��: 2021/12/9
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: CZJ -> LGC
*/

#include "NJJL_DK/protocol_GB_EX_NJJL_DK.h"

/*�Ͼ���������Զ�������*/
SelfData80* pSelfData80;					
SelfData81* pSelfData81;
SelfData82* pSelfData82;
SelfData83* pSelfData83;
SelfData84* pSelfData84;
SelfData85* pSelfData85;
SelfData86* pSelfData86;
SelfData87* pSelfData87;
SelfData88* pSelfData88;
/* Զ���������� */
REMOTELOCK_PARA gRemoteLockPara;	
uint8_t sendLockCMDSign = 0;									//��������ָ���

static uint16_t p80offset;
static uint16_t p81offset;
static uint16_t p82offset;
static uint16_t p83offset;
static uint16_t p84offset;
static uint16_t p85offset;
static uint16_t p86offset;
static uint16_t p87offset;
static uint16_t p88offset;

//������ݱ�ͷ��У��,����ֵ��
static uint16_t MakeCmd(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *wbuf,uint16_t dataLen)
{
	wbuf[0] = 0x23;
	wbuf[1] = 0x23;
	wbuf[2] = cmd;
	wbuf[3] = rsp;
	memcpy(&wbuf[4],vin,17);
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen >> 0;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

/* 
���ܣ������������״̬����
*/
static uint16_t packCtrlStaFeedBack(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen)
{
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	buff[30] = (uint8_t)(gRemoteLockPara.carDoorCtrCount >> 8);         	//������ˮ��
	buff[31] = (uint8_t)(gRemoteLockPara.carDoorCtrCount >> 0);
	buff[32] = gRemoteLockPara.carDoorCtrRspCode;
	return MakeCmd(CMD_LOCKCARRRSP,0xFE,vin,buff,9);
}

static uint8_t UnpackLockCMD(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gRemoteLockPara.carDoorCtrCount = rbuf[30] << 8 | rbuf[31];				//��ˮ��
	gRemoteLockPara.carDoorCtrCode = rbuf[32];												//��������ָ��
	gRemoteLockPara.carDoorCtrRspCode = 0;														//������Ӧ����
	gRemoteLockPara.isReturnLockState = 0xAA;													
	
	sendLockCMDSign = 5;																							//��������ָ���
	
	//����COMMON_PARA
	System_Pare_Save();
	
	DataAreaLen = MakeCmd(CMD_LOCKCAR,replyFlag,vin,ResponCtrlBuf,DataAreaLen);
	//simSendData(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

#define MAX_YZT_LINK 1

//���籣����·����
typedef struct _CFG
{
	uint32_t canMsgSeq;							//can��Ϣ��ˮ��
	uint32_t canLogSeq;							//can��־��ˮ��
}CFG;

typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;										//�Ƿ�ʹ��
	char *vin;											//���ܺ�
	uint8_t sendOverTimeCnt;				//���ʹ���
	//�ڲ�ʱ���
	uint32_t lockStaStamp;					//��״̬ʱ���
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
	gbSta[objLinkIdx].lockStaStamp = 0;
	//�Զ������ݷ����ڴ棬����ƫ����
	p80offset = 0;
	pSelfData80 = (SelfData80*)&gRealData.externData[p80offset];
	
	p81offset = p80offset + sizeof(SelfData80);
	pSelfData81 = (SelfData81*)&gRealData.externData[p81offset];
	
	p82offset = p81offset + sizeof(SelfData81);
	pSelfData82 = (SelfData82*)&gRealData.externData[p82offset];
	
	p83offset = p82offset + sizeof(SelfData82);
	pSelfData83 = (SelfData83*)&gRealData.externData[p83offset];
	
	p84offset = p83offset + sizeof(SelfData83);
	pSelfData84 = (SelfData84*)&gRealData.externData[p84offset];
	
	p85offset = p84offset + sizeof(SelfData84);
	pSelfData85 = (SelfData85*)&gRealData.externData[p85offset];

	p86offset = p85offset + sizeof(SelfData85);
	pSelfData86 = (SelfData86*)&gRealData.externData[p86offset];
	
	p87offset = p86offset + sizeof(SelfData86);
	pSelfData87 = (SelfData87*)&gRealData.externData[p87offset];
	
	p88offset = p87offset + sizeof(SelfData87);
	pSelfData88 = (SelfData88*)&gRealData.externData[p88offset];
	return &gbSta[objLinkIdx];
}

//��չЭ������   ctrl:0 �������� 1:�������� 2:˯�� sta:0 ����δ���� 1:�����ѵ��� ����ֵ:0 ��չЭ��δ���� 1:��չЭ���ѵ���
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	uint16_t dataLen = 0;
	if(pSta == NULL)//����ͨ��·δ���ã��������гɹ�
	{
		return 1;
	}
	else if(ctrl == 0 || ctrl == 2)//���߻�˯��
	{
		return 0;
	}
	if(pSta->sendOverTimeCnt >= 3 &&  osKernelGetTickCount() - pSta->lockStaStamp >= 3000)
	{
		pSta->sendOverTimeCnt = 0;
		//����3�γ�3����Ӧ��ʱ,�ز�
		//reConnect(pSta->bLink);
	}
	if(osKernelGetTickCount() - pSta->lockStaStamp >= 30000 && gRemoteLockPara.isReturnLockState == 0xA5)//����״̬�ظ�
	{
		pSta->lockStaStamp = osKernelGetTickCount();
		//if((dataLen = packCtrlStaFeedBack(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
		//  simSendData(pSta->bLink,pSta->buff,dataLen);
	}
	return 1;
}


//��չЭ����ս��
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i,ret = 1;
	GBSTA* pSta = NULL;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bLink == link && gbSta[i].bUse == 1)
		{
			pSta = &gbSta[i];
			break;
		}
	}
	if(pSta == NULL)//δ���ô���·
		return 0;
	switch(szRecvBuf[2])
	{
		case CMD_LOCKCAR:		//��������ָ��
		{
			pSta->lockStaStamp = 0;
			UnpackLockCMD(link,pSta->vin,szRecvBuf,rLen);
		}
		break;
		default: ret = 0;break;
	}
	return ret;
}

uint16_t Pack80Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x80;
	SelfData80*  p80Data = (SelfData80*)&pRealData->externData[p80offset];		
	
	//����Զ�������
	bPackBuf[index++] = p80Data->sCarState;															//����״̬				
	bPackBuf[index++] = p80Data->sRunModel;															//��������ģʽ
	bPackBuf[index++] =	(uint8_t)(p80Data->sRunMileage >> 8);
	bPackBuf[index++] = (uint8_t)(p80Data->sRunMileage >> 0);						//�������	
	bPackBuf[index++] = p80Data->sPowerAvgConsumption;									//�ٹ���ƽ�����
	bPackBuf[index++] = p80Data->sVCUTorsion >> 8;
	bPackBuf[index++] = p80Data->sVCUTorsion >> 0;											//VCUŤ������	
	bPackBuf[index++] =	(uint8_t)(p80Data->sPhaseCurr >> 8);
	bPackBuf[index++] = (uint8_t)(p80Data->sPhaseCurr >> 0);						//�����	
	bPackBuf[index++] = p80Data->sHighDevState; 												//��ѹ����ʹ��״̬
	bPackBuf[index++] = p80Data->sVCUTouchControlCmd;										//VCU�Ӵ�������ָ��	
	bPackBuf[index++] = p80Data->sVCUTouchCloseState;										//VCU�Ӵ����պ�״̬
	bPackBuf[index++] = p80Data->sVCUFault;															//VCU����
	bPackBuf[index++] = p80Data->sMCUFault;															//MCU����
	bPackBuf[index++] = p80Data->sLifeSign;															//Life�ź�
	bPackBuf[index++] = p80Data->sVCUVersionInfo;												//VCU�汾��Ϣ
	bPackBuf[index++] =	(uint8_t)(p80Data->sCarAcceleratedSpeed >> 8);
	bPackBuf[index++] = (uint8_t)(p80Data->sCarAcceleratedSpeed >> 0);	//�������ٶ�
	bPackBuf[index++] = p80Data->sCarState1;														//����״̬1
	bPackBuf[index++] = p80Data->sCarFaultState1;												//��������״̬1
	
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																						//Ԥ��3
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																						//Ԥ��4
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																						//Ԥ��5			
	
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack81Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	uint8_t Tmp = 0;
	
	bPackBuf[0] = 0x81;
	SelfData81*  p81Data = (SelfData81*)&pRealData->externData[p81offset];		
	//����Զ�������
		Tmp = 0;		
		if(p81Data->sBMSFaultLevel <= 3)
		{
			Tmp |= (p81Data->sBMSFaultLevel << 6);   													//BMS���ϵȼ�
		}
		if(p81Data->sBMSFaultShutHighPower <= 3)
		{
			Tmp |= (p81Data->sBMSFaultShutHighPower << 4);										//BMS�쳣״̬�������жϸ�ѹ
		}		
		Tmp = Tmp | (p81Data->sBalanceState > 0 ? 0x08 : 0x00);          		//����״̬
		Tmp = Tmp | (p81Data->sHeatState > 0 ? 0x04 : 0x00);         				//����״̬
		Tmp = Tmp | (p81Data->sBatCoolState > 0 ? 0x02 : 0x00);       	 		//�����ȴ״̬
		Tmp = Tmp | (p81Data->sCHGLinkState > 0 ? 0x01 : 0x00);           	//�������״̬
		bPackBuf[index++] = Tmp;
		
		Tmp = 0;		
		Tmp = Tmp | (p81Data->sSingleTemOffLinState > 0 ? 0x20 : 0x00); 		//�¶Ȳɼ�����״̬
		Tmp = Tmp | (p81Data->sSingleBatOffLinState > 0 ? 0x10 : 0x00); 		//�����ѹ�ɼ�����״̬
		Tmp = Tmp | (p81Data->sBatProtectOut > 0 ? 0x08 : 0x00);          	//��طŵ籣�������ڳ���С��10A�ŵ�2Сʱ�ж��ܸ���
		Tmp = Tmp | (p81Data->sFireLimitFaultAlarm > 0 ? 0x04 : 0x00);      //���ּ��޹��ϱ���
		Tmp = Tmp | (p81Data->sBranchPreAlarm > 0 ? 0x02 : 0x00);       	 	//֧·ѹ���
		Tmp = Tmp | (p81Data->sToucherLinkFault > 0 ? 0x01 : 0x00);         //�Ӵ���ճ������
		bPackBuf[index++] = Tmp;
				
		Tmp = 0;		
		Tmp = Tmp | (p81Data->sBMSControlOffLinAlarm > 0 ? 0x20 : 0x00); 		//BMS�ӿص��߱��������ĳ�����ݶ�ʧ��
		Tmp = Tmp | (p81Data->sCHGDevInfoAlarm > 0 ? 0x10 : 0x00); 					//�����ͨ�ű���
		Tmp = Tmp | (p81Data->sPreCHGAlarm > 0 ? 0x08 : 0x00);          		//Ԥ��籨��
		Tmp = Tmp | (p81Data->sBalanceAlarmState > 0 ? 0x04 : 0x00);      	//���ⱨ��״̬
		Tmp = Tmp | (p81Data->sHeatFaultAlarmState > 0 ? 0x02 : 0x00);     	//���ȹ��ϱ���״̬
		Tmp = Tmp | (p81Data->sBatCoolSystemFault > 0 ? 0x01 : 0x00);       //�����ȴϵͳ����
		bPackBuf[index++] = Tmp;	
		
		Tmp = 0;		
		Tmp = Tmp | (p81Data->sBatSystemOtherFault > 0 ? 0x80 : 0x00); 			//���ϵͳ��������
		Tmp = Tmp | (p81Data->sBMSCommunicationFault > 0 ? 0x40 : 0x00); 		//BMSͨѶ���ϣ����canӲ�����ϣ�		
		if(p81Data->sBatLowTemAlarm <= 3)
		{
			Tmp |= (p81Data->sBatLowTemAlarm <<4);														//��ص��±���
		}
		if(p81Data->sSOCDiffenceAlarm <= 3)
		{
			Tmp |= (p81Data->sSOCDiffenceAlarm << 2);													//SOC���챨��
		}	
		if(p81Data->sCHGCurrentAlarm <= 3)
		{
			Tmp |= (p81Data->sCHGCurrentAlarm << 0);													//����������
		}		
		bPackBuf[index++] = Tmp;		
		
		Tmp = 0;		
		if(p81Data->sOutCurrentAlarm <= 3)
		{
			Tmp |= (p81Data->sOutCurrentAlarm << 6);													//�ŵ��������
		}
		if(p81Data->sCHGGunHighTemAlarm <= 3)
		{
			Tmp |= (p81Data->sCHGGunHighTemAlarm << 2);												//���ǹ���±���
		}	
		if(p81Data->sPoleColumnHighTemAlarm <= 3)
		{
			Tmp |= (p81Data->sPoleColumnHighTemAlarm << 0);										//�������±���
		}	
		bPackBuf[index++] = Tmp;	
				
		bPackBuf[index++] = 0xFE;																						//Ԥ��9
		
		Tmp = 0;		
		if(p81Data->sInsulationAlarmState <= 3)
		{
			Tmp |= (p81Data->sInsulationAlarmState << 2);											//��Ե��ⱨ��״̬
		}	
		if(p81Data->sSOCHighAlarm <= 3)
		{
			Tmp |=(p81Data->sSOCHighAlarm << 0);															//SOC�߱���
		}	
		bPackBuf[index++] = Tmp;	
		
		Tmp = 0;		
		if(p81Data->sSOCLowAlarm1 <= 3)
		{
			Tmp |= (p81Data->sSOCLowAlarm1 << 6);															//SOC�ͱ���1
		}
		if(p81Data->sTemDiffAlarm1 <= 3)
		{
			Tmp |= (p81Data->sTemDiffAlarm1 << 4);														//�¶Ȳ��챨��1
		}
		if(p81Data->sSingleDifVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sSingleDifVlotAlarm<< 2);												//�����ѹ���챨��
		}	
		if(p81Data->sSingleLowVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sSingleLowVlotAlarm << 0);												//����Ƿѹ����
		}	
		bPackBuf[index++] = Tmp;

		Tmp = 0;		
		if(p81Data->sBatsLowVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sBatsLowVlotAlarm << 6);													//�����Ƿѹ����
		}
		if(p81Data->sBatsOverVlotAlarm <= 3)
		{		
			Tmp |= (p81Data->sBatsOverVlotAlarm << 4);												//������ѹ����
		}
		if(p81Data->sSingleOverVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sSingleOverVlotAlarm << 2);											//�����ѹ����
		}	
		if(p81Data->sBatHighTemAlarm1 <= 3)
		{
			Tmp |= (p81Data->sBatHighTemAlarm1 << 0);													//��ظ��±���1
		}	
		bPackBuf[index++] = Tmp;
		
		bPackBuf[index++] = p81Data->sBatFaultNum;													//������ع�����
		bPackBuf[index++] = p81Data->sBatsCount >> 8;
		bPackBuf[index++] = p81Data->sBatsCount >> 0;												//������ܴ���
		bPackBuf[index++] = p81Data->sBatsTemsCount;												//������¶ȵ���
		bPackBuf[index++] = p81Data->sMaxLongInCurrent;											//�����ó�����������5min��
		bPackBuf[index++] = p81Data->sMaxShortInCurrent;										//�����ö�ʱ��������30s��
		bPackBuf[index++] = p81Data->sMaxLongOutCurrent;										//�����ó����ŵ������5min��
		bPackBuf[index++] = p81Data->sMaxShortOutCurrent;										//�����ö�ʱ�ŵ������30s��
		bPackBuf[index++] = p81Data->sBMSTouchControlCMD;										//BMS�Ӵ�����������
		bPackBuf[index++] = p81Data->sBMSTouchControlCloseState;						//BMS�Ӵ����պ�״̬
		bPackBuf[index++] = p81Data->sCHGCounts >> 8;	
		bPackBuf[index++] = p81Data->sCHGCounts >> 0;												//������
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 24);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 16);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 8);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 0);			//������ۼ��������
		
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 24);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 16);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 8);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 0);			//������ۼƳ�磨�����ƶ�����������
		bPackBuf[index++] = p81Data->sInsulationCheckAllVolt >> 8;	
		bPackBuf[index++] = p81Data->sInsulationCheckAllVolt >> 0;					//��Ե�������ѹ
		bPackBuf[index++] = p81Data->sCHGPlusesTem1;												//�����1���¶�
		bPackBuf[index++] = p81Data->sCHGMinusTem1;													//���ǹ1���¶�
		bPackBuf[index++] = p81Data->sCHGPlusesTem2;												//�����2���¶�
		bPackBuf[index++] = p81Data->sCHGMinusTem2;													//���ǹ2���¶�
		bPackBuf[index++] = p81Data->sBatsProductDate_Month;								//������������ڣ��£�
		bPackBuf[index++] = p81Data->sBatsProductDate_Year;									//������������ڣ��꣩����
		bPackBuf[index++] = p81Data->sBatsProducer;													//���������������
		bPackBuf[index++] = p81Data->sBMSLifeSignal;												//BMS life�ź�
		bPackBuf[index++] = p81Data->sBMSSoftwareVersion;										//BMS����汾
		
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��2
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��3
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��4
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��5		

	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;
}

uint16_t Pack82Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x82;
	SelfData82*  p82Data = (SelfData82*)&pRealData->externData[p82offset];		
	//����Զ�������
	bPackBuf[index++] = p82Data->sFourInOne_State;												//4��1״̬		
	bPackBuf[index++] = p82Data->sFourInOne_FaultNum;											//4��1������
	bPackBuf[index++] = p82Data->sFourInOne_BMSToucherState;							//4��1 - BMS�Ӵ���״̬����
	bPackBuf[index++] = p82Data->sFourInOne_VCUToucherState;							//4��1 - VCU�Ӵ���״̬����
	bPackBuf[index++] = p82Data->sFourInOne_BMSToucherFaultState;					//4��1 - BMS�Ӵ�������״̬
	bPackBuf[index++] = p82Data->sFourInOne_VCUToucherFaultState;					//4��1 - VCU�Ӵ�������״̬	
	bPackBuf[index++] = (uint8_t)( p82Data->sHighOilPump_OutVolt >> 8);
	bPackBuf[index++] = (uint8_t)( p82Data->sHighOilPump_OutVolt >> 0);		//��ѹ�ͱ������ѹ		
	bPackBuf[index++] = p82Data->sHighOilPump_DCACOutCur;									//��ѹ�ͱ�DC/AC�������
	bPackBuf[index++] = p82Data->sHighOilPump_MoterTem;										//��ѹ�ͱõ���¶�
	bPackBuf[index++] = p82Data->sHighOilPump_DCACStateAndFault;					//��ѹ�ͱ�DC/AC״̬������
	bPackBuf[index++] = p82Data->sHighOilPump_ConverterFaultNum;					//��ѹ�ͱñ�Ƶ��������
	bPackBuf[index++] = p82Data->sHighOilPump_motorSpeed;									//��ѹ�ͱ�ת��
	bPackBuf[index++] = p82Data->sHighOilPump_DCACLifeSignal;							//��ѹ�ͱ�DC/AC life�ź�
	bPackBuf[index++] = p82Data->sDCDC_RealTimeOutCur;										//DC/DCʵʱ�������
	bPackBuf[index++] = p82Data->sDCDCTem;																//DC/DC�����¶�
	bPackBuf[index++] = p82Data->sDCDCWorkState;													//DCDC����״̬
	bPackBuf[index++] = p82Data->sDCDCLifeSignal;													//DCDC Life�ź�
	bPackBuf[index++] = (uint8_t)( p82Data->sAirPump_DCACOutVolt >> 8);
	bPackBuf[index++] = (uint8_t)( p82Data->sAirPump_DCACOutVolt >> 0);		//����DC/AC�����ѹ
	bPackBuf[index++] = p82Data->sAirPump_DCACOutCur;											//����DC/AC�������
	bPackBuf[index++] = p82Data->sAirPump_DCACStateAndFault;							//����DC/AC״̬������		
	bPackBuf[index++] = p82Data->sAirPump_Tem;														//�����¶�
	bPackBuf[index++] = p82Data->sAirPump_ConverterFaultNum;							//���ñ�Ƶ��������
	bPackBuf[index++] = p82Data->sAirPump_motorSpeed;											//����ת��
	bPackBuf[index++] = p82Data->sAirPump_DCACLifeSignal;									//����DC/AC Life�ź�
	bPackBuf[index++] = (uint8_t)( p82Data->sLowOilPump_OutVolt >> 8);
	bPackBuf[index++] = (uint8_t)( p82Data->sLowOilPump_OutVolt >> 0);		//��ѹ�ͱ������ѹ
	bPackBuf[index++] = p82Data->sLowOilPump_DCACOutCur;									//��ѹ�ͱ�DC/AC�������
	
	bPackBuf[index++] = p82Data->sLowOilPump_DCACStateAndFault;						//��ѹ�ͱ�DC/AC״̬������
	bPackBuf[index++] = p82Data->sLowOilPump_ConverterFaultNum;						//��ѹ�ͱñ�Ƶ��������
	bPackBuf[index++] = p82Data->sLowOilPump_motorSpeed;									//��ѹ�ͱ�ת��
	bPackBuf[index++] = p82Data->sLowOilPump_DCACLifeSignal;							//��ѹ�ͱ�DC/AC life�ź�
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//Ԥ��2
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//Ԥ��3
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//Ԥ��4
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//Ԥ��5		

	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;
}

uint16_t Pack83Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x83;
	SelfData83*  p83Data = (SelfData83*)&pRealData->externData[p83offset];		
	//����Զ�������	
		bPackBuf[index++] = p83Data->sLowBatVolt;														//��ѹ��ص�ѹ		
		bPackBuf[index++] =  p83Data->sFrontBrakeAirPressure;								//ǰ�ƶ�����Ͳ��ѹ
		bPackBuf[index++] =  p83Data->sRearBrakeAirPressure;								//���ƶ�����Ͳ��ѹ
		
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>24);						//�����	
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>16);
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>8);
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>0);										
		
		bPackBuf[index++] =  p83Data->sCarState1;														//����״̬1
		bPackBuf[index++] =  p83Data->sCarState2;														//����״̬2
		bPackBuf[index++] =  p83Data->sCarState3;														//����״̬3		
		bPackBuf[index++] =  p83Data->sInstrumentAlarmState1;								//�Ǳ���״̬1
		bPackBuf[index++] =  p83Data->sInstrumentAlarmState2;								//�Ǳ���״̬2
		bPackBuf[index++] = p83Data->sInstrumentSoftwareVersion;						//�Ǳ����汾

		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��19
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��20
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��21
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//Ԥ��22			
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;
}

uint16_t Pack84Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x84;
	SelfData84*  p84Data = (SelfData84*)&pRealData->externData[p84offset];		
	//����Զ�������
		bPackBuf[index++] = p84Data->sBMSCoolWorkMode;									//BMS��ȴ������ģʽ
		bPackBuf[index++] = p84Data->sBMSsetoutWaterTem;								//BMSˮ������ˮ�ڣ������ˮ�ڣ��趨�¶�
		bPackBuf[index++] = p84Data->sBatsHighestTem;										//���������¶�
		bPackBuf[index++] = p84Data->sBatsLowestTem;										//���������¶�
		bPackBuf[index++] = p84Data->sBatsQueLifeValue;									//���������lifeֵ
		bPackBuf[index++] = p84Data->sHotContrlMode;										//�ȹ���ϵͳ����ģʽ����
		bPackBuf[index++] = p84Data->sInWaterTem;												//��ˮ�¶�
		bPackBuf[index++] = p84Data->sOutWaterTem;											//��ˮ�¶�
		bPackBuf[index++] = p84Data->sCompressorPower;									//ѹ�������ر���
		bPackBuf[index++] = p84Data->sHotContrlFaultNum;								//�ȹ���ϵͳ������
		bPackBuf[index++] = p84Data->sHotContrlLifeValue;								//�ȹ�����lifeֵ

		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//Ԥ��23
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//Ԥ��24
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//Ԥ��25
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//Ԥ��26		
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}
uint16_t Pack85Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x85;
	SelfData85*  p85Data = (SelfData85*)&pRealData->externData[p85offset];		
	//����Զ�������
		bPackBuf[index++] = p85Data->sAirConditionerOpenCMD;						//�յ����ػ�����/״̬
		bPackBuf[index++] = p85Data->sAirConditionerSetTem;							//�յ��趨�¶�
		bPackBuf[index++] = p85Data->sAirConditionerRunStall;						//�յ�������е�λ
		bPackBuf[index++] = p85Data->sInCarTem;													//���ڻ����¶�		
		bPackBuf[index++] = p85Data->sAirConditionerRunMode;						//�յ���������ģʽ
		bPackBuf[index++] = p85Data->sOutCarTem;												//���⻷���¶�
		bPackBuf[index++] = p85Data->sAirToucherContrlAndState;					//�յ��Ӵ��������Լ�״̬
		bPackBuf[index++] = (uint8_t)(p85Data->sAirSystemVolt >> 8);													
		bPackBuf[index++] = (uint8_t)(p85Data->sAirSystemVolt >> 0);								//�յ�ϵͳĸ�ߵ�ѹ
		bPackBuf[index++] = p85Data->sAirSystem_PartsRunState;											//�յ�ϵͳһ��������״̬
		bPackBuf[index++] = p85Data->sAirSystem_SystemRunState;										//�յ�ϵͳһϵͳ����״̬
		bPackBuf[index++] = p85Data->sAirRunTargetHz;															//ѹ����Ŀ��Ƶ��
		bPackBuf[index++] = p85Data->sAirRunHz;																		//ѹ��������Ƶ��
		bPackBuf[index++] = p85Data->sAirFaultNum;																	//�յ�������
		bPackBuf[index++] = p85Data->sAirLife;																			//�յ�life	
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack86Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x86;
	SelfData86*  p86Data = (SelfData86*)&pRealData->externData[p86offset];		
	//����Զ�������
		bPackBuf[index++] = p86Data->sTirePosition;																//��̥λ��
		bPackBuf[index++] = p86Data->sTirePressure;																//��̥ѹ��
		bPackBuf[index++] = (uint8_t)(p86Data->sTireTem >> 8);																	
		bPackBuf[index++] = (uint8_t)(p86Data->sTireTem >> 0);										//��̥�¶�		
		bPackBuf[index++] = p86Data->sTireState;																	//״̬����̥״̬��		
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																									//Ԥ��27
		bPackBuf[index++] = p86Data->sPressureValveCheck;													//ѹ�������
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack87Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x87;
	SelfData87*  p87Data = (SelfData87*)&pRealData->externData[p87offset];		
	//����Զ�������
		bPackBuf[index++] = p87Data->sFlameArrester_SystemState;										//�����ϵͳ״̬
		bPackBuf[index++] = p87Data->sBatsNum;																			//������
		bPackBuf[index++] = p87Data->sSensorBoxState;															//�����ڴ���������״̬
		bPackBuf[index++] = p87Data->sSensorBoxFault;															//�����ڹ���״̬�ȼ�
		bPackBuf[index++] = p87Data->sSensorBoxTem;																//�������¶�
		bPackBuf[index++] = p87Data->sSensorBoxStartState;													//���������������״̬
		bPackBuf[index++] = 0xFF;																											//Ԥ��28
		bPackBuf[index++] = p87Data->sSensorBoxLife;																//LIFE
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack88Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x88;
	SelfData88*  p88Data = (SelfData88*)&pRealData->externData[p88offset];		
	//����Զ�������
		bPackBuf[index++] = p88Data->sAheadCarWarning;															//ǰ����ײ����
		bPackBuf[index++] = p88Data->sAheadCarDistance >> 8;												
		bPackBuf[index++] = p88Data->sAheadCarDistance >> 0;												//ǰ������
		bPackBuf[index++] = p88Data->sAheadCar_RelativeSpeed >> 8;									
		bPackBuf[index++] = p88Data->sAheadCar_RelativeSpeed >> 0;									//����ٶ�
		bPackBuf[index++] = p88Data->sLaneWarning;																	//����ƫ�뾯��
		bPackBuf[index++] = p88Data->sLaneDirection;																//����ƫ�뷽��
		bPackBuf[index++] = p88Data->sActivateCollisionWarning;										//������ײԤ������
		bPackBuf[index++] = p88Data->sActivateEmergencyBraking;										//���������ƶ�����
		bPackBuf[index++] = p88Data->sABESystemState;															//AEBϵͳ״̬
			
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																											//Ԥ��29	
	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack98Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0;
	bPackBuf[0] = 0x98;																			//��Ϣ���ͣ��Զ�������0x80
	PackLen = 3;
	bPackBuf[PackLen++] = gRemoteLockPara.remoteLockState;	//Զ������״̬				
	bPackBuf[PackLen++] = gRemoteLockPara.heartLockState;		//��������״̬		
	bPackBuf[1] = (uint8_t)((PackLen-3)>>8);
	bPackBuf[2] = (uint8_t)((PackLen-3)>>0);
	return PackLen;	
}

//��չʵʱ���ݣ�ʵʱ���������Զ�������
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;;
	index += Pack80Data(pRealData,&bPackBuf[index]);
	index += Pack81Data(pRealData,&bPackBuf[index]);
	index += Pack82Data(pRealData,&bPackBuf[index]);
	index += Pack83Data(pRealData,&bPackBuf[index]);
	index += Pack84Data(pRealData,&bPackBuf[index]);
	index += Pack85Data(pRealData,&bPackBuf[index]);
	index += Pack86Data(pRealData,&bPackBuf[index]);
	index += Pack87Data(pRealData,&bPackBuf[index]);
	index += Pack88Data(pRealData,&bPackBuf[index]);
	index += Pack98Data(pRealData,&bPackBuf[index]);
	return index;
}

uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
	if(!(cmd == 1 && rsp == 1))
	{
		memset(vin,'0',17);
		memcpy(&vin[5],gFrimPara.terminalId,12);
		return 1;
	}
	else if(resvBuff != NULL)
	{
		if(resvBuff[0] == 'L'  && memcmp(gSysPara.vinCode,resvBuff,17) != 0)
		{
			memcpy(gSysPara.vinCode,resvBuff,17);
			System_Pare_Save();
		}
		memcpy(vin,resvBuff,17);
	}
	return 0;
}
