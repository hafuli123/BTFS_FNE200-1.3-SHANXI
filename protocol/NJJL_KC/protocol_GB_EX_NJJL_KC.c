/*
�� ����protocol_GB_EX_NJJL_KC.c
�� �ܣ��Ͼ���ͨ��Э�� - ���������Զ�����չ����
�� ��: 2021/12/30
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: CZJ -> LGC
*/

#include "NJJL_KC/protocol_GB_EX_NJJL_KC.h"

#define MAX_YZT_LINK 1
uint8_t sendUseVIN_TermID = 1;		//1��Ĭ��ʹ���ն�ID���� 	0��ʹ��VIN����
uint8_t sendSelfData = 1;					//1��Ĭ�������Զ������� 	0���������Զ�������

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
USER_DATA gUserDara;												/* Զ���������� */
static uint32_t reConnectStamp = 0;					//�ز���ʱ

static uint8_t lockResponCout = 0;					//������Ӧ����

/*
���ܣ��洢�û�����
		�洢����ָ���������
*/
void saveUserData(void)
{
		gUserDara.store_flag = 0xAA;
		User_ID_Save(&gUserDara,sizeof(USER_DATA));				
}


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
	buff[30] = (uint8_t)(gUserDara.carDoorCtrCount >> 8);         	//������ˮ��
	buff[31] = (uint8_t)(gUserDara.carDoorCtrCount >> 0);
	buff[32] = gUserDara.carDoorCtrRspCode;
	return MakeCmd(CMD_LOCKCARRRSP,0xFE,vin,buff,9);
}

/*
���ܣ���������ָ��
*/
static uint8_t UnpackLockCMD(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.carDoorCtrCount = rbuf[30] << 8 | rbuf[31];			//��ˮ��
	gUserDara.carDoorCtrCode = rbuf[32];											//��������ָ��
	gUserDara.carDoorCtrRspCode = 0;													//������Ӧ����
	gUserDara.isReturnLockState = 0xAA;													
	
	sendLockCMDSign = 5;																			//��������ָ���
	
	//����COMMON_PARA
	saveUserData();
	
	DataAreaLen = MakeCmd(CMD_LOCKCAR,replyFlag,vin,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}


/*
���ܣ���չЭ���ʼ��
��������������״̬����
*/
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	
	/* ��ȡ�û����� */
	if(gUserDara.store_flag != 0xAA)
	{
		User_ID_Read(&gUserDara,sizeof(USER_DATA));		
		gUserDara.store_flag = 0xAA;		
	}
	
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
	
	return &gbSta[objLinkIdx];
}

/*
���ܣ���չЭ������   
������ctrl:0 �������� 1:�������� 2:˯�� 
			sta:0 ����δ���� 1:�����ѵ��� 
			����ֵ:0 ��չЭ��δ���� 1:��չЭ���ѵ���
*/
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
	if(pSta->sendOverTimeCnt >= 3 &&  osKernelGetTickCount() - reConnectStamp >= 30000)
	{
		reConnectStamp = osKernelGetTickCount();
		pSta->sendOverTimeCnt = 0;
		Fun_Gprs_Tcp_disconnect(pSta->bLink);					//����3�γ�3����Ӧ��ʱ,�ز�
	}
	//�ϱ�����״̬
	if(osKernelGetTickCount() - pSta->lockStaStamp >= 3000 && gUserDara.isReturnLockState == 0xA5)//����״̬�ظ�
	{
		pSta->lockStaStamp = osKernelGetTickCount();
		lockResponCout++;
		if(gUserDara.carDoorCtrRspCode == 1 || lockResponCout > 10)			//�յ�����״̬���������ߵȴ�30��
		{
			if((dataLen = packCtrlStaFeedBack(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
			{
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			
				gUserDara.isReturnLockState = 0xAA;
				lockResponCout = 0;
				saveUserData();
			}
		}
	}
	return 1;
}


/*
���ܣ���չЭ����ս��
*/
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

/*
���ܣ����B0����
*/
uint16_t PackB0Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	bPackBuf[0] = 0xB0;
	SelfDataB0*  pB0Data = (SelfDataB0*)&pRealData->externData[selfDataB0Pos];		
	
	//����Զ�������
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 24);											//�ۼ����ĵ���		
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 16);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 8);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 0);
	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 24);											//�ۼƳ������		
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 16);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 8);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 0);
	
	bPackBuf[PackLen++] = pB0Data->sBatsBreakOnceEnergy;																			//�����ƶ���������
	bPackBuf[PackLen++] = pB0Data->sSOH;																											//SOH
	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsPower >> 8);
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsPower >> 0);																//��ع���																						//Ԥ��5			
	
	//��䳤��
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
���ܣ����B2����
*/
uint16_t PackB2Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	
	bPackBuf[0] = 0xB2;
	SelfDataB2*  pB2Data = (SelfDataB2*)&pRealData->externData[selfDataB2Pos];		
	//����Զ�������
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE1;											//sBMSDTC_CODE1
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE2;											//sBMSDTC_CODE2
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE3;											//sBMSDTC_CODE3
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE4;											//sBMSDTC_CODE4
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE5;											//sBMSDTC_CODE5
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE6;											//sBMSDTC_CODE6
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE7;											//sBMSDTC_CODE7
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE8;											//sBMSDTC_CODE8

	bPackBuf[PackLen++] = pB2Data->sBMSCode1;													//sBMSCode1
	bPackBuf[PackLen++] = pB2Data->sBMSCode2;													//sBMSCode2

	bPackBuf[PackLen++] = pB2Data->sDTC_Code;													//sDTC_Code		

	//��䳤��
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;
}

/*
���ܣ����B3����
*/
uint16_t PackB3Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	bPackBuf[0] = 0xB3;
	SelfDataB3*  pB3Data = (SelfDataB3*)&pRealData->externData[selfDataB3Pos];		
	//����Զ�������	
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_1_8;						//1-8		�ӿ�����״̬
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_9_16;					//9-16	�ӿ�����״̬
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_17_24;					//17-24	�ӿ�����״̬
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_25_32;					//25_32	�ӿ�����״̬
	
	bPackBuf[PackLen++] = pB3Data->sBatsCount;												//���������
	bPackBuf[PackLen++] = pB3Data->sBatSlaveControlCount;							//�ӿ���

	//��䳤��
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;
}

/*
���ܣ����B4����
*/
uint16_t PackB4Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	uint8_t usTems = 0;
	
	bPackBuf[0] = 0xB4;
	SelfDataB4*  pB4Data = (SelfDataB4*)&pRealData->externData[selfDataB4Pos];		
	//����Զ�������
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sTCU_FaultCode >> 8);				//TCU���ϴ���
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sTCU_FaultCode >> 0);				

	bPackBuf[PackLen++] = pB4Data->sTCU_OtherFaultCode;										//TCU�������ϴ���
	bPackBuf[PackLen++] = pB4Data->sTCU_FaultLevel;												//TCU���ϵȼ�
	bPackBuf[PackLen++] = pB4Data->sMoter_FaultLevel;											//����������ϵȼ�

	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMoter_DTCCode >> 8);				//�������DTC_CODE
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMoter_DTCCode >> 0);				

	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMotor_FaultCount >> 8);			//���������������
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMotor_FaultCount >> 0);				

	usTems = ((pB4Data->sMCU_TempAlarm & 0x03)<<0);												//MCU�¶ȱ���
	usTems |= ((pB4Data->sMotor_TempAlarm & 0x03)<<2);										//��������¶ȱ���
	bPackBuf[PackLen++] = usTems;

	//��䳤��
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
���ܣ����B5����
*/
uint16_t PackB5Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	bPackBuf[0] = 0xB5;
	SelfDataB5*  pB5Data = (SelfDataB5*)&pRealData->externData[selfDataB5Pos];		
	//����Զ�������
	bPackBuf[PackLen++] = pB5Data->sCarFaultLevel;											//�������ϵȼ�
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum1;												//����������1
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum2;												//����������2
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum3;												//����������3
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum4;												//����������4
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum5;												//����������5	

	//��䳤��
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
���ܣ����BF����
*/
uint16_t PackBFData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	uint8_t usTems = 0;

	bPackBuf[0] = 0xBF;
	SelfDataBF*  pBFData = (SelfDataBF*)&pRealData->externData[selfDataBFPos];		
	//����Զ�������
	usTems = 0;
	usTems |= (pBFData->sCE_LockST<<0);								//������״̬
	usTems |= (pBFData->sCE_DropLockST<<4);						//����������״̬
	bPackBuf[PackLen++] = usTems;

	//��䳤��
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
���ܣ����98����
*/
uint16_t Pack98Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0;
	bPackBuf[0] = 0x98;																			//��Ϣ���ͣ��Զ�������0x80
	PackLen = 3;
	bPackBuf[PackLen++] = gUserDara.remoteLockState;	//Զ������״̬				
	bPackBuf[PackLen++] = gUserDara.heartLockState;		//��������״̬		
	bPackBuf[1] = (uint8_t)((PackLen-3)>>8);
	bPackBuf[2] = (uint8_t)((PackLen-3)>>0);
	return PackLen;	
}

/*
���ܣ��Զ�����������
��������չʵʱ���ݣ�ʵʱ���������Զ�������
*/
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;
	if(sendSelfData == 1)									//1��Ĭ�������Զ������� 	0���������Զ�������
	{
		index += PackB0Data(pRealData,&bPackBuf[index]);
		index += PackB2Data(pRealData,&bPackBuf[index]);
		index += PackB3Data(pRealData,&bPackBuf[index]);
		index += PackB4Data(pRealData,&bPackBuf[index]);
		index += PackB5Data(pRealData,&bPackBuf[index]);
		index += PackBFData(pRealData,&bPackBuf[index]);
		index += Pack98Data(pRealData,&bPackBuf[index]);
	}
	return index;
}

/*
���ܣ�ʹ���ն˱�ţ������豸�Ŵ���VIN��������ƽ̨
*/
uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
	if(sendUseVIN_TermID == 1)							//1��Ĭ��ʹ���ն�ID���� 	0��ʹ��VIN����
	{
		//if(!(cmd == 1 && rsp == 1))
		{
			memset(vin,'0',17);
			memcpy(&vin[0],gFrimPara.terminalId,12);
			return 1;
		}	
	}

	return 0;
}
