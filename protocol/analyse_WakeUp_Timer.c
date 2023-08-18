/*
�� ����analyse_WakeUp_Timer.c
�� �ܣ���ʱ���ѹ���
�� ��: 2022/7/8
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: LGC
*/

#include "analyse_WakeUp_Timer.h"

#define MAX_YZT_LINK 1


enum PARA_ID{
	PARA_WAKE_TIME = 0x9C,				//���û���ʱ��
	PARA_WAKE_INTERVAL = 0x9D,			//���û��Ѽ��
	PARA_WAKE_SENDTIME = 0x9E,			//���û��Ѻ���������ʱ�䳤��
	
	PARA_HTTPURL = 0xA2,				//httpURL
	PARA_HTTPUSERNAME = 0xA3,			//httpUserName
	PARA_HTTPPASSWORD = 0xA4,			//httppassword
	PARA_GETCANLOG = 0xA5,				//CAN��־����
};

//���籣����·����
typedef struct _CFG
{
	uint32_t canMsgSeq;					//can��Ϣ��ˮ��
	uint32_t canLogSeq;					//can��־��ˮ��
}CFG;

typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;						//��·��
	uint8_t bUse;						//�Ƿ�ʹ��
	char *vin;							//���ܺ�
	uint8_t bLogouted;					//�ǳ���־λ 0:δ�ǳ� 1:�ѵǳ�
	uint8_t sendOverTimeCnt;			//���ʹ���
	//�ڲ�ʱ���
	uint32_t chgPowerStamp;				//�������ϱ�ʱ���
	//��������
	uint8_t* buff;						//�������ݻ�����
	uint16_t buffLen;					//�������ݻ���������
}GBSTA;

/* �û��Զ������ */
static GBSTA gbSta[MAX_YZT_LINK] = {0};
USER_DATA gUserDara;					//�û����ݴ洢

/* �������Ʋ��� */
static uint32_t reConnectStamp = 0;		//�ز���ʱ
static uint8_t printPackCmd = 1;		//��ӡ���ĵ����ڿ���λ

/* ���߻��Ѳ��� */
static int wakelock = -1;				//������
static uint64_t wakeTime = 0;			//����ʱ��
static uint32_t wk_timesss = 0;			//���ѵ���ʱ
uint8_t wk_Excute = 0;					//���ѱ�־����
uint8_t allow_UserOnLine = 0;			//�����û�ƽ̨����

/* Զ��CAN��־���� */
uint8_t getCanLogSwitch = 0;			//Զ��CAN��־�ɼ�����

/*
���ܣ��洢�û�����
	�洢����ָ���������
*/
void saveUserData(void)
{
	gUserDara.store_flag = 0xAA;
	User_ID_Save(&gUserDara,sizeof(USER_DATA));				
}

/*
���ܣ����ѻص�����
������	����1 ���²�����˯�ߣ������ߵ���ʱ���ٽ�������
*/
static uint8_t wakeCallback(uint8_t type)		
{
	wk_Excute = 1;
	wk_timesss = osKernelGetTickCount();
	allow_UserOnLine = 1;
	bsp_wakelock_lock(wakelock);
	
	autoChgState = 0;								//Ĭ�ϲ�������������
	sAllowAutoChg = 1;							//Ĭ��VCU������
	return 1;
}
/*
���ܣ�������ƽ̨����ʱ
������	û���Զ�����ʱ�����ﳬʱʱ�䣬����CAN����
			���߻��Ѻ��л���Դ���������ƽ̨����
*/
static void wk_lineTime(void)
{
	if(wk_Excute && autoChgState == 0)		//�ǲ���״̬��
	{		
		//�����ϱ���ʱ || �л���Դ
		if(osKernelGetTickCount() - wk_timesss > gUserDara.wk_SendInterval*10000)
		{
			if(bsp_wakelock_unlock(wakelock) == BSP_SLEEP_SUCCESS)
			{
				wk_Excute = 0;
				allow_UserOnLine = 0;											
			}
			printfData("wk_lineTime over\r\n");
		}
	}
}

/*
���ܣ����ö�ʱ����ʱ�� ����1
���������ö�ʱ����ʱ�䣬�ͻ��Ѽ��
ʱ���ʽ��2022:3:12-16:55:46
���Ѽ������				24Сʱ = 86400s
*/
static uint8_t isSetFirstV = 0;
static void setWakeTime(void)
{
	uint32_t temp = 0;
	temp = (uint8_t)gUserDara.wk_SendInterval;
	if(temp == 0||temp > 250)
		gUserDara.wk_SendInterval = 6;	
	
	temp = (uint32_t)gUserDara.wk_Interval;
	if(temp > 84600 || temp == 0)
		gUserDara.wk_Interval = 21600;
	
	temp = (uint16_t)gUserDara.wk_Year;
	if((temp<2022 || temp > 2060))
	{
		gUserDara.wk_Year = g_system_dt.year;
		gUserDara.wk_Month = g_system_dt.month;
		gUserDara.wk_Day = g_system_dt.day;
		gUserDara.wk_Hour = g_system_dt.hour;
		gUserDara.wk_Min = g_system_dt.minute;
		gUserDara.wk_Sec = g_system_dt.second;
		/* Ĭ�϶�ʱʱ�� */
		wakeTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
		/* Ĭ��6Сʱ */
		bsp_wakelock_para(wakelock,wakeTime,gUserDara.wk_Interval,wakeCallback,0,0);	
		isSetFirstV = 1;
	}
	else
	{
		/* ת����ʱ����ʱ�� */
		wakeTime = RTC_mktime(gUserDara.wk_Year,gUserDara.wk_Month,gUserDara.wk_Day,gUserDara.wk_Hour,gUserDara.wk_Min,gUserDara.wk_Sec);
		/* ���ö�ʱ����ʱ��κ����� */
		bsp_wakelock_para(wakelock,wakeTime,gUserDara.wk_Interval,wakeCallback,0,0);	
	}
	if(isSetFirstV==1)
	{
		saveUserData();
		isSetFirstV = 0;
	}
}


/*
���ܣ���ʱ����ʱ���ʼ ����2
������
	24Сʱ = 86400s
*/
static void set_WakeTime2(uint8_t swID)								//���ö�ʱ����ʱ��
{
	if(gUserDara.wk_Sec != g_system_dt.second && g_system_dt.second %2 == 0)	
	{		
		gUserDara.wk_Sec = g_system_dt.second;
		
		if(swID == 1)				//������ʱ����
		{
			uint32_t temp = 0;
			temp = (uint8_t)gUserDara.wk_SendInterval;
			if(temp == 0||temp > 250)
				gUserDara.wk_SendInterval = 6;	
			
			temp = (uint32_t)gUserDara.wk_Interval;
			if(temp > 84600 || temp == 0)
				gUserDara.wk_Interval = 21600;		
																																												/* Ĭ�϶�ʱʱ�� */
			wakeTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);	
			bsp_wakelock_para(wakelock,wakeTime,gUserDara.wk_Interval,wakeCallback,0,0);			/* Ĭ��6Сʱ */
		}
		else 								//�رն�ʱ����
		{
			bsp_wakelock_para(wakelock,0,0,0,0,0);				//�رն�ʱ���� ����Ӳ������
		}
	}
}

/*
���ܣ��������ݵ�����
*/
static uint8_t sendNetDatas(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	uint16_t sendLen = (buf[22] << 8 | buf[23] << 0) + 25;
	if(printPackCmd == 1)
	{
		comSendBuf(RS1_COM,buf,len);
	}
	Fun_Gprs_Tcp_send(socket_fd,buf,sendLen);
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
	gbSta[objLinkIdx].chgPowerStamp = 0;
	
	reConnectStamp = 0;
	reportVINStamp = 0;

	wakelock = bsp_wakelock_create();			/* ���������� */	
	/*setWakeTime();							���û���ʱ�� */
	
	return &gbSta[objLinkIdx];
}

/*
���ܣ���չЭ������   
������	
		ctrl:0 �������� 1:�������� 2:˯�� 
		sta:0 ����δ���� 1:�����ѵ��� 
����ֵ:
		0 ��չЭ��δ���� 1:��չЭ���ѵ���
*/
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	if(obj == NULL)
		return 1;
	uint16_t dataLen = 0;
	lowVoltProtect();																	//��ѹ�ж�

	if(pSta->bLink == 0)
	{
		if(autoChgSW == 1)
		{
			if(wk_Excute ==1 && pSelfData0A->sTboxST == 2)	//��ʱ�����߻��ѣ����ж��Ƿ񲹵�											
				autoChgPower();			
		}
		
		localtionFaultFun();														//��λ�쳣�ж�
			
		wk_lineTime();

		if(pSta == NULL)																//��·δ���ã��������гɹ�
		{ 
			return 1;
		}
		else if(ctrl == 0 || ctrl == 2)									//���߻�˯��
		{
			return 0;
		}
		if(sta == 1)
		{
			if(pSta->sendOverTimeCnt >= 3 &&  osKernelGetTickCount() - reConnectStamp >= 30000)
			{
				reConnectStamp = osKernelGetTickCount();
				pSta->sendOverTimeCnt = 0;
				Fun_Gprs_Tcp_disconnect(pSta->bLink);					//����3�γ�3����Ӧ��ʱ,�ز�
			}
			//�״λ�ȡVINʱ��10s����һ��ֱ�����յ�ƽ̨��Ӧ
			else if( fisrtGetVIN == 1 && osKernelGetTickCount() - reportVINStamp >= reportVINInterval)	
			{
				reportVINStamp = osKernelGetTickCount();
				if((dataLen = packUpdataVIN(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
				{
					sendNetDatas(pSta->bLink,pSta->buff,dataLen);
					reportVINfaildCnt++;
					if(reportVINfaildCnt == 3)
						reportVINInterval = 1800000;						//����3���ϱ����ɹ��ĳ�30�����ϱ�һ��
				}
			}		
			else if(chgPowerSign == 1 && osKernelGetTickCount() - pSta->chgPowerStamp >= reportChgPInterval)	
			{		
				//�������ϱ���10s����һ��ֱ�����յ�ƽ̨��Ӧ
					pSta->chgPowerStamp = osKernelGetTickCount();
					if((dataLen = packCHGPower(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
					{
						sendNetDatas(pSta->bLink,pSta->buff,dataLen);		
						reportChgPfaildCnt++;
						if(reportChgPfaildCnt == 3)
							reportChgPInterval = 1800000;						//����3���ϱ����ɹ��ĳ�30�����ϱ�һ��
					}					
			}			
		}
		else
		{
			//�ǳ������� VIN�����Ŀ��Ʋ���
			isChgVIN = 0;
			reportVINfaildCnt = 0;
			reportVINInterval = 10000;
		}
	}
	return 1;
}
/*
���ܣ������������ϱ�
������
			��ʱ���Ѻ���Ҫ����������
*/
uint8_t extCtrlNetSta(void* obj)
{
	GBSTA *pSta = obj;
	if(obj == NULL)
		return 0;
	
	if(pSta->bLink == 0)
		return allow_UserOnLine;
	else 
		return 0;
	
	return allow_UserOnLine;
}
uint8_t weakCtrlNetSta(void)
{
	return allow_UserOnLine;
}

/*
���ܣ���չЭ����ս��
������
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
		case CMD_LOCKCMD:								
			{
				UnpackLockCMD(link,pSta->vin,szRecvBuf,rLen);								//��������ָ��(����)
			}
			break;
		case CMD_SOFTVER_CHANGE:				
			{
				UnpackSoftVer_Change(link,pSta->vin,szRecvBuf,rLen);				//����汾�л�����	
			}
			break;
		case CMD_BINDINGCTR:						
			{
				UnpackBindingCtl(link,pSta->vin,szRecvBuf,rLen);						//�󶨿���
			}
			break;
		case CMD_DOORCTR:								
			{
				UnpackDoorCtl(link,pSta->vin,szRecvBuf,rLen);								//���ſ���
			}
			break;
		case CMD_MAINTTIPS:							
			{
				UnpackMaintTips(link,pSta->vin,szRecvBuf,rLen);							//������ʾ
			}
			break;
		case CMD_PAYTIPS:								
			{
				UnpackPayTips(link,pSta->vin,szRecvBuf,rLen);								//�ɷ���ʾ
			}
			break;
		case CMD_CHECKTIPS:							
			{
				UnpackYearCheckTips(link,pSta->vin,szRecvBuf,rLen);					//������ʾ
			}
			break;
		case CMD_CHGCMD:																								//������������Ӧ
			{
				if(szRecvBuf[3] == 0x01)
				{
					chgPowerSign = 2;																					//�����Ӧ�ɹ�
				}
			}
			break;
		case CMD_SENDVINCMD:																						//�״λ�ȡVIN������Ӧ
			{
				if(szRecvBuf[3] == 0x01)
				{
					fisrtGetVIN	 = 0;																					//��Ӧ�ɹ��󲻽����ϱ�
				}			
			}
			break;
		default: 
			ret = 0;
			break;
	}
	return ret;
}

uint8_t extSetPara(uint8_t check,uint8_t setParaId,const uint8_t* setBuff)
{
	uint8_t setlen = 0;
	char wktimes[50] = {0};
	switch(setParaId)
	{
		case PARA_WAKE_TIME:				//���û���ʱ��
			if(check == 1)
			{
				memset(wktimes,0,sizeof(wktimes));
				memcpy(wktimes,&setBuff[setlen],strlen((char*)&setBuff[setlen]));			
				sscanf(wktimes,"%d:%d:%d-%d:%d:%d",(int*)&gUserDara.wk_Year,(int*)&gUserDara.wk_Month,(int*)&gUserDara.wk_Day,(int*)&gUserDara.wk_Hour,(int*)&gUserDara.wk_Min,(int*)&gUserDara.wk_Sec);	
				
				/* ���û���ʱ��
				setWakeTime();	 */
			}
			setlen += strlen((char*)&setBuff[setlen]) + 1;
			break;
		case PARA_WAKE_INTERVAL:		//���û��Ѽ��
			if(check == 1)
			{
				memset(wktimes,0,sizeof(wktimes));
				memcpy(wktimes,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				
				sscanf(wktimes,"%ld",(long*)&gUserDara.wk_Interval);
				/* ���û���ʱ��
				setWakeTime(); */
			}
			setlen += strlen((char*)&setBuff[setlen]) + 1;
			break;
		case PARA_WAKE_SENDTIME:		//���û��Ѻ���������ʱ��
			if(check == 1)
			{
				gUserDara.wk_SendInterval = setBuff[setlen];
			}
			setlen ++;
			break;
		case PARA_AUTOCHG_VOLT:				//���Զ������ѹ
			{
				if(check == 1)
				{
					uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgVolt);			
					pFval[0] = setBuff[setlen+0];
					pFval[1] = setBuff[setlen+1];
					pFval[2] = setBuff[setlen+2];
					pFval[3] = setBuff[setlen+3];
				}
				setlen+=4;
			}
			break;
		case PARA_AUTOCHGOVER_VOLT:		//ֹͣ�Զ������ѹ
			{
				if(check == 1)
				{
					uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgOverVolt);			
					pFval[0] = setBuff[setlen+0];
					pFval[1] = setBuff[setlen+1];
					pFval[2] = setBuff[setlen+2];
					pFval[3] = setBuff[setlen+3];
				}
				setlen+=4;
			}
			break;
		case PARA_UNDERVOLT:					//Ƿѹֵ
			{
				if(check == 1)
				{
					uint8_t* pFval = (uint8_t*)(&gUserDara.underVolt);			
					pFval[0] = setBuff[setlen+0];
					pFval[1] = setBuff[setlen+1];
					pFval[2] = setBuff[setlen+2];
					pFval[3] = setBuff[setlen+3];
				}
				setlen+=4;
			}
			break;
		case PARA_HTTPURL:
			{
				if(check == 1)
				{
					memcpy(gUserDara.httpURL,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				}
				setlen += strlen((char*)&setBuff[setlen]) + 1;	
			}
			break;
		case PARA_HTTPUSERNAME:
			{
				if(check == 1)
				{
					memcpy(gUserDara.httpUserName,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				}
				setlen += strlen((char*)&setBuff[setlen]) + 1;	
			}
			break;
		case PARA_HTTPPASSWORD:
			{
				if(check == 1)
				{
					memcpy(gUserDara.httpPassWord,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				}
				setlen += strlen((char*)&setBuff[setlen]) + 1;	
			}
			break;
		case PARA_GETCANLOG:
			{
				if(check == 1)
					getCanLogSwitch = setBuff[setlen];
				setlen++;
			}
			break;
	}
	//�����û�����
	saveUserData();			
	return setlen;
}

/*
���ܣ��ն˹���ƽ̨��ȡ�ն˲�����Ϣ
������
*/
uint8_t extReadPara(uint8_t readParaId,uint8_t* readBuff)
{
	uint8_t readlen = 0;
	char wktimes[50] = {0};
	switch(readParaId)
	{
		case PARA_WAKE_TIME:				//��ѯ����ʱ��
			{
				memset(wktimes,0,sizeof(wktimes));
				sprintf(wktimes,"%d:%d:%d-%d:%d:%d",gUserDara.wk_Year,gUserDara.wk_Month,gUserDara.wk_Day,gUserDara.wk_Hour,gUserDara.wk_Min,gUserDara.wk_Sec);	
				memcpy(&readBuff[readlen],wktimes,strlen((char*)wktimes));
				readlen += strlen(wktimes);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_WAKE_INTERVAL:		//��ѯ���Ѽ��
			{
				memset(wktimes,0,sizeof(wktimes));
				sprintf(wktimes,"%d",gUserDara.wk_Interval);	
				memcpy(&readBuff[readlen],wktimes,strlen((char*)wktimes));
				readlen += strlen(wktimes);
				readBuff[readlen++] = 0;		
			}
			break;
		case PARA_WAKE_SENDTIME:		//��ѯ���Ѻ���������ʱ��
			readBuff[readlen++] = gUserDara.wk_SendInterval;	
			break;
		case PARA_AUTOCHG_VOLT:				//���Զ������ѹ
			{
				uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgVolt);						
				readBuff[readlen++] = pFval[0];
				readBuff[readlen++] = pFval[1];
				readBuff[readlen++] = pFval[2];
				readBuff[readlen++] = pFval[3];
			}
			break;
		case PARA_AUTOCHGOVER_VOLT:		//ֹͣ�Զ������ѹ
			{
				uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgOverVolt);			
				readBuff[readlen++] = pFval[0];
				readBuff[readlen++] = pFval[1];
				readBuff[readlen++] = pFval[2];
				readBuff[readlen++] = pFval[3];
			}
			break;
		case PARA_UNDERVOLT:					//Ƿѹֵ
			{
				uint8_t* pFval = (uint8_t*)(&gUserDara.underVolt);			
				readBuff[readlen++] = pFval[0];
				readBuff[readlen++] = pFval[1];
				readBuff[readlen++] = pFval[2];
				readBuff[readlen++] = pFval[3];
			}
			break;
		case PARA_HTTPURL:						//httpURL
			{
				memcpy(&readBuff[readlen],gUserDara.httpURL,strlen(gUserDara.httpURL));
				readlen += strlen(gUserDara.httpURL);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_HTTPUSERNAME:				//httpUserName
			{
				memcpy(&readBuff[readlen],gUserDara.httpUserName,strlen(gUserDara.httpUserName));
				readlen += strlen(gUserDara.httpUserName);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_HTTPPASSWORD:				//httpPassWord
			{
				memcpy(&readBuff[readlen],gUserDara.httpPassWord,strlen(gUserDara.httpPassWord));
				readlen += strlen(gUserDara.httpPassWord);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_GETCANLOG:
			readBuff[readlen++] = getCanLogSwitch;	
			break;
	}	
	return readlen;
}

/*
���ܣ���չʵʱ����
������ʵʱ���������Զ�������
*/
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	GBSTA *pSta = obj;
	uint16_t index = 0;
	if(pSta->bLink == 0)
	{
		index += Pack0AData(pRealData,&bPackBuf[index]);
	}
	return index;
}

/*
���ܣ���������ʱ��ȡVIN
��������VINΪ����ʶ���룬�����ն˱�Ų��㣬�����豸��ʱ������Ӧ����Ϣ��
*/
uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
	
	return 0;
}
 
/* 
���ܣ����ݰ�ͷ
������������ݱ�ͷ��У��,����ֵ 
*/
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
���ܣ�����Զ��� 0x0A����
������
*/
static uint16_t Pack0AData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	uint8_t temp8 = 0,ipLen = 0;
	uint16_t temp16 = 0;
	
	bPackBuf[0] = 0x0A;
	SelfData0A*  p0AData = (SelfData0A*)&pRealData->externData[p0Aoffset];		
	
	//����Զ�������
	bPackBuf[index++] = 0x03;																							//�豸�ͺű��  ԭ������
	bPackBuf[index++] = (gTerminalState.pwrVolt > 3 ? 0:1);								//�豸�����ʶ  1 ��ʾ�ڲ���ع��磬0��ʾ���ⲿ��ع���
	
	memcpy(&bPackBuf[index],&getCarType()[strlen(getCarType())-5],5);			//TBOX�̼��汾��
	bPackBuf[index+2] = '0';
	index+=5;
	
	bPackBuf[index++] = 0xFF;//p0AData->sRelayST;													//�̵���״̬
	bPackBuf[index++] = 0xFF;//p0AData->sTermBindingST;										//�ն˰�״̬
	bPackBuf[index++] = p0AData->sVCUVerST;																//�����������汾״̬
	bPackBuf[index++] = p0AData->sLockCarST;															//����״̬��ָ�ն˽���ƽ̨������ָ�
	
	if(chgPower != 0xFFFF || chgPower != 0xFFFE)
		temp16 = chgPower*100;
	bPackBuf[index++] = (uint8_t)(temp16>>8);
	bPackBuf[index++] = (uint8_t)(temp16>>0);															//���γ�����
		
	memset(&bPackBuf[index],0xFF,8);																			//VCU�汾�� p0AData->sVCUVerNum	
	index += 8;

	bPackBuf[index++] = p0AData->sQorS_CHGSign;														//�����������־
	bPackBuf[index++] = p0AData->sLockCarSign;														//������־(ʵ��������ǰִ�е�����ָ��)
	bPackBuf[index++] = 0;//p0AData->sFaultCT;														//������
	bPackBuf[index++] = (uint8_t)(p0AData->sCarType>>8);									//����ʶ��	
	bPackBuf[index++] = (uint8_t)(p0AData->sCarType>>0);	

	bPackBuf[index++] = p0AData->sTboxST;																	//Tbox����״̬
	bPackBuf[index++] = (BSP_Iostatus(IN_ACC_ID) == ON);									//����ON�ź�	(ACC)			0�����źţ� 1�����ź�
	bPackBuf[index++] = (gRealData.chargeState == STOP_CHARGE || gRealData.chargeState == CHARGE_FINISH)?1:0;																//����CHARGE�ź�
	bPackBuf[index++] = (fun_can_Get_State(BSP_CAN)>0 ? 1:0);							//Tbox CAN ����״̬
	bPackBuf[index++] = (fun_can_Get_State(BSP_CAN)>0 ? 1:0);							//Tbox CAN0-3 ״̬��0�������ݣ�1�������ݣ�
	
	if(gTerminalState.pwrVolt != 0xFFFF && gTerminalState.pwrVolt != 0xFFFE)
			temp16 = (uint16_t)(gTerminalState.pwrVolt*1000);		
	bPackBuf[index++] = (uint8_t)(temp16>>8);																//����С��ƿ��ѹ		
	bPackBuf[index++] = (uint8_t)(temp16>>0);
	
	if(gTerminalState.batVolt != 0xFFFF && gTerminalState.batVolt != 0xFFFE)
			temp16 = (uint16_t)(gTerminalState.batVolt*1000);	
	bPackBuf[index++] = (uint8_t)(temp16>>8);																										//Tbox�ڲ���ص�ѹ
	bPackBuf[index++] = (uint8_t)(temp16>>0);	
	
	bPackBuf[index++] = 0;	
	memset(&bPackBuf[index],0,60);
	ipLen += sprintf((char*)&bPackBuf[index],"%s:%d",gSysPara.domain[1],gSysPara.port[1]);			//˫��·IP�˿�	
	bPackBuf[index - 1] = ipLen;
	index += ipLen;
	
	bPackBuf[index++] = ((gSysPara.linkSwitch & 0x02)>0 ? 1:0);																	//˫��·״̬
	bPackBuf[index++] = (gUserDara.maintCMD>=0x55 && gUserDara.maintCMD <= 0x5C ? 1:0);					//������ʾ
	bPackBuf[index++] = (gUserDara.payCMD>=0x55 && gUserDara.payCMD <= 0x5D ? 1:0);							//�ɷ���ʾ
	bPackBuf[index++] = (gUserDara.yearCheckCMD>=0x55 && gUserDara.yearCheckCMD <= 0x58 ? 1:0);	//������ʾ	
	
	bPackBuf[index++] = Fun_Gprs_Csq();																													//TBOX �����ź�

	if(g_tGPS.antSta == 1)
		temp8 = 0;		//����
	else if(g_tGPS.antSta == 2)
		temp8 = 2;		//��·
	else if(g_tGPS.antSta == 3 || isLocFault == 1)
		temp8 = 1;		//��·
	bPackBuf[index++] = temp8;																						//Tbox GPS ����״̬
	
	bPackBuf[index++] = (bsp_storage_state() > 0 ? 0:1);									//Tbox EMMC ����״̬  0������  1���쳣
	bPackBuf[index++] = 0xFF;//(uint8_t)(p0AData->sAirCurr>>8);						//�յ���������
	bPackBuf[index++] = 0xFF;//(uint8_t)(p0AData->sAirCurr>>0);	

	bPackBuf[index++] = 0xFF;//p0AData->sPumpFault;												//ˮ�ù���	
	bPackBuf[index++] = 0xFF;//p0AData->sVacuumPumpST;										//��ձ�״̬
	bPackBuf[index++] = 0xFF;//p0AData->sVacuumValue;											//��ն�
	
	bPackBuf[index++] = 0xFF;//p0AData->sPTCST;														//PTC ����״̬
	bPackBuf[index++] = 0xFF;//p0AData->sPTCRelayST;											//PTC�̵���ʵ��״̬
	bPackBuf[index++] = 0xFF;//p0AData->sBatRelayST;											//������ؼ��ȼ̵���״̬
	bPackBuf[index++] = 0xFF;//p0AData->sQ_CHG_Fault;											//���̵���ճ������
	bPackBuf[index++] = 0xFF;//p0AData->sAirWorkST;												//�յ�����״̬
	bPackBuf[index++] = 0xFF;//p0AData->sBreakST;													//��ɲ״̬

	//��䳤��
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}


/* 
���ܣ����������
�������ճ��ʱ��¼�����ʼʱ�䣬���������¼������ʱ�䡣
			(volt*(10000.0-curr)*t_ms)/360000000000.0 (1000��/3600ʱ/1000ǧ��/100���ӷŴ�ϵ��);
			��������
*/ 
void calculCHGPower(void)
{
	//��ʼ���
	if(chgStaST == 0 && gRealData.chargeState == STOP_CHARGE && gRealData.soc < 100)					
	{
		chgStaST = 1;
		chgPowerSign = 0;																					//����� �ϱ�����
		chgPower = 0;																							//���������
		chgStartTime = osKernelGetTickCount();										//���ʱ������
		memcpy(&chgStartTimes,&g_system_dt,sizeof(g_system_dt));	//��¼�����ʼʱ��
	}
	if(chgStaST == 1)
	{
		if(osKernelGetTickCount() - chgStartTime > 500)							//������
		{
			//�������
			if(gRealData.chargeState == CHARGE_FINISH ||(gRealData.chargeState == STOP_CHARGE && gRealData.soc == 100))
			{
				chgPowerSign = 1;		
				chgStaST = 0;
				memcpy(&chgEndTimes,&g_system_dt,sizeof(g_system_dt));
			}
			
			//���δ���� - ���������ۼӺ�
			chgPower += (gRealData.total_volt * (-gRealData.total_current))*(osKernelGetTickCount() - chgStartTime)/1000/1000/3600;			
			//chgPower += (gRealData.total_volt * (-gRealData.total_current))/1000/3600;			
			chgStartTime = osKernelGetTickCount();
		
		}
	}	
}

/* 
���ܣ����������
������
		ÿ�γ����ɺ󣬵ǳ�ǰ����һ�γ�������
		������������ʱ�����´���������ʱ����
*/
static uint16_t packCHGPower(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen)
{
	uint16_t valueTemp = 0;
	uint8_t index = 26;	
	
	buff[index++] = (uint8_t)(chgStartTimes.year - 2000);
	buff[index++] = (uint8_t)(chgStartTimes.month);
	buff[index++] = (uint8_t)(chgStartTimes.day);
	buff[index++] = (uint8_t)(chgStartTimes.hour);
	buff[index++] = (uint8_t)(chgStartTimes.minute);
	buff[index++] = (uint8_t)(chgStartTimes.second);
	
	buff[index++] = (uint8_t)(chgEndTimes.year - 2000);
	buff[index++] = (uint8_t)(chgEndTimes.month);
	buff[index++] = (uint8_t)(chgEndTimes.day);
	buff[index++] = (uint8_t)(chgEndTimes.hour);
	buff[index++] = (uint8_t)(chgEndTimes.minute);
	buff[index++] = (uint8_t)(chgEndTimes.second);

	if(chgPower != 0xFFFF || chgPower != 0xFFFE)
		valueTemp = chgPower*100;
	buff[index++] = (uint8_t)(valueTemp>>8);
	buff[index++] = (uint8_t)(valueTemp>>0);

	buff[24] = (index-26)>>8;
	buff[25] = (index-26)>>0;
	
	return MakeCmd(CMD_CHGCMD,0xFE,vin,buff,index-24);
}

/* 
���ܣ����VIN�����Ϣ
������
*/
static uint16_t packUpdataVIN(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen)
{	
		uint32_t dwVal = 0;		
		uint8_t index = 26;	
		buff[index++] = 0x03;								//Tbox�豸�ͺ�
		buff[index++] = 0x00;								//��������
		buff[index++] = 0x03;								//��������
		
		//�״�ʱ��
		buff[index++] = (uint8_t)(getVINTime.year - 2000);
		buff[index++] = (uint8_t)(getVINTime.month);
		buff[index++] = (uint8_t)(getVINTime.day);
		buff[index++] = (uint8_t)(getVINTime.hour);
		buff[index++] = (uint8_t)(getVINTime.minute);
		buff[index++] = (uint8_t)(getVINTime.second);	
		//�״ξ���
		dwVal = (uint32_t)(glongd * 1000000);    
		buff[index++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		buff[index++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		buff[index++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		buff[index++] = (uint8_t)dwVal;
		//�״�γ��
		dwVal = (uint32_t)(glatd * 1000000);  
		buff[index++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		buff[index++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		buff[index++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		buff[index++] = (uint8_t)dwVal;
		//ICCID
		Fun_Gprs_getICCID((char*)&buff[index],20);
		index += 20;

		memcpy(&buff[index],realVin,17);
		index += 17;
		
		buff[24] = (index-26)>>8;
		buff[25] = (index-26)>>0;
	
	return MakeCmd(CMD_SENDVINCMD,0xFE,vin,buff,index-24);
}

/*
���ܣ��ն˵͵籣������
������
			ȫ�������ڼ���ѹ�����Ƿѹ�󣬹رն�ʱ���ѹ���
			����Ӳ������״̬��
*/
static uint32_t printTime = 0;
static void lowVoltProtect(void)
{	
	if(autoChgState == 0) 											//�Զ�����׶β����ж�
	{
		if(gTerminalState.pwrVolt <= 5 )
		{
			pSelfData0A->sTboxST = 4;						//�Ͽ�
		}
		else if(gTerminalState.pwrVolt <= gUserDara.underVolt - 0.2f && gTerminalState.pwrVolt >5)
		{
			pSelfData0A->sTboxST = 3;						//Ƿѹ
		}
		else if(!(fun_can_Get_State(BSP_CAN) > 0 && (gRealData.carState == 1 || gRealData.chargeState == 1)))		
		{
			pSelfData0A->sTboxST = 2;						//����
		}
		else
		{
			pSelfData0A->sTboxST = 1;						//����
		}
	}
	//��ӡǷѹ״̬������
	if(pSelfData0A->sTboxST >= 2 && osKernelGetTickCount() - printTime>10000)
	{
		char tempPrintfBuff[50] = {0};
		printTime = osKernelGetTickCount();
		sprintf(tempPrintfBuff,"MainVolt = %f,break:4,low:3, result=%d\r\n",gTerminalState.pwrVolt,pSelfData0A->sTboxST);
		printfData(tempPrintfBuff);
	}
	
	//�ϵ��Ƿѹ
	if(pSelfData0A->sTboxST == 3||pSelfData0A->sTboxST == 4)
	{
		set_WakeTime2(0);															//�رն�ʱ���� ����Ӳ������
	}
	else
	{
		set_WakeTime2(1);															//Ƿѹ�������ָ��Զ�����	
	}										

}

/*
���ܣ��Զ��������ſ���
������ֻ��˯�߻����п����Զ����繦�ܣ�������������У��ն�һֱ��Ӳ�߻����źţ�
*/
#define noCANDataTime 	 						60000						//��Ͳ���ʱ�� 2��
#define autoCHgLowTime 	 						1800000					//��Ͳ���ʱ�� 30��
#define autoCHgHighTime 						10800000				//��߲���ʱ�� 3Сʱ
static uint32_t autoChgTotalTime = 0;								//������ʱ��
static float autoChgPreVolt = 0;										//������ʼ��ѹ
static void autoChgPower(void)
{
		uint8_t isStartAutoChgSign = 0;			

		//���������ж�   ���緶Χ������Ƿѹֵ С�ڵ��ڲ���ֵ
		if(autoChgState == 0 && gTerminalState.pwrVolt <= gUserDara.autoChgVolt-0.2f && gTerminalState.pwrVolt > gUserDara.underVolt)
		{
			printfData("start autoChg\r\n");
			autoChgState = 1;																			//�����߼�����
			autoChgTotalTime = osKernelGetTickCount();						//��ʼ��ʱ��
			autoChgPreVolt = gTerminalState.pwrVolt;							//��ʼ���ѹ
		}	
		
		/* �����ж� */
		if(autoChgState == 1)
		{
			//�����������1��������ɣ�������ɵ�ѹ && ���糬ʱ30�֣�
			if(gTerminalState.pwrVolt >= gUserDara.autoChgOverVolt && osKernelGetTickCount()-autoChgTotalTime >= autoCHgLowTime)
			{
				isStartAutoChgSign = 0;									
			}
			//�����������2�����糬ʱ��С����ɵ�ѹ && ��ʱ3Сʱ��
			else if(gTerminalState.pwrVolt < gUserDara.autoChgOverVolt && osKernelGetTickCount()-autoChgTotalTime >= autoCHgHighTime)
			{
				isStartAutoChgSign = 0;									
			}
			//�����������3�������쳣������10s�󣬵�ѹ����������
			else if(osKernelGetTickCount()-autoChgTotalTime > 10000 && autoChgPreVolt > gTerminalState.pwrVolt+1)		
			{
				isStartAutoChgSign = 0;									
			}
			//�����������4�������쳣����������3���Ӻ�����������CAN��
			else if(osKernelGetTickCount()-autoChgTotalTime >= noCANDataTime && fun_can_Get_State(BSP_CAN) == 0)
			{
				isStartAutoChgSign = 0;			
			}
			//��������
			else
			{
				isStartAutoChgSign = 1;			
			}
		
			/* ����ָ��� */		
			if(osKernelGetTickCount() - autoChgTotalTime < 3000 || sAllowAutoChg == 1)
			{
				if(isStartAutoChgSign == 1)
				{
					BSP_Ioctl(AUTO_CHG_ID,ON);			
					if(osKernelGetTickCount() - autoChgTotalTime < 3000)
						sendIntervalData(1);					//ǿ�Ʒ�3s		ǰ3s�������Ͳ�������   
				}
				else 
				{
					autoChgState = 0;								//������������  ���������������
					sAllowAutoChg = 0;
					BSP_Ioctl(AUTO_CHG_ID,OFF);
					printfData("autoStop autoChg\r\n");
				}			
			}
			else																//������������  3s��  VCU��������
			{
				if(isStartAutoChgSign == 1)
				{					
					autoChgState = 0;
					isStartAutoChgSign = 0;
					BSP_Ioctl(AUTO_CHG_ID,OFF);
					printfData("forceStop autoChg\r\n");					
				}
			}
		}		
}

/*
���ܣ���λ�쳣����
��������Ч��λ����50kmʱ��Ϊ��λ�쳣����
*/
static void localtionFaultFun(void)
{
	uint8_t tempMile = 0;
	if(osKernelGetTickCount()>180000)					//ÿ�ο����������ӿ�ʼ���
	{
		if(gRealData.locationState == 1)				//��Ч��λ����¼�޶�λʱ��� ������
		{
			if(gUserDara.isSiteExcute == 0)
			{
				gUserDara.isSiteExcute = 1;
				gUserDara.outSiteMile = gRealData.totalMileage;
				saveUserData();											
			}
			
			tempMile = gRealData.totalMileage - gUserDara.outSiteMile;
			if(tempMile >= 50)
			{
				//��λ�쳣;
				isLocFault = 1;
			}
		}
		else if(gRealData.locationState == 0)		//��Ч��λ
		{
				if(gUserDara.isSiteExcute != 0)
				{
					gUserDara.isSiteExcute = 0;
					isLocFault = 0;
					saveUserData();			
				}
		}
	}
}

/*
���ܣ��Զ����磬Ƿѹ���ĵ�ѹ��ʼ��
������
*/
static void AutoVoltInit()
{
	uint32_t temp = 0;
	/* ����Ĭ��ֵ */
	if(gTerminalState.pwrVolt>22)				//24V ��
	{
		temp = (uint32_t)gUserDara.autoChgVolt;
		if(!(temp > 0 && temp < 36))									//�貹���ѹ
		{
			gUserDara.autoChgVolt = 24;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.autoChgOverVolt;
		if(!(temp > 0 && temp < 36))									//ֹͣ�����ѹ
		{
			gUserDara.autoChgOverVolt = 25;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.underVolt;
		if(!(temp > 0 && temp < 36))									//Ƿѹ��ѹ
		{
			gUserDara.underVolt = 23;
			isSetFirstV = 2;
		}
	}
	else if(gTerminalState.pwrVolt<18)	//12V ��
	{
		temp = (uint32_t)gUserDara.autoChgVolt;
		if(!(temp > 0 && temp < 36))									//�貹���ѹ
		{
			gUserDara.autoChgVolt = 12;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.autoChgOverVolt;
		if(!(temp > 0 && temp < 36))									//ֹͣ�����ѹ
		{
			gUserDara.autoChgOverVolt = 13.8f;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.underVolt;
		if(!(temp > 0 && temp < 36))									//Ƿѹ��ѹ
		{
			gUserDara.underVolt = 11;
			isSetFirstV = 2;
		}
	}
	if(isSetFirstV == 2)
	{
		saveUserData();
		isSetFirstV = 3;
	}

	bsp_sleep_wakeup_volt(gUserDara.underVolt);
}

/*
���ܣ���������/����ָ��
������
*/
static uint8_t UnpackLockCMD(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.lockCMD = rbuf[24];												//��������ָ��
	switch(rbuf[24])
	{
		case 0x55:gUserDara.runCMD = 1;																						//��ֹ��Ӫ
			break;
		case 0x56:gUserDara.runCMD = 1;																						
			break;
		case 0x57:gUserDara.runCMD = 1;																						
			break;
		case 0x60:gUserDara.runCMD = 1;																						
			break;
		default:gUserDara.runCMD = 0;
			break;
	}
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_LOCKCMD,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
���ܣ�����汾�л�����
������0x55�������汾��0xAA���������汾
*/
static uint8_t UnpackSoftVer_Change(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.changeCMD = rbuf[24];										//�汾�л�����ָ��

	saveUserData();
	DataAreaLen = MakeCmd(CMD_SOFTVER_CHANGE,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
���ܣ��󶨿���
������0x55��ִ�а󶨣�0x00��Ĭ��
*/
static uint8_t UnpackBindingCtl(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.bindingCMD = rbuf[24];									//�󶨿���ָ��

	saveUserData();
	DataAreaLen = MakeCmd(CMD_BINDINGCTR,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
���ܣ����ſ���
����������:0x55, ����:0xAA,
*/
static uint8_t UnpackDoorCtl(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.doorCMD = rbuf[24];											//���ſ���ָ��
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_DOORCTR,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
���ܣ�������ʾ
������
*/
static uint8_t UnpackMaintTips(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.maintCMD = rbuf[24];										//���ſ���ָ��
		
	saveUserData();
	DataAreaLen = MakeCmd(CMD_MAINTTIPS,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
���ܣ��ɷ���ʾ
������
*/
static uint8_t UnpackPayTips(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.payCMD = rbuf[24];											//���ſ���ָ��
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_PAYTIPS,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
���ܣ�������ʾ
������
*/
static uint8_t UnpackYearCheckTips(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.yearCheckCMD = rbuf[24];								//���ſ���ָ��																	
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_CHECKTIPS,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}



