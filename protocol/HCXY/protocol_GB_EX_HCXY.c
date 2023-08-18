/*
�� ����protocol_GB_EX_HCXY.c
�� �ܣ���ʱ���ѿ��Ƽ�����߼�
�� ��: 2022/2/17
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: LGC
*/

#include "HCXY/protocol_GB_EX_HCXY.h"

enum PARA_ID{
	PARA_WAKE_TIME = 0x9C,						//���û���ʱ��
	PARA_WAKE_INTERVAL = 0x9D,				//���û��Ѽ��
	PARA_WAKE_SENDTIME = 0x9E,				//���û��Ѻ���������ʱ�䳤��
};

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

#define MAX_EXT_LINK 1

/* �û��Զ������ */
static GBSTA gbSta[MAX_EXT_LINK] = {0};
USER_DATA gUserDara;												//�û����ݴ洢

/* �������Ʋ��� */
static uint32_t reConnectStamp = 0;					//�ز���ʱ
static uint8_t printPackCmd = 1;						//��ӡ���ĵ����ڿ���λ

/* ���߻��Ѳ��� */
static int wakelock = -1;										//������
static uint64_t wakeTime = 0;								//����ʱ��
static uint32_t wk_DownTime = 0;						//���ѵ���ʱ
uint8_t wk_Excute = 0;											//���ѱ�־����
uint8_t allow_UserOnLine = 0;								//�����û�ƽ̨����
uint8_t allow_FtpUpdata = 0;								//����FTP����

/******************************************************
���ܣ��洢�û�����
�������洢����ָ���������
*******************************************************/
void saveUserData(void)
{
		gUserDara.store_flag = 0xAA;
		User_ID_Save(&gUserDara,sizeof(USER_DATA));				
}

/******************************************************
���ܣ��������ݵ�����
������
*******************************************************/
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

/***********************���ѹ���****************************/

/******************************************************
���ܣ����ѻص�����
������	
	����1 ���²�����˯�ߣ������ߵ���ʱ���ٽ�������
*******************************************************/
static uint8_t wake_RT_Callback(uint8_t type)		
{
	wk_Excute = 1;
	bsp_wakelock_lock(wakelock);
	//allow_UserOnLine = 1;
	allow_FtpUpdata = 0;
	wk_DownTime = osKernelGetTickCount();
	return 1;
}
/******************************************************
���ܣ����Ѻ��ٴ����ߵ���ʱ
������	
		
*******************************************************/
static void wake_RT_OnLineTime(void)
{
	if(wk_Excute)		
	{	
		if(osKernelGetTickCount() - wk_DownTime > (gUserDara.wk_SendInterval-3)*10000)
		{
			allow_FtpUpdata = 1;	
		}
		
		//�����ϱ���ʱ || �л���Դ
		if(osKernelGetTickCount() - wk_DownTime > gUserDara.wk_SendInterval*10000)
		{
			//���������
			if(bsp_wakelock_unlock(wakelock) == BSP_SLEEP_SUCCESS)
			{
				wk_Excute = 0;
				allow_UserOnLine = 0;											
			}
			printfData("wk_lineTime over\r\n");
		}
	}
}

/******************************************************
���ܣ���ʱ����ʱ���������
������
		 ���ö�ʱ������ʼʱ�䣬�붨ʱ����
			swID 1�����ö�ʱ���ѣ�0�����ö�ʱ����
*******************************************************/
static void wake_RT_SetTime(uint8_t swID)							
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
			
			bsp_wakelock_para(wakelock,wakeTime,600,wake_RT_Callback,0,0);												/* ����10���� */
		}
		else 								//�رն�ʱ����
		{
			bsp_wakelock_para(wakelock,0,0,0,0,0);				//�رն�ʱ���� ����Ӳ������
		}
	}
}

/******************************************************
���ܣ���ʱ�����������
������
			���ƶ�ʱ��������
*******************************************************/
static void wake_RT_MainFun(void)
{	
		if(getFTPModeState() == 1)					//������ָ��رն�ʱ���� ����Ӳ������		
		{
			wake_RT_SetTime(1);															
		}
		else
		{
			wake_RT_SetTime(0);								//������ָ��ָ���ʱ���� �����������
		}

}

/***********************���ܿ��Ʋ���****************************/

/******************************************************
���ܣ����� Զ������ָ��ִ��
�������ն�������������ǰ������Ӱ�죬�����ն˽������� 10���Ӻ��ٻ�������
		1������ 0��������
*******************************************************/
uint8_t contrlFTPModeExcute(void)
{
 return allow_FtpUpdata;
}

/******************************************************
���ܣ����ƽ̨���������ϱ�
��������ʱ���Ѻ���Ҫ����������
	0�����������ƽ̨�������� 1���������ƽ̨��������
*******************************************************/
uint8_t extCtrlNetSta(void* obj)
{
	return 0;
}

/******************************************************
���ܣ����ƻ������ݱ��Ĵ洢
��������ʱ���Ѻ���Ҫ����������
	0:���洢��ʱ���Ѻ��ϱ�ƽ̨��ʵʱ�������� 1���洢��ʱ���Ѻ��ϱ�ƽ̨��ʵʱ��������
*******************************************************/
uint8_t weakCtrlNetSta(void)
{
	return 0;
}



/********************�ն˹���ƽ̨��չ����************************/
/******************************************************
���ܣ��ն˹���ƽ̨�����ն˲�����Ϣ
*******************************************************/
uint8_t extSetPara(uint8_t check,uint8_t setParaId,const uint8_t* setBuff)
{
	return 0;
}

/******************************************************
���ܣ��ն˹���ƽ̨��ȡ�ն˲�����Ϣ
*******************************************************/
uint8_t extReadPara(uint8_t readParaId,uint8_t* readBuff)
{
	return 0;
}

/*********************����32960��չЭ��*************************/
/******************************************************
���ܣ���չЭ���ʼ��
��������������״̬����
*******************************************************/
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	
	/* ��ȡ�û����� */
	if(gUserDara.store_flag != 0xAA)
	{
		User_ID_Read(&gUserDara,sizeof(USER_DATA));		
		gUserDara.store_flag = 0xAA;		
	}
	
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
	//�ڲ�
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].chgPowerStamp = 0;
	
	reConnectStamp = 0;
	
	wakelock = bsp_wakelock_create();				/* ���������� */	
	
	return &gbSta[objLinkIdx];
}

/******************************************************
���ܣ���չЭ������   
������	
			ctrl:0 �������� 1:�������� 2:˯�� 
			sta:0 ����δ���� 1:�����ѵ��� 
����ֵ:
			0 ��չЭ��δ���� 1:��չЭ���ѵ���
*******************************************************/
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	if(obj == NULL)
		return 1;
	uint16_t dataLen = 0;
	wake_RT_MainFun();																//��ʱ��������Fun

	if(pSta->bLink == 0)
	{
		wake_RT_OnLineTime();														//��ʱ���� ����ʱ

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
		}
		else
		{
			
		}
	}
	return 1;
}
/******************************************************
���ܣ���չЭ����ս��
������
*******************************************************/
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	return 0;
}

/******************************************************
���ܣ���չʵʱ����		
������ʵʱ���������Զ�������
*******************************************************/
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	return 0;
}
