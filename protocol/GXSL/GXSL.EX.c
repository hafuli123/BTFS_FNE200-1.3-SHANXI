#include "analyse_Bms_GXSL.h"

#define MAX_YZT_LINK 1
static uint16_t MakeCmd(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *wbuf,uint16_t dataLen);
extern uint16_t packUpdataVIN(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen);					//�״λ�ȡVIN �ϱ�
static uint8_t sendNetDatas(uint8_t socket_fd, uint8_t *buf, uint16_t len);

extern char realVin[18];
uint8_t isChgVIN = 0;

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


/* �û��Զ������ */
static GBSTA gbSta[MAX_YZT_LINK] = {0};
USER_DATA gUserDara;												//�û����ݴ洢
SelfData0A* pSelfData0A;										//������Դ������ �Զ�������				
static uint16_t p0Aoffset;									//0Aָ��洢λ��ƫ�� 

/* �������Ʋ��� */
static uint32_t reConnectStamp = 0;					//�ز���ʱ
static uint8_t printPackCmd = 1;						//��ӡ���ĵ����ڿ���λ

/* ���߻��Ѳ��� */
static int wakelock = -1;										//������
static uint64_t wakeTime = 0;								//����ʱ��
static uint32_t wk_timesss = 0;							//���ѵ���ʱ
uint8_t wk_Excute = 0;											//���ѱ�־����
uint8_t allow_UserOnLine = 0;								//�����û�ƽ̨����

/* VIN������� */
RTC_INFOR getVINTime;												//�״λ�ȡVIN��ʱ�� 
double glongd;									            //����
double glatd;									              //γ��
uint8_t fisrtGetVIN = 0;										//�Ƿ�ʱ�״λ�ȡVIN 1�ǣ�0�� 
uint8_t getCanLogSwitch = 0;								//Զ��CAN��־�ɼ�����
static uint32_t reportVINStamp = 0;					//�ϱ��״λ�ȡVIN��ʱ
static uint32_t reportVINInterval = 10000;				//Ĭ�����ͼ����10s
uint8_t reportVINfaildCnt = 0;										//����VINʧ�ܴ���




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
		}
		else
		{
			//�ǳ������� VIN�����Ŀ��Ʋ���
			isChgVIN = 0;
			reportVINfaildCnt = 0;
			reportVINInterval = 10000;
		}
			return 1;
	}
