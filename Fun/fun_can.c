#include "fun_can.h"
#include "cmsis_os2.h"
#include "string.h"

#define CAN_TIME_OUT	2000																													//CAN�Ͽ��жϳ�ʱʱ��

static void CAN1_Init(void);
static void CAN2_Init(void);
static void CAN3_Init(void);

static uint8_t useCan = 0;
static	uint32_t CanGetCnt[3] = {0};																								//CAN���ձ�������
static	uint32_t CanTimeOutCnt[3] = {-CAN_TIME_OUT,-CAN_TIME_OUT,-CAN_TIME_OUT};		//CAN��ʱ����

void funCanInit(void)
{
	useCan = 0;
	iniCanData();													/* ��ʼ��CAN���� */
	CAN1_Init();
	CAN2_Init();
	CAN3_Init();
}

/* BMS��ʼ�� */
void CAN1_Init(void)
{
	if(1 == gSysPara.can1_used)
	{
		useCan = 1;
		/* CAN��ʼ�� */
		if(CAN_init (1, gSysPara.can1_baudrate) == CAN_OK)
			CAN_start (1);
	}
}

/* BMS��ʼ�� */
void CAN2_Init(void)
{
	if(1 == gSysPara.can2_used)
	{
		useCan = 1;
		/* CAN��ʼ�� */
		if(CAN_init (2, gSysPara.can2_baudrate) == CAN_OK)
			CAN_start (2);
	}
}

/* BMS��ʼ�� */
void CAN3_Init(void)
{
	if(1 == gSysPara.can3_used)
	{
		useCan = 1;
		/* CAN��ʼ�� */
		if(CAN_init (3, gSysPara.can3_baudrate) == CAN_OK)
			CAN_start (3);
	}
}
/* BMS���������� */
void funCanrun(void)
{
	CAN_msg msg_rece;
	if(useCan == 0)
	{
		osDelay(100);
		return;
	}
	/* CAN���ݽ��� */
	if (CAN_receive (&msg_rece,1) == CAN_OK)
	{
		if(msg_rece.ch >= 1 && msg_rece.ch <= 3)
		{
			if((msg_rece.id & 0xFFFFFFF0) == 0x1FFFFFF0)
			{
				msg_rece.id &= 0xFFFFFFEF;
				osDelay(5);
				CAN_send(msg_rece.ch,&msg_rece, 0);
			}
			else if((msg_rece.id & 0xFFFFFFF0) == 0x1FFFFFE0)
			{
				CanTimeOutCnt[msg_rece.ch-1] = osKernelGetTickCount();			
				if((msg_rece.id & 0x0F) == msg_rece.ch - 1)
				{
					CanGetCnt[msg_rece.ch - 1]++;
				}
			}
			else
			{
				CanTimeOutCnt[msg_rece.ch-1] = osKernelGetTickCount();
				CanGetCnt[msg_rece.ch-1]++;
			}
			unpackCAN(msg_rece.ch,&msg_rece);
		}
	}
	udsProc();
}

uint32_t calcCanValue(uint8_t nstartPos,uint8_t nlen,const uint8_t* pcanVal,BYTE_MODEL hightLowMode,CANANA_MODEL canMode)
{
	uint8_t nowPosByte = nstartPos/8;
	uint8_t posBit = nstartPos%8;
	uint8_t byteCount = nlen/8 + (((nlen%8)!=0)?1:0);//һ���ֽڳ��Ȳ���8��Ҳ��1���ֽ�
  uint32_t realVal = 0;
  uint8_t szDataTmp[8] = {0};
	int i; 
	
	memcpy(szDataTmp,pcanVal,8);
	
	if(HIGHTLOW == hightLowMode)	//�ߵ�λģʽ������Ҫת���ɵ͸�λģʽ
	{
		if(byteCount > 1)
		{
			if(INTEL == canMode)//Intel�ı����ʽ							
			{
				for(i=0;i<byteCount;++i)
				{
					szDataTmp[nowPosByte + i] = pcanVal[nowPosByte + (byteCount-1) - i];
				}
			}
			else if(MOTOROLA == canMode)//Motorola�ı����ʽ		
			{
				for(i=0;i<byteCount;++i)
				{
					szDataTmp[nowPosByte - i] = pcanVal[nowPosByte - (byteCount-1) + i];
				}
			}
		}		
	}
	
	for(i=0;i<nlen;++i)
	{
		if((posBit)>7)
		{
			if(INTEL == canMode)
      	++nowPosByte;
      else
      	--nowPosByte;

			posBit=0;
		}
		if((szDataTmp[nowPosByte]&(1<<posBit)) > 0)
		{
			realVal += (1<< i) ;
		}
		++posBit;
	}
	return realVal;
}

float calcRealValue(uint8_t nstartPos,uint8_t nlen,float factor,float offset,CALC_MODEL calcModel,
														const uint8_t* pcanVal,BYTE_MODEL hightLowMode,CANANA_MODEL canMode)
{
	float fval = 0.0;
	uint32_t n32Val = calcCanValue(nstartPos,nlen,pcanVal,hightLowMode,canMode);
	
	if(8 == nlen && 0xFF == n32Val)
		fval = 0xFF;
	else if(16 == nlen && 0xFFFF == n32Val)
		fval = 0xFFFF;
	else if(24 == nlen && 0xFFFFFF == n32Val)
		fval = 0xFFFFFF;
	else if(32 == nlen && 0xFFFFFFFF == n32Val)
		fval = 0xFFFFFFFF;
	else
	{
		if(FACTOR_OFFSET == calcModel)
			fval = ((float)n32Val *  factor) + offset;
		else
			fval = ((float)n32Val +  offset) * factor;
	}
	
	return fval;
}

//���¹�����
void updateFaultCode(ROTACODE* roataCode,uint8_t maxCnt,uint32_t newCode,uint32_t timeout)
{
	uint8_t i,j;
	for(i = 0;i < maxCnt;i++)
	{
		//�����볬ʱȡ��,����������ǰ��
		if(roataCode[i].code != 0 && osKernelGetTickCount() - roataCode[i].timeStamp > timeout)
		{
			roataCode[i].code = 0;
			for(j = i; j < maxCnt - 1;j++)
			{
				roataCode[j].code = roataCode[j + 1].code;
				roataCode[j].timeStamp = roataCode[j + 1].timeStamp;
			}
		}
		//�ɹ�������¹�����
		if((roataCode[i].code == newCode || roataCode[i].code == 0) && newCode != 0)
		{
			newCode = 0;
			roataCode[i].code = newCode;
			roataCode[i].timeStamp = osKernelGetTickCount();
		}
	}
}

//����Ƿ���ڹ�����
uint8_t checkFaultCode(ROTACODE* roataCode,uint8_t maxCnt,uint32_t code)
{
	uint8_t i;
	for(i = 0;i < maxCnt && code != 0;i++)
	{
		if(roataCode[i].code == code)
		{
			return 1;
		}
	}
	return 0;
}

uint8_t fun_can_Get_State(uint8_t ch)
{
	ch--;
	if(ch < 3)
	{
		if(osKernelGetTickCount() - CanTimeOutCnt[ch] < CAN_TIME_OUT)
			return 1;
		else
			return 0;
	}
	else
	{
		uint8_t state = 0;
		for(ch = 0;ch < 3;ch++)
		{
			if(osKernelGetTickCount() - CanTimeOutCnt[ch] < CAN_TIME_OUT)
				state++;			
		}
		return state;
	}
	return 0;
}

uint32_t fun_can_Get_recvCnt(uint8_t ch)
{
	ch--;
	if(ch < 3)
		return CanGetCnt[ch];
	return 0;
}
