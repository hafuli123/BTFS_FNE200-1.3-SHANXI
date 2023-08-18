#include "bsp_power.h"

#include "stm32f4xx_pwr.h"
#include "cmsis_os2.h"

#include "bsp_io.h"
#include "bsp_gps.h"
#include "bsp_uart_fifo.h"
#include "Fun_Net.h"
#include "fun_can.h"
#include "se.h"
#include "bsp_rtc.h"
#include "rl_fs.h"
#include "stdio.h"
#include "string.h"


typedef struct
{
	uint8_t use;																										//����1��ɾ����0
	uint8_t lock;																										//lockΪ1������˯��
	uint8_t keep_socket_flag;																				//flagΪ1������·
	int8_t socket_fd;																								//socket
	uint32_t interval;																							//�������ڣ���0��Ч
	uint32_t timestamp;																							//����ʱ�䣬��0��Ч
	uint32_t realWakeUpStamp;																				//ʵʱ����ʱ��
	uint8_t (*callback)(uint8_t type);															//�ص�����������ֵΪ1�������ѣ�����ֵΪ0��������												
}bsp_sleep_lock_t;

static BSP_SLEEP_FLAG_E sleep_enable_flag = BSP_NOT_ALLOW_SLEEP;	//�Ƿ�����˯��																				
static bsp_sleep_lock_t sleep_lock_array[BSP_WAKELOCK_CNT] = {0};	//����������
static bsp_pwrup_reason sleep_pwrup_reason = BSP_PWRUP_PWRKEY;		//����ԭ��
static uint8_t sleep_state = 0;																		//˯��״̬

static float sleep_wakeVoltVal = 0;																//�͵�ѹ����ֵ
static uint8_t sleep_wakeVoltVal_Check = 0;												//�͵�ѹ����־
static char tempPrintfBuff[50] = {0};


void bsp_sleep_enable(BSP_SLEEP_FLAG_E sleep_flag)								//˯��ʹ��
{
	sleep_enable_flag = sleep_flag;
}

int8_t bsp_wakelock_create(void)																	//����������
{
	uint8_t i;
	for(i = 0;i < BSP_WAKELOCK_CNT;i++)
	{
		if(sleep_lock_array[i].use == 0)
		{
			sleep_lock_array[i].use = 1;
			sleep_lock_array[i].lock = 0;
			sleep_lock_array[i].keep_socket_flag = 0;
			sleep_lock_array[i].socket_fd = -1;
			sleep_lock_array[i].interval = 0;
			sleep_lock_array[i].timestamp = 0;
			sleep_lock_array[i].callback = 0;
			sleep_lock_array[i].realWakeUpStamp = 0;
			return i;
		}
	}
	return -1;
}

bsp_errcode_sleep bsp_wakelock_delete(int8_t wakelock_fd)					//������ɾ��
{
	if(wakelock_fd >= 0 && wakelock_fd < BSP_WAKELOCK_CNT)
	{
		if(sleep_lock_array[wakelock_fd].use == 1)
		{
			sleep_lock_array[wakelock_fd].use = 0;
			sleep_lock_array[wakelock_fd].lock = 0;
			return BSP_SLEEP_SUCCESS;
		}
		return BSP_SLEEP_LOCK_DELETE_FAIL;
	}
	return BSP_SLEEP_INVALID_PARAM;
}

bsp_errcode_sleep bsp_wakelock_lock(int8_t wakelock_fd)						//����������
{
	if(wakelock_fd >= 0 && wakelock_fd < BSP_WAKELOCK_CNT)
	{
		if(sleep_lock_array[wakelock_fd].use == 1)
		{
			sleep_lock_array[wakelock_fd].lock = 1;
			return BSP_SLEEP_SUCCESS;
		}
		return BSP_SLEEP_LOCK_DELETE_FAIL;
	}
	return BSP_SLEEP_INVALID_PARAM;	
}

bsp_errcode_sleep bsp_wakelock_unlock(int8_t wakelock_fd)					//����������
{
	if(wakelock_fd >= 0 && wakelock_fd < BSP_WAKELOCK_CNT)
	{
		if(sleep_lock_array[wakelock_fd].use == 1)
		{
			sleep_lock_array[wakelock_fd].lock = 0;
			return BSP_SLEEP_SUCCESS;
		}
		return BSP_SLEEP_LOCK_DELETE_FAIL;
	}
	return BSP_SLEEP_INVALID_PARAM;		
}
/*******************************************
���ܣ����Ѳ�������
������ʱ��������ڣ�tcp��·
			������ID
			������ʼ��ʱʱ���
			��������			
			���ѻص�����
			����������·��			1-5
			������·����״̬		1 ����	
********************************************/
bsp_errcode_sleep bsp_wakelock_para(int8_t wakelock_fd,uint32_t timestamp,uint32_t interval,uint8_t (*callback)(uint8_t type),int8_t socket_fd,uint8_t keep_socket_flag)	
{
	if(wakelock_fd >= 0 && wakelock_fd < BSP_WAKELOCK_CNT)
	{
		if(sleep_lock_array[wakelock_fd].use == 1)
		{
			if(interval == 0)
			{
				//���λ���
				interval = 0xFFFFFFFF;
			}
			sleep_lock_array[wakelock_fd].keep_socket_flag = keep_socket_flag;
			sleep_lock_array[wakelock_fd].socket_fd = socket_fd;
			sleep_lock_array[wakelock_fd].interval = interval;
			sleep_lock_array[wakelock_fd].timestamp = timestamp;
			sleep_lock_array[wakelock_fd].callback = callback;
			sleep_lock_array[wakelock_fd].realWakeUpStamp = timestamp;
			return BSP_SLEEP_SUCCESS;
		}
		return BSP_SLEEP_INVALID_PARAM;
	}
	return BSP_SLEEP_INVALID_PARAM;
}

bsp_errcode_sleep bsp_get_powerup_reason(uint8_t *pwrup_reason)				//��ȡ����ԭ��
{
	*pwrup_reason = sleep_pwrup_reason;
	return BSP_SLEEP_SUCCESS;
}

bsp_errcode_sleep bsp_enter_sleep(void)																//����˯��
{
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
	static uint32_t enterTicks = 0;
	RTC_INFOR dt;
	uint32_t i,j,time;
	uint8_t soft_sleep = 0,canIrqSeq,riIrqSeq,isLowVolt = 0;
	
	//�ж��Ƿ�����˯��
	if(sleep_enable_flag != BSP_ALLOW_SLEEP || (osKernelGetTickCount() - enterTicks) <= 30000)
	{
		return BSP_SLEEP_EXECUTE_FAIL;
	}
	//�ж�˯�����Ƿ�����˯��
	for(i = 0;i < BSP_WAKELOCK_CNT;i++)
	{
		if(sleep_lock_array[i].use == 1 && sleep_lock_array[i].lock == 1)
		{
			return BSP_SLEEP_EXECUTE_FAIL;
		}
	}
	
	//�ɰ�����˯���ж�
	if(BSP_GetVer())
	{
		if(fun_can_Get_State(BSP_CAN) || BSP_Iostatus(IN_ACC_ID) == ON)
		{
			return BSP_SLEEP_EXECUTE_FAIL;
		}
		BSP_Ioctl(CAN_STB_ID,OFF);
	}
	//�°�����˯���ж�
	else
	{		
		BSP_Ioctl(CAN_STB_ID,OFF);									//CAN˯��	
		osDelay(1000);		
		if(BSP_Iostatus(IN_WAKEUP_ID) == ON)				//�ж��Ƿ񻹴��ڻ���Դ
		{
			BSP_Ioctl(CAN_STB_ID,ON);									//CAN����
			return BSP_SLEEP_EXECUTE_FAIL;
		}
	}
	//�ж����˯�߻���Ӳ��˯��
	for(i = 0;i < BSP_WAKELOCK_CNT;i++)
	{
		if(sleep_lock_array[i].use == 1 && sleep_lock_array[i].timestamp != 0)
		{
			//���˯��,����Զ�̻���
			soft_sleep = 1;
		}
		if(sleep_lock_array[i].use == 1 && sleep_lock_array[i].socket_fd >= 0 && sleep_lock_array[i].keep_socket_flag == 1)
		{
			//���˯�ߣ���ҪԶ�̻���
			if(BSP_GetVer() == 0)
			{
				soft_sleep = 2;
				break;
			}
		}
	}
//	if(ftpMode == 1)												
//	{
//		return BSP_SLEEP_LOCK_AUTOSLEEP_FAIL;	//������ָ�����˯��
//	}
	
	//Ӳ��˯��
	if(soft_sleep == 0)
	{
		BSP_Ioctl(CAN_STB_ID,OFF);
		BSP_Ioctl(PWR_MCU_IO,OFF);						//��Դ�����Ͽ�
		osDelay(1000);
		BSP_Ioctl(CAN_STB_ID,ON);							//CAN����
		BSP_Ioctl(PWR_MCU_IO,ON);							//��Դ������ͨ
		return BSP_SLEEP_EXECUTE_FAIL;				//Ӳ��˯��ʧ��
	}
	sleep_state = 1;
	//���˯�ߣ���ʱ���ѣ��ر�4Gģ��
	if(soft_sleep == 1)
	{
		BSP_Ioctl(PWR_GPRS_IO,OFF);
	}
	//���˯�ߣ�Զ�̻��ѣ�4Gģ��˯��
	else if(soft_sleep == 2)
	{
		Fun_Gprs_Sleep();
	}
	
	//�ر�����
	funmount("M0:");
	funinit("M0:");
	//tms_finalize();
	BSP_IoDeInit();
//	BSP_ADC_IoDeInit();
	if(soft_sleep == 2)
	{
		for(i=0;i < 6;i++)
		{
			for(j=0;j<BSP_WAKELOCK_CNT;j++)
			{
				if(sleep_lock_array[j].use == 1 && sleep_lock_array[j].socket_fd == i && sleep_lock_array[j].keep_socket_flag == 1)
				{
					break;
				}
			}
			if(j == BSP_WAKELOCK_CNT)
				Fun_Gprs_Tcp_disconnect(i);
		}
	}
	osDelay(2000);	
	soft_sleep = 0;
	canIrqSeq = CAN_IrqSeq();
	riIrqSeq = BSP_WakeUpSeq();
	while(1)
	{
		IWDG_ReloadCounter();
		//�ⲿ���ѻ�Զ�̻��Ѽ��
		if(riIrqSeq != BSP_WakeUpSeq() || CAN_IrqSeq() != canIrqSeq || BSP_Iostatus(IN_WAKEUP_ID) == ON)
		{
			//���ӻ��ѵ����籨��
			if(BSP_GetVer() == 0)
			{
				Fun_Gprs_WakeUp();
			}
			IWDG_ReloadCounter();
			
			if(riIrqSeq != BSP_WakeUpSeq())					//Զ�̻���
				isLowVolt |= 1<< 2;
			if(CAN_IrqSeq() != canIrqSeq)
				isLowVolt |= 1<< 3;
			if(BSP_Iostatus(IN_WAKEUP_ID) == ON)
				isLowVolt |= 1<< 4;
			break;
		}
		RTC_Time_Get(&dt);
		time = RTC_mktime(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second);
		
		//�͵籣�������߼�    �������1����һ��
		if(sleep_wakeVoltVal !=0 && dt.second %25 == 0 && sleep_wakeVoltVal_Check !=dt.second)		
		{
			sleep_wakeVoltVal_Check = dt.second;																								
//			BSP_ADC_IoInit();
			gTerminalState.pwrVolt = BSP_Get_ADC_Value(ADC_VCC_ID);
//			BSP_ADC_IoDeInit();
			if(gTerminalState.pwrVolt<=sleep_wakeVoltVal)	
			{
				isLowVolt |= 1<<0;							//���͵�ѹ����
			}			
		}
		
		//��ʱ����
		for(i = 0;i < BSP_WAKELOCK_CNT;i++)
		{
			if(sleep_lock_array[i].use == 1 && sleep_lock_array[i].timestamp != 0)
			{
				//���͵�ѹ���� �� ���ڻ���
				if((isLowVolt&0x1)==1||((time - sleep_lock_array[i].timestamp) % sleep_lock_array[i].interval < 30 && (time - sleep_lock_array[i].realWakeUpStamp) >= 30))
				{
					sleep_lock_array[i].realWakeUpStamp = time;
					//����
					if(sleep_lock_array[i].callback != NULL)
					{
						soft_sleep = sleep_lock_array[i].callback(0);
						IWDG_ReloadCounter();
						riIrqSeq = BSP_WakeUpSeq();
					}
					else
					{
						soft_sleep = 1;
					}
					
					if((isLowVolt & 0x1)==1)		
						isLowVolt |= 1<<1;
				}
			}
		}
		//��ʱ����
		if(soft_sleep > 0)
		{
			break;
		}
		if(BSP_GetVer() == 0)		//�°� ִ��
		{
			PWR_EnterSTOPMode(PWR_LowPowerRegulator_ON,PWR_STOPEntry_WFI);
		}
		SystemInit();
	}
	//��������
	BSP_IoInit();
//	BSP_ADC_IoInit();

	finit("M0:");
	fmount("M0:");
	bsp_InitUart();
	bsp_InitGPS();
	//whistle(100,100);
	//whistle(100,100);
	//tms_initialize(0);
	Fun_Gprs_WakeUp();
	enterTicks = osKernelGetTickCount();
	sleep_state = 0;
	
	//��ӡ����Դ������
	memset(tempPrintfBuff,0,sizeof(tempPrintfBuff));
	sprintf(tempPrintfBuff,"isLowVolt=%d\r\n",isLowVolt);
	printfData(tempPrintfBuff);
	
	gTerminalState.pwrVolt = BSP_Get_ADC_Value(ADC_VCC_ID);
	return BSP_SLEEP_SUCCESS;
}

uint8_t bsp_sleep_state(void)														//��ȡ˯��״̬
{
	return sleep_state;
}

bsp_errcode_sleep bsp_sleep_wakeup_volt(float voltLowVal)			//Ƿѹ��ѹ����ֵ����
{
	sleep_wakeVoltVal = voltLowVal;
	return BSP_SLEEP_SUCCESS;
}
