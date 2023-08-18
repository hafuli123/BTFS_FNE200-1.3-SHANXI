#include "bsp_rtc.h"
#include "stm32f4xx_rtc.h"

RTC_INFOR g_system_dt;           																											/* ϵͳʱ�� */

/*������������������*/
static uint8_t getWeekdayByYearday(uint16_t iY, uint8_t iM, uint8_t iD) 
{
  uint8_t iWeekDay; 
  if (1 == iM || 2 == iM) 
  {
    iM += 12; 
    iY--;
  }
  iWeekDay = (iD + 1 + 2 * iM + 3 * (iM + 1) / 5 + iY + iY / 4 - iY / 100 + iY / 400) % 7;
  switch(iWeekDay)
  {
      case 0 : return RTC_Weekday_Sunday; 
      case 1 : return RTC_Weekday_Monday; 
      case 2 : return RTC_Weekday_Tuesday;
      case 3 : return RTC_Weekday_Wednesday;
      case 4 : return RTC_Weekday_Thursday;
      case 5 : return RTC_Weekday_Friday;                                                    
      case 6 : return RTC_Weekday_Saturday;
      default: break;
  }
  return RTC_Weekday_Sunday;
}

static void RTC_Set_WakeUp(u16 waketime)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RTC_WakeUpCmd(DISABLE);																	//�ر�Wake Up
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);	//ʱ��ѡ��1Hz
	RTC_SetWakeUpCounter(waketime);													//����WakeUp�Զ���װ��ֵ
	RTC_ClearITPendingBit(RTC_IT_WUT);											//���WakeUp�жϱ�־
	EXTI_ClearITPendingBit(EXTI_Line22);										//���Line22�жϱ�־
	RTC_ITConfig(RTC_IT_WUT,ENABLE);												//����WakeUp�жϱ�־
	RTC_WakeUpCmd(ENABLE);																	//����WakeUp

	EXTI_InitStruct.EXTI_Line = EXTI_Line22;//LINE22
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;//�����ش���
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = RTC_WKUP_IRQn;//ѡ��WakeUp�ж�
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStruct);
}

/* 
	RTCʱ�ӻ������� 
*/
void RTC_Initialize(void)
{
	RTC_InitTypeDef RTC_InitStructure;
	
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure;
	
	/* Enable the PWR clock ��ʹ�ܵ�Դʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	/* Allow access to RTC���������RTC */
	PWR_BackupAccessCmd(ENABLE);
	/* Enable the LSE OSC ��ʹ���ⲿ���پ�������*/
	RCC_LSEConfig(RCC_LSE_ON);
	/* Wait till LSE is ready ���ȴ��ⲿ��������׼����*/  
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
	/* Select the RTC Clock Source ,ѡ���ⲿ������ΪRTC��ʱ��Դ*/
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);	
	/* Enable the RTC Clock��ʹ��RTC��ʱ�� */
	RCC_RTCCLKCmd(ENABLE);
	/* Wait for RTC APB registers synchronisation���ȴ����е�RTC�Ĵ������� */
	RTC_WaitForSynchro();
  if(RTC_ReadBackupRegister(RTC_BKP_DR0) != 0xA5A5)   //һ������������RTC��ʼ��û
  {
		/* Configure the RTC data register and RTC prescaler */
		/* ck_spre(1Hz) = RTCCLK(LSE) /(uwAsynchPrediv + 1)/(uwSynchPrediv + 1) = 32768/(127+1)/(255+1)=1Hz ��ΪRTC����������Ϊ��С��ʱ�䵥λ*/
		RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
		RTC_InitStructure.RTC_SynchPrediv = 0xFF;
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;	//24Сʱ��ʽ
		RTC_Init(&RTC_InitStructure);

		/* Set the date: Tuesday July 23th 2019��2019/07/23 ���ڶ� */
		RTC_DateStructure.RTC_Year = 19;
		RTC_DateStructure.RTC_Month = RTC_Month_July;
		RTC_DateStructure.RTC_Date = 23;
		RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Tuesday;
		RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
		
		/* Set the time to 11h 20m 57s AM ������11:20:57*/
		RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
		RTC_TimeStructure.RTC_Hours   = 11;
		RTC_TimeStructure.RTC_Minutes = 20;
		RTC_TimeStructure.RTC_Seconds = 57; 
		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  
  }
	/* Indicator for the RTC configuration��д���ݼĴ��������ڽ����жϱ�־�Ƿ�Ҫ����RTC��������ʱ�� */
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);
	RTC_Set_WakeUp(5);
}

/* ��ȡϵͳʱ�� */
uint8_t RTC_Time_Get(RTC_INFOR* rt)
{
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure;	
	
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
	
	 /* time hours */
	rt->hour= RTC_TimeStructure.RTC_Hours;
	/* time minutes */
	rt->minute = RTC_TimeStructure.RTC_Minutes;
	/* time seconds */
	rt->second = RTC_TimeStructure.RTC_Seconds;
	
	rt->year = RTC_DateStructure.RTC_Year + 2000;
	rt->month = RTC_DateStructure.RTC_Month;
	rt->day = RTC_DateStructure.RTC_Date;	
	return 0;
}

/* ����ϵͳʱ�� */
uint8_t RTC_Time_Set(RTC_INFOR* rt)
{
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure;
	
	RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours = rt->hour;
	RTC_TimeStructure.RTC_Minutes = rt->minute;
	RTC_TimeStructure.RTC_Seconds = rt->second;
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);

	RTC_DateStructure.RTC_WeekDay = getWeekdayByYearday(rt->year,rt->month,rt->day);
	RTC_DateStructure.RTC_Date = rt->day;
	RTC_DateStructure.RTC_Month = rt->month;
	RTC_DateStructure.RTC_Year = rt->year - 2000;
	
	/* Configure the RTC date register */
	RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
	/* Indicator for the RTC configuration */
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);
	return 0;
}

static uint8_t WakeUpSeq;
void RTC_WKUP_IRQHandler()
{
	while(RTC_GetFlagStatus(RTC_FLAG_WUTF)!=RESET)
	{
		WakeUpSeq++;
		RTC_ClearFlag(RTC_FLAG_WUTF);
	}
	EXTI_ClearITPendingBit(EXTI_Line22);
	RTC_Set_WakeUp(5);
}

uint8_t RTC_WakeUpSeq(void)
{
	return WakeUpSeq;
}

#include "rl_fs.h"
fsStatus fs_get_time (fsTime *time) {
  // Real time should be read from the RTC
  // Fill the FS_TIME structure with the time information
	extern RTC_INFOR g_system_dt;
  time->hr   = g_system_dt.hour;    // Hours:   0 - 23
  time->min  = g_system_dt.minute;     // Minutes: 0 - 59
  time->sec  = g_system_dt.second;     // Seconds: 0 - 59
  time->day  = g_system_dt.day;     // Day:     1 - 31
  time->mon  = g_system_dt.month;     // Month:   1 - 12
  time->year = g_system_dt.year;  // Year:    1980 - 2107
  return (fsOK);
}

//UTCת����ʱ��
void UTC2CHINA(RTC_INFOR* rt)
{
	const uint8_t ucaDays[]={31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	rt->hour += 8;	
	if(rt->hour / 24 > 0)
	{//���ڸı�
		uint8_t iDays = 0;
		rt->hour %= 24;//����24Сʱ�����µ���ʱ��
		++rt->day;		 //���ڼ�1
		iDays = ucaDays[rt->month-1];
		if(rt->month==2)
		{//2�·ݱȽ����⣬��Ҫ�����ǲ����������жϵ�����28�컹29��
			iDays =  (((rt->year%4)==0)&&((rt->year%100)!=0))||((rt->year%400)==0)?29:28;	
		}
		if(rt->day > iDays)
		{
			rt->day = 1;//��ǰ�����ȵ������ʱ������µ�������
			++rt->month;//���µ����·�

			//�ж��·��Ƿ����12
			if(rt->month > 12)
			{
				rt->month = 1;
				++rt->year;
			}
		}
	}
}

//����Unixʱ���
unsigned long RTC_mktime (unsigned int year, unsigned int mon,unsigned int day, unsigned int hour,unsigned int min, unsigned int sec)
{
   if (0 >= (int) (mon -= 2))
	{    /**//* 1..12 -> 11,12,1..10 */
         mon += 12;      /**//* Puts Feb last since it has leap day */
         year -= 1;
   }
    return (((
             (unsigned long) (year/4 - year/100 + year/400 + 367 * mon / 12 + day) + year * 365 - 719499
          )*24 + hour /**//* now have hours */
       )*60 + min /**//* now have minutes */
    )*60 + sec - 28800; /**//* finally seconds */
}

void RTC_localtime(uint32_t tick,RTC_INFOR *dt)
{
	const char Days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	unsigned int year;
	unsigned int mon,day,hour,min,sec;
	unsigned  int n32_Pass4year;
	int n32_hpery;
	tick += 28800;
	sec=(int)(tick % 60);																//ȡ��ʱ��
	tick /= 60;
	min=(int)(tick % 60);																//ȡ����ʱ��
	tick /= 60;
	n32_Pass4year=((unsigned int)tick / (1461L * 24L));	//ȡ��ȥ���ٸ����꣬ÿ������ 1461*24 Сʱ
	year=(n32_Pass4year << 2)+70;												//�������
	tick %= 1461L * 24L;																//������ʣ�µ�Сʱ��
	for (;;)																						//У������Ӱ�����ݣ�����һ����ʣ�µ�Сʱ��
	{
		n32_hpery = 365 * 24;															//һ���Сʱ��
		if ((year & 3) == 0)															//�ж�����
		{
			n32_hpery += 24;																//�����꣬һ�����24Сʱ����һ��
		}
		if (tick < n32_hpery)
		{
			break;
		}
		year++;
		tick -= n32_hpery;
	}
	hour=(int)(tick % 24);															//Сʱ��
	tick /= 24;																					//һ����ʣ�µ�����
	tick++;																							//�ٶ�Ϊ����
	if ((year & 3) == 0)																//У��������������·ݣ�����
	{
		if (tick > 60)
		{
			tick--;
		}
		else if (tick == 60)
		{
			mon = 1;
			day = 29;
			dt->year = year + 1900;
			dt->month = mon;
			dt->day = day;
			dt->hour = hour;
			dt->minute = min;
			dt->second = sec;
			return ;
		}
	}
	for (mon = 0; Days[mon] < tick;mon++)									//��������
	{
		tick -= Days[mon];
	}
	mon++;
	day = (int)(tick);
	dt->year = year + 1900;
	dt->month = mon;
	dt->day = day;
	dt->hour = hour;
	dt->minute = min;
	dt->second = sec;
}
