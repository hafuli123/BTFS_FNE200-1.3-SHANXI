#ifndef _BSP_RTC_H
#define _BSP_RTC_H

#include "stdint.h"

typedef struct DT{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
}RTC_INFOR;

extern void RTC_Initialize(void);										/* RTC��ʼ�� */
extern uint8_t RTC_Time_Get(RTC_INFOR* rt);   			/* ��ȡϵͳʱ�� */
extern uint8_t RTC_Time_Set(RTC_INFOR* rt);   			/* ����ϵͳʱ�� */
extern uint8_t RTC_WakeUpSeq(void);									/* RTC������ˮ�� */

extern void UTC2CHINA(RTC_INFOR* rt);
extern unsigned long RTC_mktime (unsigned int year, unsigned int mon,unsigned int day, unsigned int hour,unsigned int min, unsigned int sec);
extern void RTC_localtime(uint32_t tick,RTC_INFOR *dt);

extern RTC_INFOR g_system_dt;

#endif
