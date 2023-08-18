#ifndef _BSP_IO_H
#define _BSP_IO_H

#include "stdint.h"

#define ON              0xAA
#define OFF             0x55

typedef enum
{
	//状态指示
	ST_BEEP_ID,
	ST_DATA_ID,
	ST_GPRS_ID,
	ST_GPS_ID,
	ST_CAN_ID,
	//电源控制
	PWR_BAT_ID,
	PWR_3V3_ID,
	PWR_MCU_IO,
	PWR_GPRS_IO,
	PWR_VCC_ID,
	//外设睡眠控制
	CAN_STB_ID,
	GPRS_DTR_ID,
	//输入
	IN_ACC_ID,	
	IN_WAKEUP_ID,
	IN_USB_ID,
	//ADC
	ADC_VCC_ID,
	ADC_BAT_ID,
	//自动补电IO
	AUTO_CHG_ID,
}IO_E;

extern void BSP_IoInit(void);
extern void BSP_ADC_IoInit(void);
extern void BSP_IoDeInit(void);
extern void BSP_ADC_IoDeInit(void);
extern void BSP_Ioctl(uint8_t id,uint8_t toggle);
extern uint8_t BSP_WakeUpSeq(void);
extern uint8_t BSP_Iostatus (uint8_t id);
extern float BSP_Get_ADC_Value(uint8_t id);
extern void ledFlash(uint8_t id,uint16_t intervel);
extern void whistle(uint16_t time,uint16_t intervel);
extern uint8_t BSP_GetVer(void);

#endif
