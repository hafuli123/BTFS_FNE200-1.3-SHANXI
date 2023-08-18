#include "bsp.h"
#include "bsp_sys.h"
#include "bsp_storage.h"
#include "bsp_io.h"
#include "bsp_rtc.h"
#include "bsp_wdg.h"
#include "bsp_uart_fifo.h"
#include "bsp_gps.h"
#include "se.h"
#include "rl_usb.h"
#include "stm32f4xx_rtc.h"

void  bsp_Init (void)
{
	BSP_IoInit();
	BSP_ADC_IoInit();
	sys_parameter_read();
//	WDG_Init();
	RTC_Initialize();
	//睡眠后需要重开
  bsp_InitUart();
	bsp_storage_init();
	if(RTC_ReadBackupRegister(RTC_BKP_DR1) == 0xA5A5)
	{
		RTC_WriteBackupRegister(RTC_BKP_DR1, 0);
		if(BSP_Iostatus(IN_USB_ID) == ON)
		{
			USBD_Initialize(0U);
			USBD_Connect(0U);
			osDelay(500);
			while(1)
			{
				uint8_t uchar;
				if(comGetChar(RS1_COM,&uchar))
					comSendChar(RS2_COM,uchar);
				else if(comGetChar(RS2_COM,&uchar))
					comSendChar(RS1_COM,uchar);
				else
				{
					WDG_Feed(0);
					osDelay(1);
				}
				USBD_STATE sta = USBD_GetState(0);
				if(sta.active == 0)
				{
					BoardReset();
				}
			}
		}
	}
	tms_initialize(0);
	//睡眠后不需重开
	bsp_InitGPS();
	//开机滴一声
	BSP_Ioctl(ST_BEEP_ID,ON);
}
