/*
�ļ���bsp_sys.h
���ܣ�ϵͳ��Խ�
���ڣ�2022/1/28
��˾����ɽ��Դ
���ߣ�czj
*/

#ifndef __BSP_SYS_H
#define __BSP_SYS_H

#include "stm32f4xx.h"
#include "cmsis_os2.h" 
#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif
#define BoardReset()  (SCB->AIRCR = (u32)0x05FA0000 | (u32)0x04)

#define Bsp_GetSysTicks_ms() osKernelGetTickCount()
#define Bsp_Delay_ms(msec)	osDelay(msec)

#define Bsp_Lock()
#define Bsp_Unlock();

#endif
