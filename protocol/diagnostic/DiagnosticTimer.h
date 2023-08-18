/**
  ******************************************************************************
  * @file    timer.h
  * @author  Soundtech Application Team
  * @version V1.0.0
  * @date    14-Jan-2015
  * @brief   Header file for timer.c module.
  * <h2><center>&copy; COPYRIGHT 2015 Soundtech</center></h2>
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "bsp_sys.h"

/** @defgroup Exported_Macros
  * @{
  */ 
#ifndef bool
#define bool unsigned char
#endif

typedef struct{
	uint32_t TimerCounter;
	bool valid;
}DiagTimer;

/**
  * @}
  */ 

/** @defgroup Exported_Functions
  * @{
  */ 
void DiagTimer_Set(DiagTimer *STimer, uint32_t TimeLength);
bool DiagTimer_HasStopped(DiagTimer *STimer);
bool DiagTimer_HasExpired (DiagTimer *STimer);
void DiagTimer_Stop(DiagTimer *STimer);
uint32_t DiagTimer_GetTickCount(void);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif


#endif /* __TIMER_H */

/******************* (C) COPYRIGHT 2015 Soundtech *****END OF FILE****/

