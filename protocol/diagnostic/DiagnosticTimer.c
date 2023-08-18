#include "DiagnosticTimer.h"
#define TRUE 1
#define FALSE 0
/**
  * @brief  Sets a timer. timer must not equ 0 because 0 means timer is stop.
  * @param  STimer pointer to timer value.
  * @param  TimeLength - timer period.
  * @retval None.
  */
//void DiagTimer_Set(DiagTimer *STimer, uint32_t TimeLength)
//{
//	STimer->TimerCounter = osKernelGetTickCount() + TimeLength;
//	STimer->valid = TRUE;
//}

/**
  * @brief  Stop timer.
  * @param  STimer pointer to timer value.
  * @retval None.
  */
void DiagTimer_Stop(DiagTimer *STimer)
{
	STimer->valid = FALSE;
}

/**
  * @brief  Checks whether given timer has stopped.
  * @param  STimer is timer value.
  * @retval TRUE if timer is stopped.
  */
bool DiagTimer_HasStopped(DiagTimer *STimer)
{
	return (STimer->valid == FALSE); 
}

/**
  * @brief  Checks whether given timer has expired
  *        With timer tick at 1ms maximum timer period is 10000000 ticks
  *        When *STimer is set (SoftTimer-*STimer) has a min value of 0xFFF0BDBF
  *            and will be more than this while the timer hasn't expired
  *        When the timer expires
  *                (SoftTimer-*STimer)==0
  *            and (SoftTimer-*STimer)<=7FFF FFFF for the next 60 hours
  * @param  STimer pointer to timer value.
  * @retval TRUE if timer has expired or timer is stopped, otherwise FALSE.
  */
//bool DiagTimer_HasExpired(DiagTimer *STimer)
//{
//	if(STimer->valid == TRUE)
//	{
//		if(STimer->TimerCounter == 0)
//		{
//			STimer->valid = FALSE;
//			return TRUE;
//		}
//		else if((osKernelGetTickCount() - STimer->TimerCounter) <= 0x7fffffff)
//		{
//			STimer->TimerCounter = 0;		//set timer to stop
//			STimer->valid = FALSE;
//			return TRUE;
//		}
//		else
//		{
//			return FALSE;
//		}
//	}
//	else
//	{
//		return FALSE;
//	}
//}

/**
  * @brief  get the system work time.
  * @param  None.
  * @retval None.
  */
uint32_t DiagTimer_GetTickCount(void)
{
	return osKernelGetTickCount();
}

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2015 Soundtech *****END OF FILE****/

