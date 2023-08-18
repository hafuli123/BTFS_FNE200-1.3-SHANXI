#include "bsp_wdg.h"
#include "stm32f4xx_iwdg.h"
#include "cmsis_os2.h"

#define WDG_ID_CNT	10

uint8_t wdg_id[WDG_ID_CNT] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t WDG_Init(void)
{
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: 32KHz(LSI) / 256 = 0.125 KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	/* Set counter reload value to 349 */
	IWDG_SetReload(2000);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
	
	return 1;
}

//void WDG_Feed(void)
//{
//	IWDG_ReloadCounter();		
//}

static uint32_t feedTime = 0;
void WDG_Feed(int feedId)
{
	int i;
//	if(feedId !=2)
//	{
//		return;
//	}
	if(osKernelGetTickCount() - feedTime >= 14000)
	{
		wdg_id[feedId] = wdg_id[feedId];
	}
	//刷新状态
	if(feedId < WDG_ID_CNT)
	{
		wdg_id[feedId] = 1;
	}
	//ID未更新，返回
	for(i = 0;i < WDG_ID_CNT;i++)
	{
		if(wdg_id[i] == 0)
		{
			return;
		}
	}
	feedTime = osKernelGetTickCount();
	//全部线程运行，刷新看门狗
	IWDG_ReloadCounter();
	for(i = 0;i < WDG_ID_CNT;i++)
	{
		if(wdg_id[i] == 1)
		{
			//清零，需要下次线程刷新才能更新看门狗
			wdg_id[i] = 0;
		}
	}
}
