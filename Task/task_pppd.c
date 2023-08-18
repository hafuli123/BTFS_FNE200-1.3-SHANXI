#include "task_pppd.h"
#include "cmsis_os2.h"

#include "Fun_Net.h"
#include "bsp_power.h"
#include "bsp_wdg.h"

osThreadId_t task_pppd_id;
#define TASK_PPPD_STK_SZ (1400)
uint64_t task_pppd_stk[TASK_PPPD_STK_SZ / 8];
const osThreadAttr_t task_pppd_attr = {.name = "task_pppd",.stack_mem  = &task_pppd_stk[0],.stack_size = sizeof(task_pppd_stk),.priority = osPriorityNormal};
__NO_RETURN void task_pppd(void *arg)
{
	Fun_Net_Init();
	while(1)
	{
		if(bsp_sleep_state() == 1)
		{
			osDelay(100);
			continue;
		}
		Fun_Net_Run();
		WDG_Feed(1);
	}
}

void task_pppd_Create(void)
{
	task_pppd_id = osThreadNew(task_pppd, NULL, &task_pppd_attr);
}
