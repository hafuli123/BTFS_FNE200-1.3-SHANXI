#include "task_mon.h"
#include "cmsis_os2.h"

#include "fun_mon.h"
#include "bsp_power.h"
#include "bsp_wdg.h"

osThreadId_t task_mon_id;
#define TASK_MON_STK_SZ (1200)
uint64_t task_mon_stk[TASK_MON_STK_SZ / 8];
const osThreadAttr_t task_mon_attr = {.name = "task_mon",.stack_mem  = &task_mon_stk[0],.stack_size = sizeof(task_mon_stk),.priority = osPriorityNormal};
__NO_RETURN void task_mon(void *arg)
{
	funMonInit();
	while(1)
	{
		if(bsp_sleep_state() == 1)
		{
			osDelay(100);
			continue;
		}		
		funMonrun();
		WDG_Feed(2);
	}
}

void task_mon_Create(void)
{
	task_mon_id = osThreadNew(task_mon, NULL, &task_mon_attr);
}
