#include "task_prtcl.h"
#include "cmsis_os2.h"

#include "fun_prtcl.h"
#include "bsp_power.h"
#include "bsp_wdg.h"

osThreadId_t task_prtcl_id;
#define TASK_PRTCL_STK_SZ (3600)
uint64_t task_prtcl_stk[TASK_PRTCL_STK_SZ / 8];
const osThreadAttr_t task_prtcl_attr = {.name = "task_prtcl",.stack_mem  = &task_prtcl_stk[0],.stack_size = sizeof(task_prtcl_stk),.priority = osPriorityHigh};
__NO_RETURN void task_prtcl(void *arg)
{
	funPrtclInit();
	while(1)
	{
		if(bsp_sleep_state() == 1)
		{
			osDelay(100);
			continue;
		}
		funPrtclrun();
		WDG_Feed(3);
	}
}

void task_prtcl_Create(void)
{
	task_prtcl_id = osThreadNew(task_prtcl, NULL, &task_prtcl_attr);
}
