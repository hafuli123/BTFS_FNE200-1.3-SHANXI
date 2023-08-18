#include "task_can.h"
#include "cmsis_os2.h"

#include "fun_can.h"
#include "bsp_power.h"
#include "bsp_wdg.h"

osThreadId_t task_can_id;
#define TASK_CAN_STK_SZ (1600)
uint64_t task_can_stk[TASK_CAN_STK_SZ / 8];
const osThreadAttr_t task_can_attr = {.name = "task_can",.stack_mem  = &task_can_stk[0],.stack_size = sizeof(task_can_stk),.priority = osPriorityAboveNormal};
__NO_RETURN void task_can(void *arg)
{
	funCanInit();
	while(1)
	{
		if(bsp_sleep_state() == 1)
		{
			osDelay(100);
			continue;
		}
		funCanrun();
		WDG_Feed(0);
	}
}

void task_can_Create(void)
{
	task_can_id = osThreadNew(task_can, NULL, &task_can_attr);
}
