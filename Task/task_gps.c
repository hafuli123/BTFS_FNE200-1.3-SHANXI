#include "task_gps.h"
#include "cmsis_os2.h"

#include "fun_gps.h" 
#include "bsp_power.h"
#include "bsp_wdg.h"

#include "string.h"

osThreadId_t task_gps_id;
#define TASK_GPS_STK_SZ (800)
uint64_t task_gps_stk[TASK_GPS_STK_SZ / 8];
const osThreadAttr_t task_gps_attr = {.name = "task_gps",.stack_mem  = &task_gps_stk[0],.stack_size = sizeof(task_gps_stk),.priority = osPriorityNormal};

osSemaphoreId_t sem_gps_id;
int apple;

extern volatile uint8_t u8_rxBuf[100];
uint8_t test[100];

__NO_RETURN void task_gps(void *arg)
{
	while(1)
	{
//		if(bsp_sleep_state() == 1)
//		{
//			osDelay(100);
//			continue;
//		}
		
		osDelay(1000); 
		
		osSemaphoreAcquire(sem_gps_id, osWaitForever);
//		memcpy(test,(uint8_t*)u8_rxBuf,80);
		funGpsrun();
//		apple++;
		WDG_Feed(4);
		
	}
}

void task_gps_Create(void)
{
	task_gps_id = osThreadNew(task_gps, NULL, &task_gps_attr);
	
	sem_gps_id=osSemaphoreNew (1, 0, NULL);
}
