#include "cmsis_os2.h"
#include "bsp.h"
#include "task_can.h"
#include "task_pppd.h"
#include "task_prtcl.h"
#include "task_mon.h"
#include "task_gps.h"
#include "pdu.h"

#include "bsp_uart_fifo.h"
#include "bsp_gps.h"
#include "bsp_io.h"

RealData gRealData;
TERMINAL_PARA gFrimPara;
APP_PARA gSysPara;
RealData gRealData;
TerminalState gTerminalState;
uint8_t szMainBuf[2048];

osThreadId_t task_init_id;
#define TASK_INIT_STK_SZ (800)
uint64_t task_init_stk[TASK_INIT_STK_SZ / 8];
const osThreadAttr_t task_init_attr = {.name = "task_init",.stack_mem  = &task_init_stk[0],.stack_size = sizeof(task_init_stk),.priority = osPriorityNormal};
__NO_RETURN void task_init(void *arg)
{
	bsp_Init();
	task_can_Create();
	task_pppd_Create();
	task_prtcl_Create();
	task_mon_Create();
	
	task_gps_Create();
}



int main(void)
{
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();
  /* Create application main thread */
  task_init_id = osThreadNew(task_init, NULL, &task_init_attr);
  /* Start thread execution */
  osKernelStart();
  /* Infinite loop */
  while (1)
  {
  }
}
