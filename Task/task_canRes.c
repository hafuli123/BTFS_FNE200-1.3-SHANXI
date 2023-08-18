#include "task_canRes.h"
#include "cmsis_os2.h"

#include "bsp_power.h"


osThreadId_t task_canRes_id;
#define TASK_CANRES_STK_SZ (2400)
uint64_t task_canRes_stk[TASK_CANRES_STK_SZ / 8];
const osThreadAttr_t task_canRes_attr = {.name = "task_canRes",.stack_mem  = &task_canRes_stk[0],.stack_size = sizeof(task_canRes_stk),.priority = osPriorityAboveNormal};

#define GETCANLOGTIMELENTH			30000
char saveBuff[6144] = {0};

uint32_t CanLogTime = 0;
uint32_t CanLogTime1 = 0;
uint32_t CanLogTime2 = 0;
FILE* files;
fsStatus sta;
char path[35] = {0};
char databuff[50] = {0};
uint16_t saIndex = 0;
uint32_t byteswritten = 0;                /* File write counts */

static void getCANLOGs(void);

__NO_RETURN void task_canRes(void *arg)
{
	while(1)
	{
		if(bsp_sleep_state() == 1)
		{
			osDelay(100);
			continue;
		}
		getCANLOGs();
		if(getCanLogSign == 2)
			osDelay(1);
		else
			osDelay(100);
	}
}

void task_canRes_Create(void)
{
	task_canRes_id = osThreadNew(task_canRes, NULL, &task_canRes_attr);
}

uint8_t isResCANData(uint8_t ch,CAN_msg *msg)
{
		if(getCanLogSign == 2 && bsp_storage_state() > 0)
		{
			if(CanLogTime2 < GETCANLOGTIMELENTH)
			{
				if(files != NULL)
				{
					if(ftell(files) == 0)		//日志头
						sprintf(databuff,"GetCANLogTime:%04d%02d%02d%02d%02d%02d\r\n",g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);		
					else										//正文
						sprintf(databuff,"%05d %03x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",(osKernelGetTickCount() - CanLogTime), msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);				
					//采集CAN日志
					if(saIndex + 50 <= sizeof(saveBuff))
					{
						memcpy(&saveBuff[saIndex],databuff,strlen(databuff));
						saIndex += strlen(databuff);
					}
				}
			}
		}
}

/*
功能：获取实车CAN报文
描述：获取实车30sCAN 报文，接收指令后，进入日志截取倒计时
uint8_t saveHistory(uint8_t link,uint8_t flag,uint8_t* buff,uint16_t len,uint32_t idx,FILE_TYPE fileType)

*/

static void getCANLOGs(void)
{
	if(getCanLogSign == 1)
	{
		getCanLogSign = 2;
		CanLogTime = osKernelGetTickCount();
		memset(saveBuff,0,sizeof(saveBuff));
		
		sprintf(path,"M0:\\%02x\\%s%02d%02d%02d%02d.%s",0xA2,gFrimPara.terminalId,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second,"LOG");
		if(bsp_storage_state() > 0)
		{
			files = fopen(path, "ab");
		}
	}
	
	if(getCanLogSign == 2 && bsp_storage_state() > 0)
	{		
		CanLogTime1 = osKernelGetTickCount();
		CanLogTime2 = osKernelGetTickCount() - CanLogTime;
		if(CanLogTime2 < GETCANLOGTIMELENTH)
		{
			if(files == NULL)
			{
				sta = funmount("M0:");
				sta = funinit("M0:");			
				osDelay(2);
				sta = finit("M0:");
				sta = fmount("M0:");
				if(sta == fsOK)
				{
					files = fopen(path, "ab");
				}		
			}				

			if(files != NULL)
			{
				if(saIndex + 500 >= sizeof(saveBuff))
				{
					uint16_t bufLen = strlen(saveBuff);
					byteswritten = fwrite(saveBuff,1,bufLen,  files);	
					if(byteswritten != bufLen)
					{
						bufLen = fflush(files);	
						if(bufLen == 0)
						{
							saIndex = bufLen;		
						}
						fclose(files);
						files = NULL;
					}
					memset(saveBuff,0,sizeof(saveBuff));
					saIndex = 0;
				}
			}
		}
		else
		{
			if(files != NULL)
			{
				byteswritten = fwrite(saveBuff,1, strlen(saveBuff), files);	
				fflush(files);
				fclose(files);
				files = NULL;
				memset(saveBuff,0,sizeof(saveBuff));
				saIndex = 0;
				getCanLogSign = 0;
			}
		}
	}
}

