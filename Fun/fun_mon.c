#include "fun_mon.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_sys.h"

#include "bsp_wdg.h"
#include "bsp_io.h"
#include "bsp_storage.h"
#include "bsp_gps.h"

#include "pdu.h"
#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_power.h"
#include "bsp_rtc.h"
#include <stdio.h>
#include "bsp_uart_fifo.h"

#define PWRBREAKTIME 10000								//连续10电压为0 认为断电

static void funMonFtpProc(void);
static uint32_t pwrBreakTime = 0;

static uint8_t gps_AdjTime;
/*ftp服务器参数*/
static uint8_t ftpMode;
static char ftpHostName[40];
static uint16_t ftpHostPort;
static char ftpUserName[30];
static char ftpPassword[30];
static char ftpFilePath[50];
static char ftpFileName[30];
static void (*ftpcallback)(uint8_t type,void* parameter) = NULL;

void funMonInit(void)
{
	gps_AdjTime = 0;
	ftpMode = 0;
	osDelay(100);
	BSP_Ioctl(ST_BEEP_ID,OFF);
	if(bsp_first_start() == 1)
	{
		osDelay(100);
		whistle(100,100);whistle(100,100);whistle(100,100);
	}
	bsp_sleep_enable(BSP_ALLOW_SLEEP);
}
void funMonrun(void)
{
		//rtk_analysis(u8_rxRowVec);  
		
		//电压读取
		gTerminalState.batVolt = BSP_Get_ADC_Value(ADC_BAT_ID);
		gTerminalState.pwrVolt = BSP_Get_ADC_Value(ADC_VCC_ID);
		
		if(gTerminalState.pwrVolt > 3)
		{
			gTerminalState.powerState = 1;
			pwrBreakTime = osKernelGetTickCount();
		}
		else if(osKernelGetTickCount() - pwrBreakTime >PWRBREAKTIME)	//电压持续低于3V 认为主电源断电
		{
			gTerminalState.powerState = 0;
		}
		if(gTerminalState.batVolt < 3.8f)
				BSP_Ioctl(PWR_BAT_ID,ON);
		else if(gTerminalState.batVolt >= 4.1f)
				BSP_Ioctl(PWR_BAT_ID,OFF);
		//定位读取
		if(g_tGPS.PositionOk == 1)
		{
			gRealData.latd = g_tGPS.latd;
			gRealData.longd = g_tGPS.longd;
			gRealData.locationState = ((g_tGPS.NS == 'N') ? 0x00 : 0x02) | ((g_tGPS.EW == 'E') ? 0x00 : 0x04);
			if(g_tGPS.PositionOk == 1 && g_tGPS.Year >= 21 && g_tGPS.Month > 0 && g_tGPS.Day > 0  &&  g_tGPS.Sec > 5 &&  g_tGPS.Sec <= 55 && gps_AdjTime == 0)
			{
				RTC_INFOR gpsTime;
				gpsTime.year = g_tGPS.Year + 2000;
				gpsTime.month = g_tGPS.Month;
				gpsTime.day = g_tGPS.Day;
				gpsTime.hour = g_tGPS.Hour;
				gpsTime.minute = g_tGPS.Min;
				gpsTime.second = g_tGPS.Sec;
				UTC2CHINA(&gpsTime);//把UTC时间转为北京时间
				RTC_Time_Set(&gpsTime);
				gps_AdjTime = 1;
			}
		}
		else
		{
			gRealData.locationState |= 0x01;
		}
		//CAN指示
		if(fun_can_Get_State(BSP_CAN) > 0)
		{
			BSP_Ioctl(ST_CAN_ID,ON);
			switch(fun_can_Get_State(BSP_CAN))
			{
				case 1:osDelay(100);break;
				case 2:osDelay(1000);break;
				default:osDelay(1000);break;
			}
			BSP_Ioctl(ST_CAN_ID,OFF);
		}
		//GPS指示
		if(g_tGPS.PositionOk)
		{
			ledFlash(ST_GPS_ID,100);
		}
		else if(g_tGPS.antSta != 1)
		{
			ledFlash(ST_GPS_ID,50);ledFlash(ST_GPS_ID,50);ledFlash(ST_GPS_ID,50);
		}
		//GPRS指示
		if(Fun_Gprs_GetSta() >= FUN_GPRS_MON)
		{
			ledFlash(ST_GPRS_ID,100);
		}
		else if(Fun_Gprs_GetSta() <= FUN_GPRS_GET_SIM)
		{
			ledFlash(ST_GPRS_ID,50);ledFlash(ST_GPRS_ID,50);ledFlash(ST_GPRS_ID,50);
		}
		saveRecoverData();
		
		if(ftpMode == 1 &&(gRealData.chargeState != CARSTA_START || gRealData.speed == 0))
		{
			ftpMode = 0;
			funMonFtpProc();
		}	
		
		bsp_enter_sleep();					//进入睡眠
		osDelay(1000);
//		osDelay(1);

//		if(osKernelGetTickCount() > 30000)
//		{
//			funMonSetFtpParas("basic.oicp.io",21,"ftpusr","4yGY1!5%2021&","F4_HCXY_STD_V1.00/V5.00/V1.04","NEVT200-F4_Ftp.bin",NULL);
//			funMonFtpProc();
//		}
}

void funMonSetFtpParas(char *hostname,uint16_t hostport,char *username,char *password,char *filePach,char* fileName,void (*callback)(uint8_t type,void* parameter))
{
	strcpy(ftpHostName,hostname);
	ftpHostPort = hostport;
	strcpy(ftpUserName,username);
	strcpy(ftpPassword,password);
	strcpy(ftpFilePath,filePach);
	strcpy(ftpFileName,fileName);
	ftpcallback = callback;
	ftpMode = 1;
}

static void funMonFtpProc(void)
{
	if(Fun_Gprs_Ftp_connect(ftpHostName,ftpHostPort,ftpUserName,ftpPassword))
	{
		uint8_t callBackType = 0;
		uint8_t callBackPara = 0;
		uint32_t pos = 0;
		uint32_t size = Fun_Gprs_Ftp_GetFileSize(ftpFilePath,ftpFileName);
		if(size > 0)
		{
			USART_Cmd(USART3,DISABLE);	//关闭GPS串口
			BSP_Ioctl(CAN_STB_ID,OFF);	//CAN睡眠
			while(pos < size)
			{
				if(pos % 0x1000 == 0)
					BSP_Ioctl(ST_GPRS_ID,ON);
				else
					BSP_Ioctl(ST_GPRS_ID,OFF);
				WDG_Feed(2);									//喂狗
				//读取文件
				if(Fun_Gprs_Ftp_ReadFile(ftpFileName,pos,(size - pos) > 0x800? 0x800: (size - pos),szMainBuf) == 0)
				{
					//读取文件异常
					callBackType = 2;
					callBackPara = 1;
				}
				//写入文件
				else if(saveBinFile(pos,0x800,size,szMainBuf) == 0)
				{
					//写入文件异常
					callBackType = 2;							
					callBackPara = 2;
				}
				//升级成功
				else if(pos + 0x800 >= size)
				{
					callBackType = 1;
					callBackPara = 100;
				}
				else
				{
					callBackType = 0;
					callBackPara = 100 * (pos + 0x800) / size;
				}
				pos += (size - pos) > 0x800 ? 0x800: (size - pos);
				if(ftpcallback != NULL)
				{
					ftpcallback(callBackType,&callBackPara);
				}
				if(callBackType != 0)
				{
					if(callBackType == 1)
					{
						osDelay(3000);
						BoardReset();
					}
					break;
				}
			}
		}
	}
	Fun_Gprs_Ftp_disconnect();	
	USART_Cmd(USART3,ENABLE);	//关闭GPS串口
	BSP_Ioctl(CAN_STB_ID,ON);	//CAN睡眠
}





