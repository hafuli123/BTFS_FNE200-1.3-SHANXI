/*
文 件：analyse_Bms_HCXY_EX.c
功 能：地上铁-华晨鑫源 - 企标控制及自定义数据采集功能
日 期: 2022/2/17
公 司：北理新源(佛山)信息科技有限公司
作 者: LGC
*/

#include "HCXY_DST/protocol_GB_EX_HCXY.h"

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;		/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;					/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;	/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/
	
//STANDARD_TYPE 标准帧；EXTENDED_TYPE 扩展帧
static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
static void getCANLog_SendToHttps(uint8_t ch,CAN_msg *msg);			/* 采集CAN报文到https服务器 */
static uint8_t isOutLine = 0;												//是否网络异常
   
/*
功能：CAN数据接收、解析、发送

*/
void unpackDSTcan(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val = 0;
	uint32_t n32Val = 0;
	
	if(gUserDara.store_flag == 0xAA)		//用户数据有读取时再发送		
	{
        sendIntervalData(1);							//发送CAN数据功能接口						
	}
	

    
	if(pSelfData0A == NULL )
	{
		return;
	}
	switch(msg->id)
	{
		case 0x8039F80:
			{
					//限速等级	 -- 锁车状态
					n8Val = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n8Val == 1)
						pSelfData0A->sLockCarST  = 1;
					else if(n8Val == 2)
						pSelfData0A->sLockCarST  = 0;	
               }
      }	
}

#define OUTLINECSQ				8										//网络异常限速判定CSQ 小于8  
#define OUTLINEMILE				50									//网络异常限速判定里程 50km   
#define OUTLIMETIME				172800							//网络异常限速判定时间 48小时 = 172800

static uint8_t outLineLimitSpeedFun(uint8_t currCMD)
{
	uint8_t retCMD = currCMD;
	uint32_t outlineTimes = 0;
	
	//非一级限速 且非 库存融限速时执行 网络限速判定
	if(gUserDara.lockCMD != 0x55 && gUserDara.lockCMD != 0x60)
	{
		/* 网络异常限速 csq = 8 联网临界值 */
		if(Fun_Gprs_Csq() < OUTLINECSQ || Fun_Gprs_Csq() == 99 ||Fun_Gprs_GetSta() <= FUN_GPRS_GET_SIM)	
		{
			if(isOutLine == 1)																			//断网时间点参数更新
			{
				isOutLine = 0;
				gUserDara.outLineTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
				saveUserData();
			}		
			if((gRealData.totalMileage - gUserDara.outLineMile) > OUTLINEMILE)
			{
				if(g_system_dt.minute%2 == 0)
				{
					outlineTimes = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
					//if(outlineTimes - gUserDara.outLineTime >= 172800)		//大于48小时 172800秒
					if(outlineTimes - gUserDara.outLineTime >= OUTLIMETIME)		//大于48小时 172800秒
					{
						isOutLine = 2;
					}
				}			
				if(isOutLine == 2)																			//满足网络异常48小时触发一级限速
				{
					gUserDara.runCMD = 1;																	//禁止运营
					retCMD = 0;																						//限速一级
					pSelfData0A->sLockCarSign = 1;		
				}				
			}
		}
		else
		{
			isOutLine = 1;
			gUserDara.outLineMile = gRealData.totalMileage;						
		}
	
	}
	return retCMD;
}



/*******************************************************************************
功能: 地上铁华晨鑫源自定义功能 入口函数
描述：发送控制指令
*******************************************************************************/
void sendIntervalData(unsigned char canCh)
{
	uint8_t currCMD = 1,stockMode = 0;
	uint32_t tempCMD = 0;
	
	static uint32_t sendHeartBeatTime = 0;
	static uint8_t heartNum = 0;				//Life

	if(isEnableSendCANData() && osKernelGetTickCount() - sendHeartBeatTime >= 1000)//发送周期信息  0x348  100ms执行一次	
	{
		sendHeartBeatTime = osKernelGetTickCount();	
		memset(msg_buf.data,0,sizeof(msg_buf.data));

		//未限速
		if(gUserDara.lockCMD == 0xAA)	
		{
			gUserDara.runCMD = 0;																			//允许运营	
            currCMD = 1;
			pSelfData0A->sLockCarSign = 0;														//未限速															
		}
		else if(gUserDara.lockCMD == 0x60)													//库存融限速
		{
			currCMD = 0;																							//6：重启一级限速
			pSelfData0A->sLockCarSign = 1;																					
		}
		else if(gUserDara.lockCMD == 0x55)	
		{
			currCMD = 0;																							//1级
			pSelfData0A->sLockCarSign = 1;														
		}	
		else if(gUserDara.lockCMD == 0x56)
		{
			currCMD = 0;																							//2级
			pSelfData0A->sLockCarSign = 1;														
		}
		else if(gUserDara.lockCMD == 0x57)
		{
			currCMD = 0;																							//3级
			pSelfData0A->sLockCarSign = 1;														
		}
        
//        if(gUserDara.runCMD == 0)
//        {
//           msg_buf.data[0] = 1;
//        }
//        else
//        msg_buf.data[0] = 0;
   
        tempCMD |= currCMD;
        msg_buf.data[0] = tempCMD;
		msg_buf.id = 0x1801D0D8;
		
		startSendCANData(canCh,&msg_buf);
	}
}

char databuff[50] = {0};
uint8_t httpLinkSt = 0;								//https连接状态
uint32_t httpCanLogStartTime = 0;			//https采集开始时间
uint32_t sendlen = 0;
uint32_t sendlenss = 0;
uint8_t switchss = 1;
static void getCANLog_SendToHttps(uint8_t ch,CAN_msg *msg)
{
	if(getCanLogSwitch == 1)
	{
		getCanLogSwitch = 2;
		char fileName[30];
		sprintf(fileName,"%s-%02d%02d%02d%02d%02d.txt",gFrimPara.terminalId,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
		
		//开启http 上送
		if(switchss == 0)
			httpLinkSt = Fun_Gprs_Http_Post_File_Start("http://basic.oicp.io:19004/test/","usr","123",fileName,70000+30);
		else if(switchss == 1)
			httpLinkSt = Fun_Gprs_Http_Post_File_Start("https://adas-file.flybees.com.cn:20001/upload/d0cc3e0817758ff5/1648635072/C9F41CA22B01E63360DA5A6D7295A8D050BE41117613B67F0FA2A74F4806FB1F",NULL,NULL,fileName,70000+30);
		else
			httpLinkSt = Fun_Gprs_Http_Post_File_Start(gUserDara.httpURL,gUserDara.httpUserName,gUserDara.httpPassWord,fileName,70000+30);		
		sendlen = 0;
	}
	if(getCanLogSwitch == 2)
	{
		if(httpLinkSt < 3 && httpLinkSt > 0)
		{
			if(httpLinkSt == 1)
			{
				sprintf(databuff,"GetCANLogTime:%04d%02d%02d%02d%02d%02d\r\n",g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);				
				httpCanLogStartTime = osKernelGetTickCount();
				httpLinkSt = 2;
			}
			else
				sprintf(databuff,"%05d %03x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",(osKernelGetTickCount() - httpCanLogStartTime), msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
			sendlenss = Fun_Gprs_Http_Post_File_Input(databuff,strlen(databuff));
			sendlen+= strlen(databuff);
			if(sendlenss == 0 || sendlen > 71000)
			{
				getCanLogSwitch = 0;
				httpLinkSt = 0;
				sendlen = 0; 
			}

		}
		else
			getCanLogSwitch = 0;
	}	
}




/*static void getCANLOGs(uint8_t ch,CAN_msg *msg)
{
	uint8_t ret = 0;

	if(getCanLogSign == 1)
	{
		getCanLogSign = 2;
		CanLogTime = osKernelGetTickCount();
		memset(saveBuff,0,sizeof(saveBuff));
		sprintf(path,"M0:\\%02x\\%s%02d%02d%02d%02d.%s",0xA2,gFrimPara.terminalId,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second,"LOG");
		if(bsp_storage_state() > 0)
		{
			files = fopen(path, "ab");
			if(files)
			{
				if(ftell(files) == 0)
				{
					//日志头
					sprintf(databuff,"GetCANLogTime:%04d%02d%02d%02d%02d%02d\r\n",g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
					byteswritten = fwrite(databuff,1, strlen(databuff), files);		
				}
			}
		}	
	}
	if(getCanLogSign == 2 && bsp_storage_state() > 0)
	{		
		CanLogTime1 = osKernelGetTickCount();
		if(CanLogTime1>0)
			CanLogTime2 = CanLogTime1 - CanLogTime;
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
				//采集CAN日志
				sprintf(databuff,"%05d %03x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",(osKernelGetTickCount() - CanLogTime), msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
				memcpy(&saveBuff[saIndex],databuff,strlen(databuff));
				saIndex += strlen(databuff);
				if(saIndex + 150 > sizeof(saveBuff))
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
				getCanLogSign = 0;
				byteswritten = fwrite(saveBuff,1, strlen(saveBuff), files);	
				fflush(files);
				fclose(files);
				files = NULL;
				memset(saveBuff,0,sizeof(saveBuff));
				saIndex = 0;				
			}
		}
	}
}

*/



