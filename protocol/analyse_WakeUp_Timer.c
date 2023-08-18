/*
文 件：analyse_WakeUp_Timer.c
功 能：定时唤醒功能
日 期: 2022/7/8
公 司：北理新源(佛山)信息科技有限公司
作 者: LGC
*/

#include "analyse_WakeUp_Timer.h"

#define MAX_YZT_LINK 1


enum PARA_ID{
	PARA_WAKE_TIME = 0x9C,				//设置唤醒时间
	PARA_WAKE_INTERVAL = 0x9D,			//设置唤醒间隔
	PARA_WAKE_SENDTIME = 0x9E,			//设置唤醒后数据上送时间长度
	
	PARA_HTTPURL = 0xA2,				//httpURL
	PARA_HTTPUSERNAME = 0xA3,			//httpUserName
	PARA_HTTPPASSWORD = 0xA4,			//httppassword
	PARA_GETCANLOG = 0xA5,				//CAN日志开关
};

//掉电保存链路参数
typedef struct _CFG
{
	uint32_t canMsgSeq;					//can消息流水号
	uint32_t canLogSeq;					//can日志流水号
}CFG;

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;						//链路号
	uint8_t bUse;						//是否使用
	char *vin;							//车架号
	uint8_t bLogouted;					//登出标志位 0:未登出 1:已登出
	uint8_t sendOverTimeCnt;			//发送次数
	//内部时间戳
	uint32_t chgPowerStamp;				//充电电量上报时间戳
	//其他参数
	uint8_t* buff;						//发送数据缓冲区
	uint16_t buffLen;					//发送数据缓冲区长度
}GBSTA;

/* 用户自定义参数 */
static GBSTA gbSta[MAX_YZT_LINK] = {0};
USER_DATA gUserDara;					//用户数据存储

/* 联网控制参数 */
static uint32_t reConnectStamp = 0;		//重播计时
static uint8_t printPackCmd = 1;		//打印报文到串口控制位

/* 休眠唤醒参数 */
static int wakelock = -1;				//唤醒锁
static uint64_t wakeTime = 0;			//唤醒时间
static uint32_t wk_timesss = 0;			//唤醒倒计时
uint8_t wk_Excute = 0;					//唤醒标志控制
uint8_t allow_UserOnLine = 0;			//允许用户平台联网

/* 远程CAN日志参数 */
uint8_t getCanLogSwitch = 0;			//远程CAN日志采集开关

/*
功能：存储用户数据
	存储锁车指令相关数据
*/
void saveUserData(void)
{
	gUserDara.store_flag = 0xAA;
	User_ID_Save(&gUserDara,sizeof(USER_DATA));				
}

/*
功能：唤醒回调函数
描述：	返回1 当下不允许睡眠，需休眠倒计时后再进入休眠
*/
static uint8_t wakeCallback(uint8_t type)		
{
	wk_Excute = 1;
	wk_timesss = osKernelGetTickCount();
	allow_UserOnLine = 1;
	bsp_wakelock_lock(wakelock);
	
	autoChgState = 0;								//默认补电条件不满足
	sAllowAutoChg = 1;							//默认VCU允许补电
	return 1;
}
/*
功能：唤醒连平台倒计时
描述：	没有自动补电时，到达超时时间，或有CAN数据
			休眠唤醒后，有唤醒源，激活管理平台重连
*/
static void wk_lineTime(void)
{
	if(wk_Excute && autoChgState == 0)		//非补电状态下
	{		
		//数据上报超时 || 有唤醒源
		if(osKernelGetTickCount() - wk_timesss > gUserDara.wk_SendInterval*10000)
		{
			if(bsp_wakelock_unlock(wakelock) == BSP_SLEEP_SUCCESS)
			{
				wk_Excute = 0;
				allow_UserOnLine = 0;											
			}
			printfData("wk_lineTime over\r\n");
		}
	}
}

/*
功能：设置定时唤醒时间 函数1
描述：设置定时唤醒时间，和唤醒间隔
时间格式：2022:3:12-16:55:46
唤醒间隔：秒				24小时 = 86400s
*/
static uint8_t isSetFirstV = 0;
static void setWakeTime(void)
{
	uint32_t temp = 0;
	temp = (uint8_t)gUserDara.wk_SendInterval;
	if(temp == 0||temp > 250)
		gUserDara.wk_SendInterval = 6;	
	
	temp = (uint32_t)gUserDara.wk_Interval;
	if(temp > 84600 || temp == 0)
		gUserDara.wk_Interval = 21600;
	
	temp = (uint16_t)gUserDara.wk_Year;
	if((temp<2022 || temp > 2060))
	{
		gUserDara.wk_Year = g_system_dt.year;
		gUserDara.wk_Month = g_system_dt.month;
		gUserDara.wk_Day = g_system_dt.day;
		gUserDara.wk_Hour = g_system_dt.hour;
		gUserDara.wk_Min = g_system_dt.minute;
		gUserDara.wk_Sec = g_system_dt.second;
		/* 默认定时时间 */
		wakeTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
		/* 默认6小时 */
		bsp_wakelock_para(wakelock,wakeTime,gUserDara.wk_Interval,wakeCallback,0,0);	
		isSetFirstV = 1;
	}
	else
	{
		/* 转换定时唤醒时间 */
		wakeTime = RTC_mktime(gUserDara.wk_Year,gUserDara.wk_Month,gUserDara.wk_Day,gUserDara.wk_Hour,gUserDara.wk_Min,gUserDara.wk_Sec);
		/* 设置定时唤醒时间何和周期 */
		bsp_wakelock_para(wakelock,wakeTime,gUserDara.wk_Interval,wakeCallback,0,0);	
	}
	if(isSetFirstV==1)
	{
		saveUserData();
		isSetFirstV = 0;
	}
}


/*
功能：定时唤醒时间初始 函数2
描述：
	24小时 = 86400s
*/
static void set_WakeTime2(uint8_t swID)								//设置定时唤醒时间
{
	if(gUserDara.wk_Sec != g_system_dt.second && g_system_dt.second %2 == 0)	
	{		
		gUserDara.wk_Sec = g_system_dt.second;
		
		if(swID == 1)				//开启定时唤醒
		{
			uint32_t temp = 0;
			temp = (uint8_t)gUserDara.wk_SendInterval;
			if(temp == 0||temp > 250)
				gUserDara.wk_SendInterval = 6;	
			
			temp = (uint32_t)gUserDara.wk_Interval;
			if(temp > 84600 || temp == 0)
				gUserDara.wk_Interval = 21600;		
																																												/* 默认定时时间 */
			wakeTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);	
			bsp_wakelock_para(wakelock,wakeTime,gUserDara.wk_Interval,wakeCallback,0,0);			/* 默认6小时 */
		}
		else 								//关闭定时唤醒
		{
			bsp_wakelock_para(wakelock,0,0,0,0,0);				//关闭定时唤醒 进入硬件休眠
		}
	}
}

/*
功能：发送数据到网络
*/
static uint8_t sendNetDatas(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	uint16_t sendLen = (buf[22] << 8 | buf[23] << 0) + 25;
	if(printPackCmd == 1)
	{
		comSendBuf(RS1_COM,buf,len);
	}
	Fun_Gprs_Tcp_send(socket_fd,buf,sendLen);
	return 0;
}

/*
功能：扩展协议初始化
描述：返回配置状态参数
*/
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	
	/* 读取用户参数 */
	if(gUserDara.store_flag != 0xAA)
	{
		User_ID_Read(&gUserDara,sizeof(USER_DATA));		
		gUserDara.store_flag = 0xAA;		
	}
	
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bUse == 1 && gbSta[i].bLink == link)
		{
			oldLinkIdx = i;//重新初始化
		}
		if(gbSta[i].bUse == 0 && objLinkIdx == 0xFF)
		{
			objLinkIdx = i;
		}
	}
	if(oldLinkIdx != 0xFF)
	{
		objLinkIdx = oldLinkIdx;
	}
	if(objLinkIdx == 0xFF)
		return NULL;
	//外部参数
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].buff = buff;
	gbSta[objLinkIdx].buffLen = buffLen;
	//内部
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].chgPowerStamp = 0;
	
	reConnectStamp = 0;
	reportVINStamp = 0;

	wakelock = bsp_wakelock_create();			/* 创建唤醒锁 */	
	/*setWakeTime();							设置唤醒时间 */
	
	return &gbSta[objLinkIdx];
}

/*
功能：扩展协议运行   
描述：	
		ctrl:0 网络离线 1:网络在线 2:睡眠 
		sta:0 国标未登入 1:国标已登入 
返回值:
		0 扩展协议未登入 1:扩展协议已登入
*/
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	if(obj == NULL)
		return 1;
	uint16_t dataLen = 0;
	lowVoltProtect();																	//电压判断

	if(pSta->bLink == 0)
	{
		if(autoChgSW == 1)
		{
			if(wk_Excute ==1 && pSelfData0A->sTboxST == 2)	//定时的休眠唤醒，才判断是否补电											
				autoChgPower();			
		}
		
		localtionFaultFun();														//定位异常判断
			
		wk_lineTime();

		if(pSta == NULL)																//链路未配置，返回运行成功
		{ 
			return 1;
		}
		else if(ctrl == 0 || ctrl == 2)									//掉线或睡眠
		{
			return 0;
		}
		if(sta == 1)
		{
			if(pSta->sendOverTimeCnt >= 3 &&  osKernelGetTickCount() - reConnectStamp >= 30000)
			{
				reConnectStamp = osKernelGetTickCount();
				pSta->sendOverTimeCnt = 0;
				Fun_Gprs_Tcp_disconnect(pSta->bLink);					//连续3次超3秒响应超时,重播
			}
			//首次获取VIN时，10s上送一次直至接收到平台响应
			else if( fisrtGetVIN == 1 && osKernelGetTickCount() - reportVINStamp >= reportVINInterval)	
			{
				reportVINStamp = osKernelGetTickCount();
				if((dataLen = packUpdataVIN(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
				{
					sendNetDatas(pSta->bLink,pSta->buff,dataLen);
					reportVINfaildCnt++;
					if(reportVINfaildCnt == 3)
						reportVINInterval = 1800000;						//连续3次上报不成功改成30分钟上报一次
				}
			}		
			else if(chgPowerSign == 1 && osKernelGetTickCount() - pSta->chgPowerStamp >= reportChgPInterval)	
			{		
				//充电电量上报，10s上送一次直至接收到平台响应
					pSta->chgPowerStamp = osKernelGetTickCount();
					if((dataLen = packCHGPower(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
					{
						sendNetDatas(pSta->bLink,pSta->buff,dataLen);		
						reportChgPfaildCnt++;
						if(reportChgPfaildCnt == 3)
							reportChgPInterval = 1800000;						//连续3次上报不成功改成30分钟上报一次
					}					
			}			
		}
		else
		{
			//登出后重置 VIN码变更的控制参数
			isChgVIN = 0;
			reportVINfaildCnt = 0;
			reportVINInterval = 10000;
		}
	}
	return 1;
}
/*
功能：企标控制联网上报
描述：
			定时唤醒后，需要联网操作。
*/
uint8_t extCtrlNetSta(void* obj)
{
	GBSTA *pSta = obj;
	if(obj == NULL)
		return 0;
	
	if(pSta->bLink == 0)
		return allow_UserOnLine;
	else 
		return 0;
	
	return allow_UserOnLine;
}
uint8_t weakCtrlNetSta(void)
{
	return allow_UserOnLine;
}

/*
功能：扩展协议接收解包
描述：
*/
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i,ret = 1;
	GBSTA* pSta = NULL;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bLink == link && gbSta[i].bUse == 1)
		{
			pSta = &gbSta[i];
			break;
		}
	}
	if(pSta == NULL)//未配置此链路
		return 0;				

	switch(szRecvBuf[2])
	{		
		case CMD_LOCKCMD:								
			{
				UnpackLockCMD(link,pSta->vin,szRecvBuf,rLen);								//锁车控制指令(限速)
			}
			break;
		case CMD_SOFTVER_CHANGE:				
			{
				UnpackSoftVer_Change(link,pSta->vin,szRecvBuf,rLen);				//程序版本切换控制	
			}
			break;
		case CMD_BINDINGCTR:						
			{
				UnpackBindingCtl(link,pSta->vin,szRecvBuf,rLen);						//绑定控制
			}
			break;
		case CMD_DOORCTR:								
			{
				UnpackDoorCtl(link,pSta->vin,szRecvBuf,rLen);								//车门控制
			}
			break;
		case CMD_MAINTTIPS:							
			{
				UnpackMaintTips(link,pSta->vin,szRecvBuf,rLen);							//保养提示
			}
			break;
		case CMD_PAYTIPS:								
			{
				UnpackPayTips(link,pSta->vin,szRecvBuf,rLen);								//缴费提示
			}
			break;
		case CMD_CHECKTIPS:							
			{
				UnpackYearCheckTips(link,pSta->vin,szRecvBuf,rLen);					//年审提示
			}
			break;
		case CMD_CHGCMD:																								//充电电量上送响应
			{
				if(szRecvBuf[3] == 0x01)
				{
					chgPowerSign = 2;																					//充电响应成功
				}
			}
			break;
		case CMD_SENDVINCMD:																						//首次获取VIN上送响应
			{
				if(szRecvBuf[3] == 0x01)
				{
					fisrtGetVIN	 = 0;																					//响应成功后不进行上报
				}			
			}
			break;
		default: 
			ret = 0;
			break;
	}
	return ret;
}

uint8_t extSetPara(uint8_t check,uint8_t setParaId,const uint8_t* setBuff)
{
	uint8_t setlen = 0;
	char wktimes[50] = {0};
	switch(setParaId)
	{
		case PARA_WAKE_TIME:				//设置唤醒时间
			if(check == 1)
			{
				memset(wktimes,0,sizeof(wktimes));
				memcpy(wktimes,&setBuff[setlen],strlen((char*)&setBuff[setlen]));			
				sscanf(wktimes,"%d:%d:%d-%d:%d:%d",(int*)&gUserDara.wk_Year,(int*)&gUserDara.wk_Month,(int*)&gUserDara.wk_Day,(int*)&gUserDara.wk_Hour,(int*)&gUserDara.wk_Min,(int*)&gUserDara.wk_Sec);	
				
				/* 设置唤醒时间
				setWakeTime();	 */
			}
			setlen += strlen((char*)&setBuff[setlen]) + 1;
			break;
		case PARA_WAKE_INTERVAL:		//设置唤醒间隔
			if(check == 1)
			{
				memset(wktimes,0,sizeof(wktimes));
				memcpy(wktimes,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				
				sscanf(wktimes,"%ld",(long*)&gUserDara.wk_Interval);
				/* 设置唤醒时间
				setWakeTime(); */
			}
			setlen += strlen((char*)&setBuff[setlen]) + 1;
			break;
		case PARA_WAKE_SENDTIME:		//设置唤醒后上送数据时长
			if(check == 1)
			{
				gUserDara.wk_SendInterval = setBuff[setlen];
			}
			setlen ++;
			break;
		case PARA_AUTOCHG_VOLT:				//需自动补电电压
			{
				if(check == 1)
				{
					uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgVolt);			
					pFval[0] = setBuff[setlen+0];
					pFval[1] = setBuff[setlen+1];
					pFval[2] = setBuff[setlen+2];
					pFval[3] = setBuff[setlen+3];
				}
				setlen+=4;
			}
			break;
		case PARA_AUTOCHGOVER_VOLT:		//停止自动补电电压
			{
				if(check == 1)
				{
					uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgOverVolt);			
					pFval[0] = setBuff[setlen+0];
					pFval[1] = setBuff[setlen+1];
					pFval[2] = setBuff[setlen+2];
					pFval[3] = setBuff[setlen+3];
				}
				setlen+=4;
			}
			break;
		case PARA_UNDERVOLT:					//欠压值
			{
				if(check == 1)
				{
					uint8_t* pFval = (uint8_t*)(&gUserDara.underVolt);			
					pFval[0] = setBuff[setlen+0];
					pFval[1] = setBuff[setlen+1];
					pFval[2] = setBuff[setlen+2];
					pFval[3] = setBuff[setlen+3];
				}
				setlen+=4;
			}
			break;
		case PARA_HTTPURL:
			{
				if(check == 1)
				{
					memcpy(gUserDara.httpURL,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				}
				setlen += strlen((char*)&setBuff[setlen]) + 1;	
			}
			break;
		case PARA_HTTPUSERNAME:
			{
				if(check == 1)
				{
					memcpy(gUserDara.httpUserName,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				}
				setlen += strlen((char*)&setBuff[setlen]) + 1;	
			}
			break;
		case PARA_HTTPPASSWORD:
			{
				if(check == 1)
				{
					memcpy(gUserDara.httpPassWord,&setBuff[setlen],strlen((char*)&setBuff[setlen]));
				}
				setlen += strlen((char*)&setBuff[setlen]) + 1;	
			}
			break;
		case PARA_GETCANLOG:
			{
				if(check == 1)
					getCanLogSwitch = setBuff[setlen];
				setlen++;
			}
			break;
	}
	//保存用户数据
	saveUserData();			
	return setlen;
}

/*
功能：终端管理平台读取终端参数信息
描述：
*/
uint8_t extReadPara(uint8_t readParaId,uint8_t* readBuff)
{
	uint8_t readlen = 0;
	char wktimes[50] = {0};
	switch(readParaId)
	{
		case PARA_WAKE_TIME:				//查询唤醒时间
			{
				memset(wktimes,0,sizeof(wktimes));
				sprintf(wktimes,"%d:%d:%d-%d:%d:%d",gUserDara.wk_Year,gUserDara.wk_Month,gUserDara.wk_Day,gUserDara.wk_Hour,gUserDara.wk_Min,gUserDara.wk_Sec);	
				memcpy(&readBuff[readlen],wktimes,strlen((char*)wktimes));
				readlen += strlen(wktimes);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_WAKE_INTERVAL:		//查询唤醒间隔
			{
				memset(wktimes,0,sizeof(wktimes));
				sprintf(wktimes,"%d",gUserDara.wk_Interval);	
				memcpy(&readBuff[readlen],wktimes,strlen((char*)wktimes));
				readlen += strlen(wktimes);
				readBuff[readlen++] = 0;		
			}
			break;
		case PARA_WAKE_SENDTIME:		//查询唤醒后上送数据时长
			readBuff[readlen++] = gUserDara.wk_SendInterval;	
			break;
		case PARA_AUTOCHG_VOLT:				//需自动补电电压
			{
				uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgVolt);						
				readBuff[readlen++] = pFval[0];
				readBuff[readlen++] = pFval[1];
				readBuff[readlen++] = pFval[2];
				readBuff[readlen++] = pFval[3];
			}
			break;
		case PARA_AUTOCHGOVER_VOLT:		//停止自动补电电压
			{
				uint8_t* pFval = (uint8_t*)(&gUserDara.autoChgOverVolt);			
				readBuff[readlen++] = pFval[0];
				readBuff[readlen++] = pFval[1];
				readBuff[readlen++] = pFval[2];
				readBuff[readlen++] = pFval[3];
			}
			break;
		case PARA_UNDERVOLT:					//欠压值
			{
				uint8_t* pFval = (uint8_t*)(&gUserDara.underVolt);			
				readBuff[readlen++] = pFval[0];
				readBuff[readlen++] = pFval[1];
				readBuff[readlen++] = pFval[2];
				readBuff[readlen++] = pFval[3];
			}
			break;
		case PARA_HTTPURL:						//httpURL
			{
				memcpy(&readBuff[readlen],gUserDara.httpURL,strlen(gUserDara.httpURL));
				readlen += strlen(gUserDara.httpURL);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_HTTPUSERNAME:				//httpUserName
			{
				memcpy(&readBuff[readlen],gUserDara.httpUserName,strlen(gUserDara.httpUserName));
				readlen += strlen(gUserDara.httpUserName);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_HTTPPASSWORD:				//httpPassWord
			{
				memcpy(&readBuff[readlen],gUserDara.httpPassWord,strlen(gUserDara.httpPassWord));
				readlen += strlen(gUserDara.httpPassWord);
				readBuff[readlen++] = 0;
			}		
			break;
		case PARA_GETCANLOG:
			readBuff[readlen++] = getCanLogSwitch;	
			break;
	}	
	return readlen;
}

/*
功能：扩展实时数据
描述：实时数据增加自定义数据
*/
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	GBSTA *pSta = obj;
	uint16_t index = 0;
	if(pSta->bLink == 0)
	{
		index += Pack0AData(pRealData,&bPackBuf[index]);
	}
	return index;
}

/*
功能：数据上送时获取VIN
描述：以VIN为上送识别码，或者终端编号补零，或者设备号时，填充对应的信息。
*/
uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
	
	return 0;
}
 
/* 
功能：数据包头
描述：添加数据报头及校验,返回值 
*/
static uint16_t MakeCmd(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *wbuf,uint16_t dataLen)
{
	wbuf[0] = 0x23;
	wbuf[1] = 0x23;
	wbuf[2] = cmd;
	wbuf[3] = rsp;
	memcpy(&wbuf[4],vin,17);
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen >> 0;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

/*
功能：打包自定义 0x0A数据
描述：
*/
static uint16_t Pack0AData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	uint8_t temp8 = 0,ipLen = 0;
	uint16_t temp16 = 0;
	
	bPackBuf[0] = 0x0A;
	SelfData0A*  p0AData = (SelfData0A*)&pRealData->externData[p0Aoffset];		
	
	//填充自定义数据
	bPackBuf[index++] = 0x03;																							//设备型号编号  原厂标配
	bPackBuf[index++] = (gTerminalState.pwrVolt > 3 ? 0:1);								//设备供电标识  1 表示内部电池供电，0表示是外部电池供电
	
	memcpy(&bPackBuf[index],&getCarType()[strlen(getCarType())-5],5);			//TBOX固件版本号
	bPackBuf[index+2] = '0';
	index+=5;
	
	bPackBuf[index++] = 0xFF;//p0AData->sRelayST;													//继电器状态
	bPackBuf[index++] = 0xFF;//p0AData->sTermBindingST;										//终端绑定状态
	bPackBuf[index++] = p0AData->sVCUVerST;																//车辆控制器版本状态
	bPackBuf[index++] = p0AData->sLockCarST;															//锁车状态（指终端接收平台的锁车指令）
	
	if(chgPower != 0xFFFF || chgPower != 0xFFFE)
		temp16 = chgPower*100;
	bPackBuf[index++] = (uint8_t)(temp16>>8);
	bPackBuf[index++] = (uint8_t)(temp16>>0);															//本次充电电量
		
	memset(&bPackBuf[index],0xFF,8);																			//VCU版本号 p0AData->sVCUVerNum	
	index += 8;

	bPackBuf[index++] = p0AData->sQorS_CHGSign;														//车辆快慢充标志
	bPackBuf[index++] = p0AData->sLockCarSign;														//锁车标志(实车反馈当前执行的锁车指令)
	bPackBuf[index++] = 0;//p0AData->sFaultCT;														//故障数
	bPackBuf[index++] = (uint8_t)(p0AData->sCarType>>8);									//车型识别	
	bPackBuf[index++] = (uint8_t)(p0AData->sCarType>>0);	

	bPackBuf[index++] = p0AData->sTboxST;																	//Tbox工作状态
	bPackBuf[index++] = (BSP_Iostatus(IN_ACC_ID) == ON);									//车辆ON信号	(ACC)			0：无信号； 1：有信号
	bPackBuf[index++] = (gRealData.chargeState == STOP_CHARGE || gRealData.chargeState == CHARGE_FINISH)?1:0;																//车辆CHARGE信号
	bPackBuf[index++] = (fun_can_Get_State(BSP_CAN)>0 ? 1:0);							//Tbox CAN 唤醒状态
	bPackBuf[index++] = (fun_can_Get_State(BSP_CAN)>0 ? 1:0);							//Tbox CAN0-3 状态（0：无数据；1：有数据）
	
	if(gTerminalState.pwrVolt != 0xFFFF && gTerminalState.pwrVolt != 0xFFFE)
			temp16 = (uint16_t)(gTerminalState.pwrVolt*1000);		
	bPackBuf[index++] = (uint8_t)(temp16>>8);																//车辆小电瓶电压		
	bPackBuf[index++] = (uint8_t)(temp16>>0);
	
	if(gTerminalState.batVolt != 0xFFFF && gTerminalState.batVolt != 0xFFFE)
			temp16 = (uint16_t)(gTerminalState.batVolt*1000);	
	bPackBuf[index++] = (uint8_t)(temp16>>8);																										//Tbox内部电池电压
	bPackBuf[index++] = (uint8_t)(temp16>>0);	
	
	bPackBuf[index++] = 0;	
	memset(&bPackBuf[index],0,60);
	ipLen += sprintf((char*)&bPackBuf[index],"%s:%d",gSysPara.domain[1],gSysPara.port[1]);			//双链路IP端口	
	bPackBuf[index - 1] = ipLen;
	index += ipLen;
	
	bPackBuf[index++] = ((gSysPara.linkSwitch & 0x02)>0 ? 1:0);																	//双链路状态
	bPackBuf[index++] = (gUserDara.maintCMD>=0x55 && gUserDara.maintCMD <= 0x5C ? 1:0);					//保养提示
	bPackBuf[index++] = (gUserDara.payCMD>=0x55 && gUserDara.payCMD <= 0x5D ? 1:0);							//缴费提示
	bPackBuf[index++] = (gUserDara.yearCheckCMD>=0x55 && gUserDara.yearCheckCMD <= 0x58 ? 1:0);	//年审提示	
	
	bPackBuf[index++] = Fun_Gprs_Csq();																													//TBOX 网络信号

	if(g_tGPS.antSta == 1)
		temp8 = 0;		//正常
	else if(g_tGPS.antSta == 2)
		temp8 = 2;		//短路
	else if(g_tGPS.antSta == 3 || isLocFault == 1)
		temp8 = 1;		//断路
	bPackBuf[index++] = temp8;																						//Tbox GPS 工作状态
	
	bPackBuf[index++] = (bsp_storage_state() > 0 ? 0:1);									//Tbox EMMC 工作状态  0：正常  1：异常
	bPackBuf[index++] = 0xFF;//(uint8_t)(p0AData->sAirCurr>>8);						//空调反馈电流
	bPackBuf[index++] = 0xFF;//(uint8_t)(p0AData->sAirCurr>>0);	

	bPackBuf[index++] = 0xFF;//p0AData->sPumpFault;												//水泵故障	
	bPackBuf[index++] = 0xFF;//p0AData->sVacuumPumpST;										//真空泵状态
	bPackBuf[index++] = 0xFF;//p0AData->sVacuumValue;											//真空度
	
	bPackBuf[index++] = 0xFF;//p0AData->sPTCST;														//PTC 工作状态
	bPackBuf[index++] = 0xFF;//p0AData->sPTCRelayST;											//PTC继电器实际状态
	bPackBuf[index++] = 0xFF;//p0AData->sBatRelayST;											//动力电池加热继电器状态
	bPackBuf[index++] = 0xFF;//p0AData->sQ_CHG_Fault;											//快充继电器粘连故障
	bPackBuf[index++] = 0xFF;//p0AData->sAirWorkST;												//空调工作状态
	bPackBuf[index++] = 0xFF;//p0AData->sBreakST;													//手刹状态

	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}


/* 
功能：计算充电电量
描述：刚充电时记录充电起始时间，充电结束后记录充电结束时间。
			(volt*(10000.0-curr)*t_ms)/360000000000.0 (1000秒/3600时/1000千瓦/100分子放大系数);
			计算充电量
*/ 
void calculCHGPower(void)
{
	//开始充电
	if(chgStaST == 0 && gRealData.chargeState == STOP_CHARGE && gRealData.soc < 100)					
	{
		chgStaST = 1;
		chgPowerSign = 0;																					//充电量 上报重置
		chgPower = 0;																							//充电量重置
		chgStartTime = osKernelGetTickCount();										//充电时间重置
		memcpy(&chgStartTimes,&g_system_dt,sizeof(g_system_dt));	//记录充电起始时间
	}
	if(chgStaST == 1)
	{
		if(osKernelGetTickCount() - chgStartTime > 500)							//计算间隔
		{
			//结束充电
			if(gRealData.chargeState == CHARGE_FINISH ||(gRealData.chargeState == STOP_CHARGE && gRealData.soc == 100))
			{
				chgPowerSign = 1;		
				chgStaST = 0;
				memcpy(&chgEndTimes,&g_system_dt,sizeof(g_system_dt));
			}
			
			//充电未结束 - 计算充电量累加和
			chgPower += (gRealData.total_volt * (-gRealData.total_current))*(osKernelGetTickCount() - chgStartTime)/1000/1000/3600;			
			//chgPower += (gRealData.total_volt * (-gRealData.total_current))/1000/3600;			
			chgStartTime = osKernelGetTickCount();
		
		}
	}	
}

/* 
功能：打包充电电量
描述：
		每次充电完成后，登出前上送一次充电电量，
		充电完成无网络时，等下次正常登入时上送
*/
static uint16_t packCHGPower(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen)
{
	uint16_t valueTemp = 0;
	uint8_t index = 26;	
	
	buff[index++] = (uint8_t)(chgStartTimes.year - 2000);
	buff[index++] = (uint8_t)(chgStartTimes.month);
	buff[index++] = (uint8_t)(chgStartTimes.day);
	buff[index++] = (uint8_t)(chgStartTimes.hour);
	buff[index++] = (uint8_t)(chgStartTimes.minute);
	buff[index++] = (uint8_t)(chgStartTimes.second);
	
	buff[index++] = (uint8_t)(chgEndTimes.year - 2000);
	buff[index++] = (uint8_t)(chgEndTimes.month);
	buff[index++] = (uint8_t)(chgEndTimes.day);
	buff[index++] = (uint8_t)(chgEndTimes.hour);
	buff[index++] = (uint8_t)(chgEndTimes.minute);
	buff[index++] = (uint8_t)(chgEndTimes.second);

	if(chgPower != 0xFFFF || chgPower != 0xFFFE)
		valueTemp = chgPower*100;
	buff[index++] = (uint8_t)(valueTemp>>8);
	buff[index++] = (uint8_t)(valueTemp>>0);

	buff[24] = (index-26)>>8;
	buff[25] = (index-26)>>0;
	
	return MakeCmd(CMD_CHGCMD,0xFE,vin,buff,index-24);
}

/* 
功能：打包VIN变更信息
描述：
*/
static uint16_t packUpdataVIN(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen)
{	
		uint32_t dwVal = 0;		
		uint8_t index = 26;	
		buff[index++] = 0x03;								//Tbox设备型号
		buff[index++] = 0x00;								//整车类型
		buff[index++] = 0x03;								//整车类型
		
		//首次时间
		buff[index++] = (uint8_t)(getVINTime.year - 2000);
		buff[index++] = (uint8_t)(getVINTime.month);
		buff[index++] = (uint8_t)(getVINTime.day);
		buff[index++] = (uint8_t)(getVINTime.hour);
		buff[index++] = (uint8_t)(getVINTime.minute);
		buff[index++] = (uint8_t)(getVINTime.second);	
		//首次经度
		dwVal = (uint32_t)(glongd * 1000000);    
		buff[index++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		buff[index++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		buff[index++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		buff[index++] = (uint8_t)dwVal;
		//首次纬度
		dwVal = (uint32_t)(glatd * 1000000);  
		buff[index++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		buff[index++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		buff[index++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		buff[index++] = (uint8_t)dwVal;
		//ICCID
		Fun_Gprs_getICCID((char*)&buff[index],20);
		index += 20;

		memcpy(&buff[index],realVin,17);
		index += 17;
		
		buff[24] = (index-26)>>8;
		buff[25] = (index-26)>>0;
	
	return MakeCmd(CMD_SENDVINCMD,0xFE,vin,buff,index-24);
}

/*
功能：终端低电保护功能
描述：
			全运行周期检测电压，检测欠压后，关闭定时唤醒功能
			进入硬件休眠状态。
*/
static uint32_t printTime = 0;
static void lowVoltProtect(void)
{	
	if(autoChgState == 0) 											//自动补电阶段不做判断
	{
		if(gTerminalState.pwrVolt <= 5 )
		{
			pSelfData0A->sTboxST = 4;						//断开
		}
		else if(gTerminalState.pwrVolt <= gUserDara.underVolt - 0.2f && gTerminalState.pwrVolt >5)
		{
			pSelfData0A->sTboxST = 3;						//欠压
		}
		else if(!(fun_can_Get_State(BSP_CAN) > 0 && (gRealData.carState == 1 || gRealData.chargeState == 1)))		
		{
			pSelfData0A->sTboxST = 2;						//休眠
		}
		else
		{
			pSelfData0A->sTboxST = 1;						//正常
		}
	}
	//打印欠压状态到串口
	if(pSelfData0A->sTboxST >= 2 && osKernelGetTickCount() - printTime>10000)
	{
		char tempPrintfBuff[50] = {0};
		printTime = osKernelGetTickCount();
		sprintf(tempPrintfBuff,"MainVolt = %f,break:4,low:3, result=%d\r\n",gTerminalState.pwrVolt,pSelfData0A->sTboxST);
		printfData(tempPrintfBuff);
	}
	
	//断电或欠压
	if(pSelfData0A->sTboxST == 3||pSelfData0A->sTboxST == 4)
	{
		set_WakeTime2(0);															//关闭定时唤醒 进入硬件休眠
	}
	else
	{
		set_WakeTime2(1);															//欠压或解除，恢复自动唤醒	
	}										

}

/*
功能：自动补电引脚控制
描述：只在睡眠唤醒中开启自动补电功能，开启补电过程中，终端一直发硬线唤醒信号；
*/
#define noCANDataTime 	 						60000						//最低补电时间 2分
#define autoCHgLowTime 	 						1800000					//最低补电时间 30分
#define autoCHgHighTime 						10800000				//最高补电时间 3小时
static uint32_t autoChgTotalTime = 0;								//补电起时间
static float autoChgPreVolt = 0;										//补电起始电压
static void autoChgPower(void)
{
		uint8_t isStartAutoChgSign = 0;			

		//补电条件判断   补电范围（大于欠压值 小于等于补电值
		if(autoChgState == 0 && gTerminalState.pwrVolt <= gUserDara.autoChgVolt-0.2f && gTerminalState.pwrVolt > gUserDara.underVolt)
		{
			printfData("start autoChg\r\n");
			autoChgState = 1;																			//补电逻辑开启
			autoChgTotalTime = osKernelGetTickCount();						//起始点时间
			autoChgPreVolt = gTerminalState.pwrVolt;							//起始点电压
		}	
		
		/* 补电判定 */
		if(autoChgState == 1)
		{
			//补电结束条件1：补电完成（大于完成电压 && 补电超时30分）
			if(gTerminalState.pwrVolt >= gUserDara.autoChgOverVolt && osKernelGetTickCount()-autoChgTotalTime >= autoCHgLowTime)
			{
				isStartAutoChgSign = 0;									
			}
			//补电结束条件2：补电超时（小于完成电压 && 超时3小时）
			else if(gTerminalState.pwrVolt < gUserDara.autoChgOverVolt && osKernelGetTickCount()-autoChgTotalTime >= autoCHgHighTime)
			{
				isStartAutoChgSign = 0;									
			}
			//补电结束条件3：补电异常（补电10s后，电压不增反减）
			else if(osKernelGetTickCount()-autoChgTotalTime > 10000 && autoChgPreVolt > gTerminalState.pwrVolt+1)		
			{
				isStartAutoChgSign = 0;									
			}
			//补电结束条件4：补电异常（触发补电3分钟后，线束被拔无CAN）
			else if(osKernelGetTickCount()-autoChgTotalTime >= noCANDataTime && fun_can_Get_State(BSP_CAN) == 0)
			{
				isStartAutoChgSign = 0;			
			}
			//继续补电
			else
			{
				isStartAutoChgSign = 1;			
			}
		
			/* 补电指令发送 */		
			if(osKernelGetTickCount() - autoChgTotalTime < 3000 || sAllowAutoChg == 1)
			{
				if(isStartAutoChgSign == 1)
				{
					BSP_Ioctl(AUTO_CHG_ID,ON);			
					if(osKernelGetTickCount() - autoChgTotalTime < 3000)
						sendIntervalData(1);					//强制发3s		前3s主动发送补电请求   
				}
				else 
				{
					autoChgState = 0;								//主动结束补电  补电结束条件满足
					sAllowAutoChg = 0;
					BSP_Ioctl(AUTO_CHG_ID,OFF);
					printfData("autoStop autoChg\r\n");
				}			
			}
			else																//被动结束补电  3s后  VCU不允许补电
			{
				if(isStartAutoChgSign == 1)
				{					
					autoChgState = 0;
					isStartAutoChgSign = 0;
					BSP_Ioctl(AUTO_CHG_ID,OFF);
					printfData("forceStop autoChg\r\n");					
				}
			}
		}		
}

/*
功能：定位异常报警
描述：无效定位超过50km时，为定位异常报警
*/
static void localtionFaultFun(void)
{
	uint8_t tempMile = 0;
	if(osKernelGetTickCount()>180000)					//每次开机后三分钟开始检测
	{
		if(gRealData.locationState == 1)				//无效定位，记录无定位时里程 并保存
		{
			if(gUserDara.isSiteExcute == 0)
			{
				gUserDara.isSiteExcute = 1;
				gUserDara.outSiteMile = gRealData.totalMileage;
				saveUserData();											
			}
			
			tempMile = gRealData.totalMileage - gUserDara.outSiteMile;
			if(tempMile >= 50)
			{
				//定位异常;
				isLocFault = 1;
			}
		}
		else if(gRealData.locationState == 0)		//有效定位
		{
				if(gUserDara.isSiteExcute != 0)
				{
					gUserDara.isSiteExcute = 0;
					isLocFault = 0;
					saveUserData();			
				}
		}
	}
}

/*
功能：自动补电，欠压报文电压初始化
描述：
*/
static void AutoVoltInit()
{
	uint32_t temp = 0;
	/* 设置默认值 */
	if(gTerminalState.pwrVolt>22)				//24V 车
	{
		temp = (uint32_t)gUserDara.autoChgVolt;
		if(!(temp > 0 && temp < 36))									//需补电电压
		{
			gUserDara.autoChgVolt = 24;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.autoChgOverVolt;
		if(!(temp > 0 && temp < 36))									//停止补电电压
		{
			gUserDara.autoChgOverVolt = 25;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.underVolt;
		if(!(temp > 0 && temp < 36))									//欠压电压
		{
			gUserDara.underVolt = 23;
			isSetFirstV = 2;
		}
	}
	else if(gTerminalState.pwrVolt<18)	//12V 车
	{
		temp = (uint32_t)gUserDara.autoChgVolt;
		if(!(temp > 0 && temp < 36))									//需补电电压
		{
			gUserDara.autoChgVolt = 12;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.autoChgOverVolt;
		if(!(temp > 0 && temp < 36))									//停止补电电压
		{
			gUserDara.autoChgOverVolt = 13.8f;
			isSetFirstV = 2;
		}
		temp = (uint32_t)gUserDara.underVolt;
		if(!(temp > 0 && temp < 36))									//欠压电压
		{
			gUserDara.underVolt = 11;
			isSetFirstV = 2;
		}
	}
	if(isSetFirstV == 2)
	{
		saveUserData();
		isSetFirstV = 3;
	}

	bsp_sleep_wakeup_volt(gUserDara.underVolt);
}

/*
功能：解析锁车/限速指令
描述：
*/
static uint8_t UnpackLockCMD(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.lockCMD = rbuf[24];												//锁车控制指令
	switch(rbuf[24])
	{
		case 0x55:gUserDara.runCMD = 1;																						//禁止运营
			break;
		case 0x56:gUserDara.runCMD = 1;																						
			break;
		case 0x57:gUserDara.runCMD = 1;																						
			break;
		case 0x60:gUserDara.runCMD = 1;																						
			break;
		default:gUserDara.runCMD = 0;
			break;
	}
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_LOCKCMD,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
功能：程序版本切换控制
描述：0x55，锁车版本；0xAA，非锁车版本
*/
static uint8_t UnpackSoftVer_Change(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.changeCMD = rbuf[24];										//版本切换控制指令

	saveUserData();
	DataAreaLen = MakeCmd(CMD_SOFTVER_CHANGE,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
功能：绑定控制
描述：0x55，执行绑定；0x00，默认
*/
static uint8_t UnpackBindingCtl(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.bindingCMD = rbuf[24];									//绑定控制指令

	saveUserData();
	DataAreaLen = MakeCmd(CMD_BINDINGCTR,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
功能：车门控制
描述：开门:0x55, 关门:0xAA,
*/
static uint8_t UnpackDoorCtl(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.doorCMD = rbuf[24];											//车门控制指令
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_DOORCTR,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
功能：保养提示
描述：
*/
static uint8_t UnpackMaintTips(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.maintCMD = rbuf[24];										//车门控制指令
		
	saveUserData();
	DataAreaLen = MakeCmd(CMD_MAINTTIPS,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
功能：缴费提示
描述：
*/
static uint8_t UnpackPayTips(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.payCMD = rbuf[24];											//车门控制指令
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_PAYTIPS,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

/*
功能：年审提示
描述：
*/
static uint8_t UnpackYearCheckTips(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	char vins[18] = {0};
	memcpy(&vins[0],&rbuf[4],17);
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.yearCheckCMD = rbuf[24];								//车门控制指令																	
	
	saveUserData();
	DataAreaLen = MakeCmd(CMD_CHECKTIPS,replyFlag,vins,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}



