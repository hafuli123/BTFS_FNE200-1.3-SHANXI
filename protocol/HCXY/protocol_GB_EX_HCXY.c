/*
文 件：protocol_GB_EX_HCXY.c
功 能：定时唤醒控制及企标逻辑
日 期: 2022/2/17
公 司：北理新源(佛山)信息科技有限公司
作 者: LGC
*/

#include "HCXY/protocol_GB_EX_HCXY.h"

enum PARA_ID{
	PARA_WAKE_TIME = 0x9C,						//设置唤醒时间
	PARA_WAKE_INTERVAL = 0x9D,				//设置唤醒间隔
	PARA_WAKE_SENDTIME = 0x9E,				//设置唤醒后数据上送时间长度
};

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;										//是否使用
	char *vin;											//车架号
	uint8_t bLogouted;							//登出标志位 0:未登出 1:已登出
	uint8_t sendOverTimeCnt;				//发送次数
	//内部时间戳
	uint32_t chgPowerStamp;					//充电电量上报时间戳
	//其他参数
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffLen;								//发送数据缓冲区长度
}GBSTA;

#define MAX_EXT_LINK 1

/* 用户自定义参数 */
static GBSTA gbSta[MAX_EXT_LINK] = {0};
USER_DATA gUserDara;												//用户数据存储

/* 联网控制参数 */
static uint32_t reConnectStamp = 0;					//重播计时
static uint8_t printPackCmd = 1;						//打印报文到串口控制位

/* 休眠唤醒参数 */
static int wakelock = -1;										//唤醒锁
static uint64_t wakeTime = 0;								//唤醒时间
static uint32_t wk_DownTime = 0;						//唤醒倒计时
uint8_t wk_Excute = 0;											//唤醒标志控制
uint8_t allow_UserOnLine = 0;								//允许用户平台联网
uint8_t allow_FtpUpdata = 0;								//允许FTP连接

/******************************************************
功能：存储用户数据
描述：存储锁车指令相关数据
*******************************************************/
void saveUserData(void)
{
		gUserDara.store_flag = 0xAA;
		User_ID_Save(&gUserDara,sizeof(USER_DATA));				
}

/******************************************************
功能：发送数据到网络
描述：
*******************************************************/
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

/***********************唤醒功能****************************/

/******************************************************
功能：唤醒回调函数
描述：	
	返回1 当下不允许睡眠，需休眠倒计时后再进入休眠
*******************************************************/
static uint8_t wake_RT_Callback(uint8_t type)		
{
	wk_Excute = 1;
	bsp_wakelock_lock(wakelock);
	//allow_UserOnLine = 1;
	allow_FtpUpdata = 0;
	wk_DownTime = osKernelGetTickCount();
	return 1;
}
/******************************************************
功能：唤醒后再次休眠倒计时
描述：	
		
*******************************************************/
static void wake_RT_OnLineTime(void)
{
	if(wk_Excute)		
	{	
		if(osKernelGetTickCount() - wk_DownTime > (gUserDara.wk_SendInterval-3)*10000)
		{
			allow_FtpUpdata = 1;	
		}
		
		//数据上报超时 || 有唤醒源
		if(osKernelGetTickCount() - wk_DownTime > gUserDara.wk_SendInterval*10000)
		{
			//解除唤醒锁
			if(bsp_wakelock_unlock(wakelock) == BSP_SLEEP_SUCCESS)
			{
				wk_Excute = 0;
				allow_UserOnLine = 0;											
			}
			printfData("wk_lineTime over\r\n");
		}
	}
}

/******************************************************
功能：定时唤醒时间参数设置
描述：
		 设置定时唤醒起始时间，与定时周期
			swID 1：启用定时唤醒；0：禁用定时唤醒
*******************************************************/
static void wake_RT_SetTime(uint8_t swID)							
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
			
			bsp_wakelock_para(wakelock,wakeTime,600,wake_RT_Callback,0,0);												/* 升级10分钟 */
		}
		else 								//关闭定时唤醒
		{
			bsp_wakelock_para(wakelock,0,0,0,0,0);				//关闭定时唤醒 进入硬件休眠
		}
	}
}

/******************************************************
功能：定时唤醒主体控制
描述：
			控制定时唤醒主体
*******************************************************/
static void wake_RT_MainFun(void)
{	
		if(getFTPModeState() == 1)					//无升级指令，关闭定时唤醒 进入硬件休眠		
		{
			wake_RT_SetTime(1);															
		}
		else
		{
			wake_RT_SetTime(0);								//有升级指令，恢复定时唤醒 进入软件休眠
		}

}

/***********************功能控制部分****************************/

/******************************************************
功能：控制 远程升级指令执行
描述：终端在线升级，受前置条件影响，控制终端进入休眠 10分钟后再唤醒升级
		1可升级 0不可升级
*******************************************************/
uint8_t contrlFTPModeExcute(void)
{
 return allow_FtpUpdata;
}

/******************************************************
功能：企标平台控制联网上报
描述：定时唤醒后，需要联网操作。
	0：不允许企标平台网络连接 1：允许企标平台网络连接
*******************************************************/
uint8_t extCtrlNetSta(void* obj)
{
	return 0;
}

/******************************************************
功能：控制唤醒数据报文存储
描述：定时唤醒后，需要联网操作。
	0:不存储定时唤醒后上报平台的实时报文数据 1：存储定时唤醒后上报平台的实时报文数据
*******************************************************/
uint8_t weakCtrlNetSta(void)
{
	return 0;
}



/********************终端管理平台扩展部分************************/
/******************************************************
功能：终端管理平台设置终端参数信息
*******************************************************/
uint8_t extSetPara(uint8_t check,uint8_t setParaId,const uint8_t* setBuff)
{
	return 0;
}

/******************************************************
功能：终端管理平台读取终端参数信息
*******************************************************/
uint8_t extReadPara(uint8_t readParaId,uint8_t* readBuff)
{
	return 0;
}

/*********************国标32960拓展协议*************************/
/******************************************************
功能：扩展协议初始化
描述：返回配置状态参数
*******************************************************/
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	
	/* 读取用户参数 */
	if(gUserDara.store_flag != 0xAA)
	{
		User_ID_Read(&gUserDara,sizeof(USER_DATA));		
		gUserDara.store_flag = 0xAA;		
	}
	
	for(i = 0;i < MAX_EXT_LINK;i++)
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
	
	wakelock = bsp_wakelock_create();				/* 创建唤醒锁 */	
	
	return &gbSta[objLinkIdx];
}

/******************************************************
功能：扩展协议运行   
描述：	
			ctrl:0 网络离线 1:网络在线 2:睡眠 
			sta:0 国标未登入 1:国标已登入 
返回值:
			0 扩展协议未登入 1:扩展协议已登入
*******************************************************/
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	if(obj == NULL)
		return 1;
	uint16_t dataLen = 0;
	wake_RT_MainFun();																//定时唤醒主体Fun

	if(pSta->bLink == 0)
	{
		wake_RT_OnLineTime();														//定时唤醒 倒计时

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
		}
		else
		{
			
		}
	}
	return 1;
}
/******************************************************
功能：扩展协议接收解包
描述：
*******************************************************/
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	return 0;
}

/******************************************************
功能：扩展实时数据		
描述：实时数据增加自定义数据
*******************************************************/
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	return 0;
}
