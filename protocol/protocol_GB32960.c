#include "bsp_sys.h"
#include "protocol_GB32960.h"
#include "bsp_storage.h"
#include "bsp_rtc.h"
#include "string.h"
#include "stdio.h"
#include "algo_verify.h"
#include "Fun_Net.h"
#include "bsp_uart_fifo.h"
#include "fun_can.h"
#include "fun_mon.h"



#define MAX_GB_LINK 3
#define SHDB_STA 0
enum COMMAND_CODE
{
	CMD_NONE = 0,
	CMD_LOGIN,                 			//车辆登入
	CMD_REALDATA,              			//实时数据
	CMD_REISSUEDATA,           			//补发数据
	CMD_LOGOUT,                			//车辆登出
	CMD_HEARTBEAT = 0x07,      			//心跳
	CMD_TIMING,               			//校时
	CMD_QUERYPARAS = 0x80,     			//查询命令
	CMD_SETPARAS,              			//设置命令
	CMD_CTRL,                  			//控制命令
};

//掉电保存链路参数
typedef struct _CFG
{
	uint32_t regTimeStamp;					//注册时间戳标志
	uint16_t regNo;									//注册流水号
}CFG;

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;										//是否使用
	uint8_t ctrl;										//链路登入控制

	char *vin;											//车架号
	char* domain;										//链路域名
	uint32_t* port;									//链路端口号	
	uint8_t realHz;									//实时数据周期
	uint8_t heartbeatHz;						//心跳周期
	//平台连接状态参数
	uint8_t bLogined;       				//登入标志位 0:未登入 1:已登入 0xFF:发送登入中
	uint8_t bLogouted;							//登出标志位 0:未登出 1:已登出
	uint8_t bTiming;								//校时标志
	uint8_t sendOverTimeCnt;				//发送超时计数
	uint8_t reissueSta;							//补发状态，0：需要补发 1：补发完成
	//实时计时控制
	uint8_t realSec;								//实时秒针
	uint8_t realSecCnt;							//实时秒计数
	//3级报警控制
	uint8_t lastAlarmLevel;					//上一次报警等级
	uint8_t afterAlarm1hzCnt;				//报警后1秒频率条数
	uint8_t afterAlarmSecCnt;			  //报警后重新计时，用于频繁触发3级报警，实时30秒部分不用补发
	//内部时间戳
	uint32_t heartbeatStamp;				//心跳时间戳
	uint32_t reissueStamp;					//补发时间戳
	//其他参数
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffSize;							//发送数据缓冲区大小
	//补发文件名
	char reissueFileName[30];				//补发文件名
	uint8_t day;										//日
	void *extCfg;										//扩展配置
	uint8_t extSta;									//扩展协议状态
	//内部掉电保存参数
	CFG cfg;
}GBSTA;

static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo);							//车辆登入
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen);														//终端校时
static uint16_t packHeartBeat(char* vin,uint8_t* buff,uint16_t maxLen);													//终端心跳
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo);							//车辆登出
static uint16_t buildRealData(GBSTA *pSta,uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,RealData *pRealData);//打包实时数据
static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen);															//解包数据

static GBSTA gbSta[MAX_GB_LINK] = {0};																													//链路参数静态
static uint8_t printPackCmd = 0;																																//打印报文命令
static uint8_t logoutTestCmd = 0;																																//登出测试命令
static uint32_t ctrlOfflineStaTimeStamp = 0;																										//离线测试时间记录
static uint8_t ctrlOfflineSta = 0;																															//离线测试命令
static uint8_t ctrlAlarmSta = 0;																																//报警测试命令
static uint32_t ctrlAlarmStaTimeStamp = 0;																											//报警测试时间记录

/*初始化配置*/
void* gb32960Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t realHz,uint8_t heartbeatHz)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_GB_LINK;i++)
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
	realHz = (realHz == 0 || realHz > 60) ? 10 : realHz;
	heartbeatHz = (heartbeatHz < 10 || heartbeatHz > 120) ? 30 : heartbeatHz;
	//外部参数
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].domain = domain;
	gbSta[objLinkIdx].port = port;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].heartbeatHz = heartbeatHz;
	gbSta[objLinkIdx].realHz = realHz;	
	//内部
	gbSta[objLinkIdx].bLogined = 0;
	gbSta[objLinkIdx].bLogouted = 0;
	gbSta[objLinkIdx].bTiming = 0;
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].reissueSta = 0;
	
	gbSta[objLinkIdx].realSec = 0xFF;
	gbSta[objLinkIdx].realSecCnt = 0;
	gbSta[objLinkIdx].lastAlarmLevel = 0;
	gbSta[objLinkIdx].afterAlarm1hzCnt = 0;
	gbSta[objLinkIdx].afterAlarmSecCnt = 60;
	gbSta[objLinkIdx].heartbeatStamp = 0;
	gbSta[objLinkIdx].reissueStamp = 0;
	
	gbSta[objLinkIdx].buff = pBuff;
	gbSta[objLinkIdx].buffSize = pBuffSize;
	gbSta[objLinkIdx].day = g_system_dt.day;
	//掉电存储参数恢复
	readConfig(link,&gbSta[objLinkIdx].cfg,sizeof(gbSta[objLinkIdx].cfg),0);
	//同步日志文件
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),LOG);
	//同步补发文件
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),REC);
	if(objLinkIdx == 0)
	{
		gbSta[objLinkIdx].extCfg = extInit(gbSta[objLinkIdx].bLink,vin,0,gbSta[objLinkIdx].buff,gbSta[objLinkIdx].buffSize);
	}
	else
		gbSta[objLinkIdx].extCfg = 0;
	//返回当前链路参数状态指针
	return &gbSta[objLinkIdx];
}

void cleanPara(void* obj)
{
	GBSTA *pSta = obj;
	if((gSysPara.linkSwitch & (1 << pSta->bLink)) == 0)
	{
		Fun_Gprs_Tcp_disconnect(pSta->bLink);
		pSta->ctrl = 0;
	}
	pSta->reissueSta = 0;
	pSta->bLogined = 0;
	pSta->bLogouted = 0;
	pSta->bTiming = 0;	
}

static uint16_t realDataReadyLen(GBSTA *pSta)
{
	RealData* pRealData;
	uint8_t outDataIdx;
	uint16_t dataLen = 0;
	if(pSta->realSec != gRealData.second)
	{
		pSta->bTiming = (pSta->bTiming >= 1 && pSta->bTiming < 0xFF) ? pSta->bTiming + 1: pSta->bTiming;
		pSta->realSec = gRealData.second;
		pSta->afterAlarmSecCnt = pSta->afterAlarmSecCnt < 60 ? pSta->afterAlarmSecCnt + 1 : 60;
		//控制三级报警
		if(ctrlAlarmSta == 1)
		{
			gRealData.alarmLevel = 3;
			gRealData.batHighTempAlert = 1;
		} 
		//报警补发控制
		if(gRealData.alarmLevel == 3 && pSta->lastAlarmLevel != gRealData.alarmLevel)
		{
			//控制报警补发条数
			uint8_t reissueCnt = pSta->afterAlarmSecCnt - 30;
			//补发状态清零
			pSta->reissueSta = 0;
			//存储补发数据
			for(;reissueCnt > 0;reissueCnt--)
			{
				pRealData = getRealCache(reissueCnt,&outDataIdx);
				if(outDataIdx != 0xFF)
				{
					reissueCnt = outDataIdx;
					dataLen = buildRealData(pSta,CMD_REISSUEDATA,pSta->vin,pSta->buff,pSta->buffSize,pRealData);
					if(dataLen > 0)
					{
						//存储报警前30秒数据，用于补发
						saveHistory(pSta->bLink,0,pSta->buff,dataLen,0,REC);
					}
				}
				else
				{
					break;
				}
			}
			dataLen = 0;
			pSta->afterAlarm1hzCnt = 30;
			pSta->afterAlarmSecCnt = 0;
		}
		if(ctrlAlarmSta == 1)
		{
			gRealData.alarmLevel = 3;
			gRealData.batHighTempAlert = 1;
			if(osKernelGetTickCount() - ctrlAlarmStaTimeStamp > 180000)
			{
				ctrlAlarmSta = 0;
			}
		}
		pSta->lastAlarmLevel = gRealData.alarmLevel;
		//周期控制 | 报警30秒
		if(++pSta->realSecCnt >= pSta->realHz || pSta->afterAlarm1hzCnt > 0)
		{
			pSta->realSecCnt = 0;
			pSta->afterAlarm1hzCnt = (pSta->afterAlarm1hzCnt > 0 ? pSta->afterAlarm1hzCnt - 1 : 0 );
			pRealData = getRealCache(0,&outDataIdx);
			if(outDataIdx != 0xFF)
			{
				dataLen = buildRealData(pSta,pSta->bLogined != 1 ? CMD_REISSUEDATA : CMD_REALDATA,pSta->vin,pSta->buff,pSta->buffSize,pRealData);
				if(dataLen > 0 && pSta->bLogined != 1)
				{
					//存储实时数据,用于补发
					saveHistory(pSta->bLink,pSta->bLogined,pSta->buff,dataLen,0,REC);
				}
			}
			else
			{
				dataLen = 0;
			}
		}
	}
	return dataLen;
}

static uint8_t sendNetData(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	uint16_t sendLen = (buf[22] << 8 | buf[23] << 0) + 25;

//		comSendBuf(RS1_COM,buf,len);
	
	Fun_Gprs_Tcp_send(socket_fd,buf,sendLen);
	if(sendLen < len)
	{
		Fun_Gprs_Tcp_send(socket_fd,&buf[sendLen],len - sendLen);
	}
	if(buf[2] >= CMD_LOGIN && buf[2] <= CMD_LOGOUT)
	{
		saveHistory(socket_fd,1,buf,len,0,LOG);
	}
	return 0;
}
//uint32_t gRealDataSendCnt = 0;	//记录实时数据帧数量
/*参数：链路状态参数 控制 0:可以登录平台 1:登出平台*/
uint8_t gb32960Run(void* obj)
{
	static uint32_t startTicks = 0,endTicks = 0;
	uint8_t active = 0;
	uint32_t ticks = osKernelGetTickCount();
	uint16_t dataLen = 0;
	uint8_t svrSta = 0;
	GBSTA *pSta = obj;
	if(pSta->day != g_system_dt.day)
	{
		pSta->day = g_system_dt.day;
		//同步日志文件
		syncHistory(pSta->bLink,pSta->reissueFileName,sizeof(pSta->reissueFileName),LOG);
		//同步补发文件
		syncHistory(pSta->bLink,pSta->reissueFileName,sizeof(pSta->reissueFileName),REC);		
	}
	if(Fun_Gprs_Tcp_Sta(pSta->bLink) == FUN_GPRS_CONNECTED && ctrlOfflineSta == 0 && Fun_Gprs_Csq()>14)
	{
		svrSta = 1;
	}
	if(extCtrlNetSta(pSta->extCfg) == 1 ||(fun_can_Get_State(BSP_CAN) > 0 && (gRealData.carState == 1 || gRealData.chargeState == 1)))
	{
		//网络断开，设置联网参数
		if(svrSta == 0)
		{
			Fun_Gprs_Tcp_connect(pSta->bLink,pSta->domain,*pSta->port);
			Fun_Gprs_Tcp_Set_RecvCallBack(pSta->bLink,unpack);
		}
		startTicks = ticks;
		if(ticks - endTicks >= 10000)
		{
			pSta->ctrl = 1;
		}
	}
	else
	{
		endTicks = ticks;
		if(gRealData.chargeState == 4)
		{
			if(ticks - startTicks >= 360000)
			{
				pSta->ctrl = 0;
			}
		}
		else
		{
			if(ticks - startTicks >= 60000)
			{
				pSta->ctrl = 0;
			}			
		}
		if(pSta->ctrl == 0 && svrSta == 1 && pSta->bLogined != 1)
		{
			Fun_Gprs_Tcp_disconnect(pSta->bLink);
		}
	}
	//控制登出
	if(logoutTestCmd == 1)
	{
		pSta->ctrl = 0;
	}
	//联网状态查询或控制
	if(ticks - ctrlOfflineStaTimeStamp > 800000)
	{
		ctrlOfflineSta = 0;
	}
	//无法联网或链路未使用且完全停止，相关变量清零
	if(svrSta == 0 || pSta->bLogined == 0  || pSta->bLogouted == 1)
	{
		cleanPara(pSta);
	}
	//任务开启或终端已经登入且未登出
	if(pSta->ctrl == 1 || (pSta->bLogined == 1 && pSta->bLogouted == 0))
	{
		active = 1;
		dataLen = realDataReadyLen(pSta);
		//网络正常，平台交互
		if(svrSta == 1)
		{
			//连续3次超3秒响应超时,重播
			if(pSta->sendOverTimeCnt >= 3 &&  ticks - pSta->heartbeatStamp >= 3000)
			{
				pSta->sendOverTimeCnt = 0;
				Fun_Gprs_Tcp_disconnect(pSta->bLink);
				cleanPara(pSta);		
			}
			//登入后，正常交互报文
			else if(pSta->bLogined == 1)
			{
				if(dataLen > 0)//发送实时报文
				{
					sendNetData(pSta->bLink,pSta->buff,dataLen);
//					gRealDataSendCnt++;
				}
				else if(pSta->bTiming == 0)//校时
				{
					pSta->bTiming = 1;
					if((dataLen = packTiming(pSta->vin,pSta->buff,pSta->buffSize)) > 0)
						sendNetData(pSta->bLink,pSta->buff,dataLen);
				}
				else if(pSta->reissueFileName[0] != 0 && ticks - pSta->reissueStamp >= 500 && pSta->afterAlarm1hzCnt == 0)//补发报文
				{
					pSta->reissueStamp = ticks;
					dataLen = readHistory(pSta->bLink,pSta->reissueFileName,pSta->buff,pSta->buffSize,REC);
					if(dataLen > 0)
					{
						sendNetData(pSta->bLink,pSta->buff,dataLen);
					}
					else
						pSta->reissueFileName[0] = 0;//当前文件补发完成
				}
				else if(pSta->reissueSta == 0 && pSta->reissueFileName[0] == 0 && ticks - pSta->reissueStamp >= 1000)//同步补发文件
				{
					if(syncHistory(pSta->bLink,pSta->reissueFileName,sizeof(pSta->reissueFileName),REC) == 0)
					{
						pSta->reissueSta = 1;
					}
				}
				else if(ticks - pSta->heartbeatStamp >= pSta->heartbeatHz * 1000)//心跳
				{
					pSta->heartbeatStamp = ticks;
					pSta->sendOverTimeCnt++;
					if((dataLen = packHeartBeat(pSta->vin,pSta->buff,pSta->buffSize)) > 0)
						sendNetData(pSta->bLink,pSta->buff,dataLen);
					
				}
				else if(pSta->ctrl == 0 && pSta->bLogouted == 0)//登出
				{
					pSta->bLogouted = 1;
					if((dataLen = packLogout(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo)) > 0)
					{
						sendNetData(pSta->bLink,pSta->buff,dataLen);
					}
				}
			}
			else if(pSta->extSta == 1 && ticks - pSta->heartbeatStamp >= 20000 && pSta->bLogined != 1)//国标平台登入
			{
				uint32_t regTimeStamp = g_system_dt.year << 16 | g_system_dt.month << 8 | g_system_dt.day << 0;
				pSta->heartbeatStamp = ticks;
				pSta->bLogined = 0xFF;
				//流水号
				if(pSta->cfg.regTimeStamp != regTimeStamp)
				{
					pSta->cfg.regTimeStamp = regTimeStamp;
					pSta->cfg.regNo = 0;
				}
				pSta->cfg.regNo++;
				saveConfig(pSta->bLink,&pSta->cfg,sizeof(pSta->cfg),0);
				//打包
				if((dataLen = packLogin(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo)) > 0)
				{
					sendNetData(pSta->bLink,pSta->buff,dataLen);
				}
				pSta->sendOverTimeCnt++;
			}
		}
	}
	//运行扩展协议
	pSta->extSta = extRun(pSta->extCfg,(pSta->ctrl == 1 || (pSta->bLogined == 1 && pSta->bLogouted == 0)) ? svrSta : 2,(pSta->bLogined == 1 &&  pSta->bLogouted == 0) ? 1 : 0);
	return active;
}

/*获取链路状态*/
uint8_t gb32960GetSta(uint8_t id)
{
	if(id < MAX_GB_LINK)
		return (gbSta[id].bLogined == 1);
	else if(id == 0xFF)
	{
		for(id = 0;id < MAX_GB_LINK;id++)
		{
			if(gbSta[id].bLogined == 1)
			{
				return 1;
			}
		}
	}
	return 0;
}

uint8_t gb32960Ctrl(uint8_t ctrl)
{
	switch(ctrl)
	{
		case GB32960_TEST_LOGIN:logoutTestCmd = 0;break;
		case GB32960_TEST_LOGOUT:logoutTestCmd = 1;break;
		case GB32960_TEST_ONLINE:ctrlOfflineSta = 0;break;
		case GB32960_TEST_OFFLINE:ctrlOfflineSta = 1;ctrlOfflineStaTimeStamp = osKernelGetTickCount();break;
		case GB32960_TEST_UNALARM:ctrlAlarmSta = 0;break;
		case GB32960_TEST_ALARM:ctrlAlarmSta = 1;ctrlAlarmStaTimeStamp = osKernelGetTickCount();break;
		case GB32960_TEST_UNPRINT:printPackCmd = 0;break;
		case GB32960_TEST_PRINT:printPackCmd = 1;break;
	}
	return 0;
}

//添加数据报头及校验
static uint16_t MakeCmd(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *wbuf,uint16_t dataLen)
{
	wbuf[0] = 0x23;
	wbuf[1] = 0x23;
	wbuf[2] = cmd;
	wbuf[3] = rsp;
	memcpy(&wbuf[4],vin,17);
	extGetVin(cmd,rsp,(char*)&wbuf[4],NULL);
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

//打包登入数据
static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo)
{
	uint16_t DataAreLen = 0;
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	DataAreLen = 6;
	buff[30] = (regNo) >> 8;         	//注册流水号H
	buff[31] = (regNo);
	DataAreLen += 2;
	Fun_Gprs_getICCID((char*)&buff[32],20);
	DataAreLen += 20;
	buff[52] = gRealData.subBatSysCnt;        			//可充电储能子系统个数
	DataAreLen++;
	buff[53] = 0;
	DataAreLen++;
	if(gRealData.subBatSysCnt > 0 && gRealData.subBatSysCnt <= 2 && gRealData.rechargeSysCodeLen > 0	&&  gRealData.rechargeSysCodeLen <= 50)
	{
		uint8_t i = 0;
		buff[53] = gRealData.rechargeSysCodeLen; //可充电储能系统编码长度
		for(i = 0;i < gRealData.subBatSysCnt;i++)
		{
			memcpy(&buff[54 + i * gRealData.rechargeSysCodeLen],gRealData.subSysData[i].rechargeSysCode,gRealData.rechargeSysCodeLen);
			DataAreLen += gRealData.rechargeSysCodeLen;
		}
	}
	return MakeCmd(CMD_LOGIN,0xFE,vin,buff,DataAreLen);
}

//打包校时数据
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen)
{
	return MakeCmd(CMD_TIMING,0xFE,vin,buff,0);	
}

//打包心跳数据
static uint16_t packHeartBeat(char* vin,uint8_t* buff,uint16_t maxLen)
{
	return MakeCmd(CMD_HEARTBEAT,0xFE,vin,buff,0);
}

//打包登出数据
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo)
{
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	buff[30] = (regNo) >> 8;
	buff[31] = (regNo);
	return MakeCmd(CMD_LOGOUT,0xFE,vin,buff,8);
}

/*打包整车数据 21字节*/
static uint8_t PackVehicleData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t usVal = 0;
	uint32_t dwVal = 0;
	uint8_t Idx = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x01;
		bPackBuf[Idx++] = pRealData->carState;                     //车辆状态
		bPackBuf[Idx++] = pRealData->chargeState;                  //充电状态
		bPackBuf[Idx++] = pRealData->operationState;               //运行模式
		
		usVal = (uint16_t)(pRealData->speed);                      //车速
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->speed * 10);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
		dwVal = (uint32_t)(pRealData->totalMileage);               //里程
		if(dwVal != 0xFFFFFFFF  && dwVal != 0xFFFFFFFE)
			dwVal = (uint32_t)(pRealData->totalMileage * 10);
		else
			dwVal = 0xFFFFFFFE;
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 24);
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 16);
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(dwVal >> 0);
		
		usVal = (uint16_t)(pRealData->total_volt);                 //总电压
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->total_volt * 10);          //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
		usVal = (uint16_t)(pRealData->total_current);               //总电流
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->total_current * 10 + 10000);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);	
		
		bPackBuf[Idx++] = pRealData->soc;                           //SOC		
		bPackBuf[Idx++] = pRealData->dc2dcState;                    //DC-DC状态	
		bPackBuf[Idx++] = pRealData->stall;                         //档位		   
		bPackBuf[Idx++] = (uint8_t)(pRealData->mohm >> 8);          //绝缘电阻
		bPackBuf[Idx++] = (uint8_t)(pRealData->mohm >> 0);		
		bPackBuf[Idx++] = pRealData->acceleratorVal;                //加速踏板行程值
		bPackBuf[Idx++] = pRealData->brakingVal;                    //制动踏板行程值
	}
	return Idx;
}

/*打包驱动电机数据*/
static uint16_t PackMotorData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0, usVal = 0;
	uint8_t i = 0;
	int16_t sVal = 0;
	
	if(bPackBuf != 0 && pRealData->motorCnt >= 1 && pRealData->motorCnt <= 2 && pRealData->chargeState != 1 && pRealData->chargeState != 4)
	{
		bPackBuf[Idx++] = 0x02;
		bPackBuf[Idx++] = pRealData->motorCnt;
		for(i = 0; i < pRealData->motorCnt; i++)
		{
			bPackBuf[Idx++] = pRealData->motorData[i].motorIdx;                    //驱动电机序号
			bPackBuf[Idx++] = pRealData->motorData[i].motorState;                  //驱动电机状态
			
			sVal = pRealData->motorData[i].motorCtrTemp;                           //驱动电机控制器温度
			if(sVal < 0xFE)
				sVal = sVal + 40;
			bPackBuf[Idx++] = (uint8_t)sVal;
			
			usVal = pRealData->motorData[i].motorSpeed;                            //驱动电机转速
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = usVal + 20000;
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
			
			usVal = (uint16_t)pRealData->motorData[i].motorTorsion;                //驱动电机转矩
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = (uint16_t)(pRealData->motorData[i].motorTorsion * 10 + 20000);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
			
			sVal = pRealData->motorData[i].motorTemp;                              //驱动电机温度
			if(sVal < 0xFE)
				sVal = sVal + 40;
			bPackBuf[Idx++] = (uint8_t)sVal;
			
			usVal = (uint16_t)(pRealData->motorData[i].motorVol);                  //电机控制器输入电压
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = (uint16_t)(pRealData->motorData[i].motorVol * 10);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
			usVal = (uint16_t)(pRealData->motorData[i].motorCur);                  //电机控制器直流母线电流
			if(usVal != 0xFFFF && usVal != 0xFFFE)
				usVal = (uint16_t)(pRealData->motorData[i].motorCur * 10 + 10000);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		}
	}
	return Idx;
}

/*打包发动机数据*/
static uint16_t PackEngineData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0, usVal = 0;
	if(bPackBuf != 0 && pRealData->engineState < 0xFE && pRealData->fuelConsumption < 0xFFFE && pRealData->chargeState != 1 && pRealData->chargeState != 4)
	{
		bPackBuf[Idx++] = 0x04;
		bPackBuf[Idx++] = pRealData->engineState;                   //发动机状态
		
		usVal = pRealData->crankshaftSpeed;                         //曲轴转速
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		
		usVal = (uint16_t)(pRealData->fuelConsumption);             //燃料消耗率
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->fuelConsumption * 100);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
		bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
	}
	return Idx;
}

/*打包燃料电池数据*/
static uint16_t PackFuelBatData(RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t Idx = 0, usVal = 0;
	uint8_t FuelBatTemCnt = 0, i = 0;
	int16_t sVal = 0;
	
	if(bPackBuf != 0 && pRealData->fuelBatVol < 0xFFFE && pRealData->maxHydrPressure < 0xFFFE && 18 + pRealData->fuelBatTemCnt < RemainLen )
	{
		bPackBuf[Idx++] = 0x03;
		usVal = (uint16_t)(pRealData->fuelBatVol);                  //燃料电池电压
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->fuelBatVol * 10);           //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		usVal = (uint16_t)(pRealData->fuelBatCur);                  //燃料电池电流
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->fuelBatCur * 10);          //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal / 256);
		bPackBuf[Idx++] = (uint8_t)(usVal % 256);
		
		usVal = (uint16_t)(pRealData->batFuelConsumption);         //电池燃料消耗率
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->batFuelConsumption * 100);//usVal * 100;
		
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		FuelBatTemCnt = ((pRealData->fuelBatTemCnt <= countof(pRealData->fuelBatTem)) ? pRealData->fuelBatTemCnt : 0); //燃料电池温度探针总数
		bPackBuf[Idx++] = (uint8_t)(pRealData->fuelBatTemCnt/256);
		bPackBuf[Idx++] = (uint8_t)(pRealData->fuelBatTemCnt%256);
		
		for(i = 0; i < FuelBatTemCnt; i++)
		{
			sVal = pRealData->fuelBatTem[i];                              //燃料电池探针温度值
			if(sVal < 0xFE)
				sVal = sVal + 40;
			bPackBuf[Idx++] = (uint8_t)sVal;
		}
		
		usVal = pRealData->maxHydrSysTem;                               //氢系统中最高温度
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			sVal = (pRealData->maxHydrSysTem + 40) * 10;
		
		bPackBuf[Idx++] = (uint8_t)(sVal/256);
		bPackBuf[Idx++] = (uint8_t)(sVal%256);
		
		bPackBuf[Idx++] = pRealData->maxHydrSysTemIdx;                 //氢系统中最高温度探针代号
		
		usVal = pRealData->maxHydrThickness;                           //氢气最高浓度
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		bPackBuf[Idx++] = pRealData->maxHydrThicknessIdx;              //氢气最高浓度传感器代号
		
		usVal = (uint16_t)(pRealData->maxHydrPressure);                //氢气最高压力
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(pRealData->maxHydrPressure * 10);  //usVal * 10;
		
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		bPackBuf[Idx++] = pRealData->maxHydrPressureIdx;              //氢气最高压力传感器代号
		bPackBuf[Idx++] = pRealData->dc2dcState_highVol;              //高压DC/DC状态
	}
	return Idx;
}

/*打包定位数据*/
static uint16_t PacklocatorData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	uint32_t dwVal = 0;		
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x05;
		//定位状态
		bPackBuf[Idx++] = pRealData->locationState;						
		//经度
		dwVal = (uint32_t)(pRealData->longd * 1000000);    
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		bPackBuf[Idx++] = (uint8_t)dwVal;
		//纬度
		dwVal = (uint32_t)(pRealData->latd * 1000000);  
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0xFF000000)>>24);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
		bPackBuf[Idx++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
		bPackBuf[Idx++] = (uint8_t)dwVal;
	}
	return Idx;
}

/*打包极值数据*/
static uint16_t PackExtremeVal(RealData *pRealData,uint8_t *bPackBuf,uint8_t packIdx)
{
	uint16_t Idx = 0,usVal = 0,VolIdx = 0;
	int16_t sVal = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x06;
		bPackBuf[Idx++] = pRealData->maxVolPack_index;                 	//最高电压电池子系统号
		
		VolIdx = (pRealData->maxVol_index - 1) / 200;
		if(VolIdx == packIdx)
		{
			bPackBuf[Idx++] = (pRealData->maxVol_index - 1) % 200 + 1;			//最高电压电池单体代号
			usVal = (uint16_t)(pRealData->max_singleVol);                  	//电池单体电压最高值
			if(usVal < 0xFFFE)
					usVal = (uint16_t)(pRealData->max_singleVol * 1000);
			else
				usVal = 0xFFFE;
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		}
		else
		{
			if(packIdx < VolIdx)//极值在后一分包
			{
				bPackBuf[Idx++] = 200;
			}
			else//极值在前一分包
			{
				bPackBuf[Idx++] = 0;
			}
			bPackBuf[Idx++] = 0xFF;
			bPackBuf[Idx++] = 0xFF;			
		}
		
		
		bPackBuf[Idx++] = pRealData->minVolPack_index;                	//最低电压电池子系统号 
		VolIdx = (pRealData->minVol_index - 1) / 200;
		if(VolIdx == packIdx)
		{
			bPackBuf[Idx++] = (pRealData->minVol_index - 1) % 200 + 1;             
			usVal = (uint16_t)(pRealData->min_singleVol);                 //电池单体电压最低值
			if(usVal < 0xFFFE)
				usVal = (uint16_t)(pRealData->min_singleVol * 1000);
			else
				usVal = 0xFFFE;
			bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
			bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
		}
		else
		{
			if(packIdx < VolIdx)//极值在后一分包
				bPackBuf[Idx++] = 200;
			else//极值在前一分包
				bPackBuf[Idx++] = 0;
			bPackBuf[Idx++] = 0xFF;
			bPackBuf[Idx++] = 0xFF;			
		}
		
		bPackBuf[Idx++] = pRealData->maxTemperPack_index;            //最高温度子系统号
		bPackBuf[Idx++] = pRealData->maxTemper_index;                //最高温度探针序号
		sVal = pRealData->max_singleTemper;                          //最高温度值
		if(sVal < 0xFE)
			sVal = sVal + 40;
		bPackBuf[Idx++] = (uint8_t)sVal;
		
		bPackBuf[Idx++] = pRealData->minTemperPack_index;            //最低温度子系统号
		bPackBuf[Idx++] = pRealData->minTemper_index;                //最低温度探针序号
		sVal = pRealData->min_singleTemper;                          //最低温度值
		if(sVal < 0xFE)
			sVal = sVal + 40;
		bPackBuf[Idx++] = (uint8_t)sVal;
	}
	return Idx;
}

/*打包报警数据*/
static uint16_t PackAlertData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0, i = 0;
	uint8_t N1Cnt = 0, N2Cnt = 0, N3Cnt = 0, N4Cnt = 0, Tmp = 0;
	
	N1Cnt = ((pRealData->batFaultCnt <= sizeof(pRealData->batFault) / 4) ? pRealData->batFaultCnt : 0);
	N2Cnt = ((pRealData->motorFaultCnt <= sizeof(pRealData->motorFault) / 4) ? pRealData->motorFaultCnt : 0);
	N3Cnt = ((pRealData->engineFaultCnt <= sizeof(pRealData->engineFault) / 4) ? pRealData->engineFaultCnt : 0);
	N4Cnt = ((pRealData->otherFaultCnt <= sizeof(pRealData->otherFault) / 4) ? pRealData->otherFaultCnt : 0);
	
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x07;
		bPackBuf[Idx++] = pRealData->alarmLevel;                                   //报警等级
		bPackBuf[Idx++] = 0;                                                      //预留
		Tmp = pRealData->batOverCharge > 0 ? 0x04 : 0x00;                          //车载储能装置类型过充
		Tmp = Tmp | (pRealData->motorTempAlert > 0 ? 0x02 : 0x00);                 //驱动电机温度报警
		Tmp = Tmp | (pRealData->highPressInterlockStateAlert > 0 ? 0x01 : 0x00);   //高压互锁状态报警
		bPackBuf[Idx++] = Tmp;
		
		Tmp = 0;
		Tmp = pRealData->motorCtrTemAlert > 0 ? 0x80 : 0x00;                 //驱动电机控制器温度报警
		Tmp = Tmp | (pRealData->dc2dcStateAlert > 0 ? 0x40 : 0x00);          //DC-DC状态报警
		Tmp = Tmp | (pRealData->brakingAlert > 0 ? 0x20 : 0x00);             //制动系统报警
		Tmp = Tmp | (pRealData->dc2dcTemAlert > 0 ? 0x10 : 0x00);            //DC-DC温度报警
		Tmp = Tmp | (pRealData->insulationFailtAlert > 0 ? 0x08 : 0x00);     //绝缘故障
		Tmp = Tmp | (pRealData->singleBatPoorConsisAlert > 0 ? 0x04 : 0x00); //电池单体一致性差报警
		Tmp = Tmp | (pRealData->batNotMatchAlert > 0 ? 0x02 : 0x00);         //可充电储能系统不匹配报警
		Tmp = Tmp | (pRealData->socHopAlert > 0 ? 0x01 : 0x00);              //SOC跳变报警
		bPackBuf[Idx++] = Tmp;
		
		Tmp = 0;
		Tmp = pRealData->socHighAlert > 0 ? 0x80 : 0x00;                    //SOC过高报警
		Tmp = Tmp | (pRealData->singleBattLowVolAlert > 0 ? 0x40 : 0x00);   //单体电池欠压报警
		Tmp = Tmp | (pRealData->singleBatHighVolAlert > 0 ? 0x20 : 0x00);   //单体电池过压报警
		Tmp = Tmp | (pRealData->socLowAlert > 0 ? 0x10 : 0x00);             //SOC低报警
		Tmp = Tmp | (pRealData->batLowVolAlert > 0 ? 0x08 : 0x00);          //车载储能装置类型欠压报警
		Tmp = Tmp | (pRealData->batHighVolAlert > 0 ? 0x04 : 0x00);         //车载储能装置类型过压报警
		Tmp = Tmp | (pRealData->batHighTempAlert > 0 ? 0x02 : 0x00);        //电池高温报警
		Tmp = Tmp | (pRealData->tempDiffAlert > 0 ? 0x01 : 0x00);           //温度差异报警
		bPackBuf[Idx++] = Tmp;
		
		bPackBuf[Idx++] = N1Cnt;//可充电储能装置故障总数N1
		for(i = 0; i < N1Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->batFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->batFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->batFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->batFault[i];
		}                

		bPackBuf[Idx++] = N2Cnt;//驱动电机故障总数N2
		for(i = 0; i < N2Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->motorFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->motorFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->motorFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->motorFault[i];
		}
		
		bPackBuf[Idx++] = N3Cnt;//发动机故障总数N3
		for(i = 0; i < N3Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->engineFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->engineFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->engineFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->engineFault[i];
		}
		
		bPackBuf[Idx++] = N4Cnt;//其它故障总数N4
		for(i = 0; i < N4Cnt; i++)
		{
			bPackBuf[Idx++] = (uint8_t)((pRealData->otherFault[i] & 0xFF000000)>>24);
			bPackBuf[Idx++] = (uint8_t)((pRealData->otherFault[i] & 0x00FF0000)>>16);
			bPackBuf[Idx++] = (uint8_t)((pRealData->otherFault[i] & 0x0000FF00)>>8);
			bPackBuf[Idx++] = (uint8_t)pRealData->otherFault[i];
		}
	}
	return Idx;
}

/*打包单体电压*/
static uint16_t PackMonomerBatterys(RealData *pRealData,uint8_t *bPackBuf,uint16_t RemainLen,uint8_t packIdx,uint8_t *pLastFlag)
{
	uint16_t Idx = 0, i = 0, j = 0;
	uint16_t usVal = 0, BatIdx = 0;
	uint8_t packVolCnt = 0,flag = 0;
	if(bPackBuf != 0 && pRealData->subBatSysCnt >= 1 && pRealData->subBatSysCnt <= 2 && pRealData->subSysData[0].singleVolCnt > 0 && (pRealData->subSysData[0].singleVolCnt + pRealData->subSysData[1].singleVolCnt) <= sizeof(pRealData->single_vol)/sizeof(pRealData->single_vol[0]))
	{
		if(RemainLen >= (2 + pRealData->subBatSysCnt * 10))
		{
			bPackBuf[Idx++] = 0x08;
			bPackBuf[Idx++] = pRealData->subBatSysCnt;
			for(i = 0; i < pRealData->subBatSysCnt; i++)
			{
				BatIdx = packIdx * 200;
				if(pRealData->subSysData[i].singleVolCnt - BatIdx > 200)
				{
					//分别未完
					flag = 1;
					packVolCnt = 200;
				}
				else if(pRealData->subSysData[i].singleVolCnt - BatIdx > 0)
				{
					//分包完成
					packVolCnt = pRealData->subSysData[i].singleVolCnt - BatIdx;
					flag = 0;
				}
				else
				{
					//空包
					packVolCnt = 0;
				}
				if(RemainLen > Idx + 10 + (packVolCnt * 2))
				{
					bPackBuf[Idx++] = pRealData->subSysData[i].subSysIdx;        //可充电储能子系统号
					usVal = (uint16_t)(pRealData->subSysData[i].subSysVol);      //可充电储能装置电压
					if(usVal < 0xFFFE)
					{
						usVal = (uint16_t)(pRealData->subSysData[i].subSysVol * 10);
					}
					bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
					bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
					
					usVal = (uint16_t)(pRealData->subSysData[i].subSysCur);      //可充电储能装置电流
					if(usVal < 0xFFFE)
					{
						usVal = (uint16_t)(pRealData->subSysData[i].subSysCur * 10 + 10000);
					}
					bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
					bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
					
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleVolCnt >> 8);//子系统单体电池总数
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleVolCnt >> 0);
					
					bPackBuf[Idx++] = (BatIdx + 1) >> 8;     									//本帧起始电池序号
					bPackBuf[Idx++] = (BatIdx + 1) >> 0;
					bPackBuf[Idx++] = packVolCnt;   													//本帧单体电池总数
					for(j = 0; j < packVolCnt && j < 200; j++)
					{
						BatIdx = pRealData->subSysData[i].singleVolStartIdx + j + (packIdx * 200);
						usVal = (uint16_t)(pRealData->single_vol[BatIdx]);			//单体电池电压
						if(usVal < 0xFFFE && pRealData->single_vol[BatIdx] >= 0)
							usVal = (uint16_t)(pRealData->single_vol[BatIdx] * 1000);
						else
							usVal = 0xFFFE;
						bPackBuf[Idx++] = (uint8_t)(usVal >> 8);
						bPackBuf[Idx++] = (uint8_t)(usVal >> 0);
					}
				}
				else
				{
					Idx = 0;
					break;
				}
			}
		}
	}
	if(flag == 0)
	{
		*pLastFlag = 1;
	}
	return Idx;
}

/*打包温度数据*/
static uint16_t PackMonomerTemperatures(RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t Idx = 0, TempIdx = 0;
  uint16_t i = 0, j = 0;
	int16_t sVal = 0;
	
	if(bPackBuf != 0)
	{
		if(pRealData->subBatSysCnt >= 1 && pRealData->subBatSysCnt <= 2 && pRealData->subSysData[0].singleTemCnt > 0 && (pRealData->subSysData[0].singleTemCnt + pRealData->subSysData[1].singleTemCnt) <= sizeof(pRealData->single_temper)/sizeof(pRealData->single_temper[0]))
		{
			if(RemainLen >= (2 + pRealData->subBatSysCnt * 3 + pRealData->subSysData[0].singleTemCnt + pRealData->subSysData[1].singleTemCnt))
			{
				bPackBuf[0] = 0x09;
				bPackBuf[1] = pRealData->subBatSysCnt;//子系统个数
				Idx = 2;
				for(i = 0; i < pRealData->subBatSysCnt; i++)
				{
					bPackBuf[Idx++] = pRealData->subSysData[i].subSysIdx;//子系统序号
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleTemCnt >> 8);//温度探针个数
					bPackBuf[Idx++] = (uint8_t)(pRealData->subSysData[i].singleTemCnt >> 0);
						
					for(j = 0; j < pRealData->subSysData[i].singleTemCnt; j++)
					{
						TempIdx = pRealData->subSysData[i].singleTemStartIdx + j;
						sVal = pRealData->single_temper[TempIdx];//探针温度值
						if(sVal < 0xFE)
							sVal = sVal + 40;
						bPackBuf[Idx++] = (uint8_t)sVal;
					}
				}
			}
		}
	}
	return Idx;
}

/*
*  函数说明: 打包实时数据用于存储
*  参数wbuf: 输入缓冲区, byte0:数据单元单元长度高字节,byte1:数据单元长度低字节
*  返 回 值: 打包长度
*/
uint16_t buildRealData(GBSTA *pSta,uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,RealData *pRealData)
{
	uint16_t index,subLen,totalLen = 0;
	uint8_t i,lastFlag = 0,*pbuf;
	for(i = 0;i<3;i++)
	{
		pbuf = &buff[totalLen];
		index = 24;
		pbuf[index++] = (uint8_t)(pRealData->year - 2000);
		pbuf[index++] = (uint8_t)(pRealData->month);
		pbuf[index++] = (uint8_t)(pRealData->day);
		pbuf[index++] = (uint8_t)(pRealData->hour);
		pbuf[index++] = (uint8_t)(pRealData->minute);
		pbuf[index++] = (uint8_t)(pRealData->second);
		if(i == 0)
		{
			//整车数据
			index += PackVehicleData(pRealData,&pbuf[index]);
			//驱动电机数据
			index += PackMotorData(pRealData,&pbuf[index]);
			//发动机数据
			index += PackEngineData(pRealData,&pbuf[index]);
			//燃料电池数据
			index += PackFuelBatData(pRealData,&pbuf[index],maxLen - totalLen - index);
			//定位数据
			index += PacklocatorData(pRealData,&pbuf[index]);
		}
		//极值数据，多单体需要分包
		index += PackExtremeVal(pRealData,&pbuf[index],i);
		if(i == 0)
		{
			//报警数据
			index += PackAlertData(pRealData,&pbuf[index]);
		}
		//单体电压,多单体需要分包
		index += PackMonomerBatterys(pRealData,&pbuf[index],maxLen - totalLen - index,i,&lastFlag);
		if(i == 0)
		{
			//探针温度
			index += PackMonomerTemperatures(pRealData,&pbuf[index],maxLen - totalLen - index);					
		}
		if(lastFlag)
		{
			//附加扩展协议
			index += extReal(pSta->extCfg,pRealData,&pbuf[index],maxLen - totalLen - index);	
		}
        if(SHDB_STA)
        {
//            index += extHyFuelCellReal(pSta->extCfg,pRealData,&pbuf[index],maxLen - totalLen - index);
//            index += extSHDBReal(pSta->extCfg,pRealData,&pbuf[index],maxLen - totalLen - index);
        }
		subLen = MakeCmd(cmd,0xFE,vin,pbuf,index - 24);
		totalLen += subLen;
		//判断是否最后一包
		if(lastFlag)
		{
			break;
		}
	}
	return totalLen;
}

static uint8_t uppackQueryParas(GBSTA* pSta,const uint8_t *rbuf,uint16_t rLen)
{
	uint8_t RspParasBuf[150] = {0};
	uint8_t index = 0;
	uint8_t ParasPackIndex = 31;
	uint8_t QueryParaCnt,ResPondParasCnt = 0,i;
	QueryParaCnt = rbuf[30];
	memcpy(&RspParasBuf[24],&rbuf[24],6);
	for(i = 0; i < QueryParaCnt;i++)
	{
		uint8_t ParaId = rbuf[31+i];
		switch(ParaId)
		{
			case 0x01: //终端本地存储时间周期
			{
				RspParasBuf[ParasPackIndex + index++] = 0x01;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.localSaveInterval / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.localSaveInterval % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x02: //正常时，信息上报时间周期
			{
				RspParasBuf[ParasPackIndex + index++] = 0x02;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.realDataInterval / 256);
        RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.realDataInterval % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x03: //报警时，信息上报时间周期
			{
				RspParasBuf[ParasPackIndex + index++] = 0x03;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.warnRealDataInterval / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.warnRealDataInterval % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x04: //远程服务与管理平台域名长度
			{
				RspParasBuf[ParasPackIndex + index++] = 0x04;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)strlen(gSysPara.domain[pSta->bLink]);
				ResPondParasCnt++;
			}
			break;
			case 0x05: //远程服务与管理平台域名
			{
				uint8_t UrlLen = strlen(gSysPara.domain[pSta->bLink]);
				RspParasBuf[ParasPackIndex + index++] = 0x05;
				if(UrlLen > 0 && UrlLen <= MAX_URL_LEN)
				{
					memcpy(&RspParasBuf[ParasPackIndex + index],gSysPara.domain[pSta->bLink],UrlLen);
					index += UrlLen;					
				}
				ResPondParasCnt++;
			}
			break;
			case 0x06: //综合平台端口
			{
				RspParasBuf[ParasPackIndex + index++] = 0x06;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[pSta->bLink] / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[pSta->bLink] % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x07: //硬件版本
			{
				RspParasBuf[ParasPackIndex + index++] = 0x07;
				memcpy(&RspParasBuf[ParasPackIndex + index],HW_VERSION,5);
				index += 5;
				ResPondParasCnt++;
			}
			break;
			case 0x08: //固件版本
			{
				RspParasBuf[ParasPackIndex + index++] = 0x08;
				memcpy(&RspParasBuf[ParasPackIndex + index],SW_VERSION,5);
				index += 5;
				ResPondParasCnt++;
			}
			break;
			case 0x09: //终端心跳周期
			{
				RspParasBuf[ParasPackIndex + index++] = 0x09;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.heartInterval);
				ResPondParasCnt++;
			}
			break;
			case 0x0A: //终端应答超时时间
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0a;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.terminalRespTimeOut / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.terminalRespTimeOut % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x0B: //综合平台应答超时时间
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0b;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.platformRespTimeOut / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.platformRespTimeOut % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x0C: //下一轮登入时间间隔
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0c;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.nextLoginInterval);
				ResPondParasCnt++;
			}
			break;
			case 0x0D: //公共平台域名长度
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0d;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)strlen(gSysPara.domain[EV2_SERVER]);
				ResPondParasCnt++;
			}
			break;
			case 0x0E: //公共平台域名
			{
				int UrlLen = strlen(gSysPara.domain[EV2_SERVER]);
				RspParasBuf[ParasPackIndex + index++] = 0x0e;
				if(UrlLen > 0 && UrlLen <= MAX_URL_LEN)
				{
					memcpy(&RspParasBuf[ParasPackIndex + index],gSysPara.domain[EV2_SERVER],UrlLen);
					index += UrlLen;
				}
				ResPondParasCnt++;
			}
			break;
			case 0x0F: //公共平台端口
			{
				RspParasBuf[ParasPackIndex + index++] = 0x0f;
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[EV2_SERVER] / 256);
				RspParasBuf[ParasPackIndex + index++] = (uint8_t)(gSysPara.port[EV2_SERVER] % 256);
				ResPondParasCnt++;
			}
			break;
			case 0x10: //开启抽样检测
			{
				RspParasBuf[ParasPackIndex + index++] = 0x10;
				RspParasBuf[ParasPackIndex + index++] = (gSysPara.linkSwitch & (1 << EV2_SERVER)) ? 1 : 0;
				ResPondParasCnt++;
			}
			break;
		}
	}
	RspParasBuf[30] = ResPondParasCnt;
	index++;
	if(ResPondParasCnt >0)
	{
		uint8_t datalen = MakeCmd(CMD_QUERYPARAS,0x01,pSta->vin,RspParasBuf,index);
		Fun_Gprs_Tcp_send(pSta->bLink,RspParasBuf,datalen);
	}
	return 0;
}

static uint8_t uppackSetParas(GBSTA* pSta,const uint8_t *rbuf,uint16_t rLen)
{
	uint16_t ResponSetBufLen = 0;
	uint8_t RspParasBuf[150] = {0};
	uint16_t index;
	uint8_t i,j,paraCnt,packParaCnt;
	uint8_t *pData = (uint8_t*)&rbuf[24];
	uint8_t ServerUrlLen = 0, PublicUrlLen = 0;
	for(j = 0;j < 2;j++)//第一次校验数据，第二次设置
	{
		//解析跳过时间
		index = 6;
		paraCnt = pData[index++];
		packParaCnt = 0;
		for(i = 0;i< paraCnt;i++)
		{
			packParaCnt++;
			switch(pData[index++])
			{
				case 0x01:
				{
					if(j == 1)//本地存储时间周期
					{
						gSysPara.localSaveInterval = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x02:
				{
					if(j == 1) //正常时，信息上报时间周期
					{
						gSysPara.realDataInterval = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x03://报警时，信息上报时间周期
				{
					if(j == 1)
					{
						gSysPara.warnRealDataInterval = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x04://远程服务与管理平台域名长度
				{
					if(j == 1)
					{
						ServerUrlLen = pData[index];
					}
					index += 1;
				}
				break;
				case 0x05://远程服务与管理平台域名
				{
					if(j == 1 && ServerUrlLen > 0)
					{
						memset(gSysPara.domain[pSta->bLink],0,sizeof(gSysPara.domain[0]));
						strncpy(gSysPara.domain[pSta->bLink],(char*)&pData[index],ServerUrlLen);								
					}
					index += ServerUrlLen;
				}
				break;
				case 0x06://远端服务器端口号
				{
					if(j == 1)
					{
						gSysPara.port[pSta->bLink] = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x07://硬件版本
				{
					index += 5;
				}
				break;
				case 0x08://固件版本
				{
					index += 5;
				}
				break;
				case 0x09://心跳发送周期:
				{
					if(j == 1)
					{
						gSysPara.heartInterval = pData[index];
					}
					index += 1;	
				}
				break;
				case 0x0A://终端应答超时时间
				{
					if(j == 1)
					{
						gSysPara.terminalRespTimeOut = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x0B://平台应答超时时间
				{
					if(j == 1)
					{
						gSysPara.platformRespTimeOut = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x0C://新一轮登入周期
				{
					if(j == 1)
					{
						gSysPara.nextLoginInterval = pData[index];
					}
					index += 1;	
				}
				break;
				case 0x0D://公共平台域名长度
				{
					if(j == 1)
					{
						PublicUrlLen = pData[index];
					}
					index += 1;
				}
				break;
				case 0x0E://公共平台域名
				{
					if(j == 1 && ServerUrlLen > 0)
					{
						memset(gSysPara.domain[EV2_SERVER],0,sizeof(gSysPara.domain[0]));
						strncpy(gSysPara.domain[EV2_SERVER],(char*)&pData[index],PublicUrlLen);								
					}
					index += ServerUrlLen;
				}
				break;
				case 0x0F://公共平台端口号
				{
					if(j == 1)
					{
						gSysPara.port[EV2_SERVER] = (pData[index] << 8) | pData[index + 1];
					}
					index += 2;
				}
				break;
				case 0x10://抽样检测
				{
					if(j == 1)
					{
						gSysPara.linkSwitch &= ~ (1 << EV2_SERVER);
						gSysPara.linkSwitch |= (pData[index] > 0) << EV2_SERVER;
					}
					index += 1;
				}
				break;
				default:packParaCnt = 0;
					break;
			}
		}
		if(j == 0 && packParaCnt != paraCnt)//参数错误
		{
			break;
		}
	}
	if(packParaCnt == paraCnt)
	{
		//gUpdateParaFlag = 0x1F;
		System_Pare_Save();
	}
	memcpy(&RspParasBuf[24],&rbuf[24],6);
	ResponSetBufLen = 6;
	ResponSetBufLen = MakeCmd(CMD_SETPARAS,packParaCnt == paraCnt ? 0x01:0x02,pSta->vin,RspParasBuf,ResponSetBufLen);
	Fun_Gprs_Tcp_send(pSta->bLink,RspParasBuf,ResponSetBufLen);
	return 0;	
}

static uint8_t uppackCtrl(GBSTA* pSta,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[150];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	uint8_t isReconnet = 0,isReStart = 0,isFtpUpdate = 0,isPowerOff = 0;
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	if(rbuf[30] >= 0x01 && rbuf[30] <= 0x07)
	{
		switch(rbuf[30])
		{
			case 0x01: //远程升级
			{
//				char *pSplit = NULL;
				
//				if(strncmp((char*)&rbuf[31],"FTP://",6) == 0 || strncmp((char*)&rbuf[31],"ftp://",6)==0)
//				{
//					if(sscanf((char*)&rbuf[37],"%[^:]:%[^@]@%[^/;]/%[^;]",gFtpParas.ftpUser,gFtpParas.ftpPwd,gFtpParas.ftpUrl,gFtpParas.ftpDir) >= 3)
//					{
//						//拆分路径和文件名
//						if((pSplit = strrchr(gFtpParas.ftpDir,'/')) != NULL){
//							pSplit++;
//						}else{
//							pSplit = gFtpParas.ftpDir;
//						}
//						strcpy(gFtpParas.ftpName,pSplit);
//						*pSplit = '\0';
//						//拆分地址和端口
//						if((pSplit = strstr(gFtpParas.ftpUrl,":")) != NULL){
//							sscanf(pSplit,":%hu",&gFtpParas.ftpPort);
//							*pSplit = '\0';
//						}else{
//							gFtpParas.ftpPort = 21;
//						}
//						replyFlag = 1;
//						isFtpUpdate = 1;
//					}
//				}
				
				char *hostname;
				uint16_t hostport;
				char *username = NULL;
				char *password = NULL;
				char *filePach = NULL;
				char* fileName = NULL;
				char *pSplit = NULL;
				//用户名
				username = strstr((char*)&rbuf[31],"FTP://");
				if(username == NULL)
				{
					username = strstr((char*)&rbuf[31],"ftp://");
				}
				if(username != NULL)
				{
					username += 6;
					//密码
					password = strstr(username,":");
				}
				//URL
				if(password != NULL)
				{
					*password = 0;
					password++;
					hostname = strstr(password,"@");
				}
				//路径
				if(hostname != NULL)
				{
					*hostname = 0;
					hostname++;
					filePach = strstr(hostname,"/");
					//路径分隔兼容旧协议
					if(filePach == NULL)
					{
						filePach = strstr(hostname,";");
					}
					//取出端口号
					hostport = 21;
					pSplit = strstr(hostname,":");
					if(pSplit != NULL)
					{
						*pSplit = 0;
						pSplit++;
						sscanf(pSplit,"%hu",&hostport);
					}
				}
				if(filePach != NULL)
				{
					*filePach = 0;
					filePach++;
					//拆分路径和文件名
					fileName = "NEVT200-F4_Ftp.bin";
					if((pSplit = strstr(filePach,";")) != NULL)
					{
						*pSplit = 0;
					}
					if((pSplit = strrchr(filePach,'/')) != NULL)
					{
						*pSplit = 0;
						pSplit++;
						if(*pSplit != 0)
						{
							fileName = pSplit;
						}
					}
					else
					{
						fileName = filePach;
						filePach = NULL;
					}
					
				}
				replyFlag = 2;

				//开始升级			
				funMonSetFtpParas(hostname,hostport,username,password,filePach,fileName,NULL);
			}
			break;
			case 0x02: //车载终端关机
			{
				isPowerOff = 1;
			}
			break;
			case 0x03: //车载终端复位
			{
				isReStart = 1;
			}
			break;
			case 0x04: //恢复出厂设置
			{
				System_Pare_Restore();
				isReconnet = 1;
			}
			break;
			case 0x05: //断开数据通信链路
			{
				//重拔
				isReconnet = 1;
			}
			break;
			case 0x06: //车载终端报警
			{
//				gRealData.alarmLevel = rbuf[31];
//				if(gRealData.alarmLevel == 3)
//				{
//					gTerminalState.ctrlAlarmSta = 1;
//					gTerminalState.ctrlAlarmStaTimeStamp = osKernelGetTickCount();	
//				}
			}
			break;
			case 0x07: //抽样监测
			{
				gSysPara.linkSwitch |= 1 << EV2_SERVER;
				//开启监测链路
			}
			break;
		}
		DataAreaLen = MakeCmd(CMD_CTRL,replyFlag,pSta->vin,ResponCtrlBuf,DataAreaLen);
		sendNetData(pSta->bLink,ResponCtrlBuf,DataAreaLen);
		osDelay(50);
		if(isReconnet)
		{//重连
			Fun_Gprs_Tcp_disconnect(pSta->bLink);;
		}
//	if(isFtpUpdate)
//	{//远程升级
//		gIsPendingUpdate = 0x55;
//	}
		if(isReStart)
		{//重启
			BoardReset();
		}
		if(isPowerOff == 1)
		{//关机
			osDelay(500);
		}
	}
	return 0;
}

static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i,exSta;
	char vin[17];
	GBSTA* pSta = NULL;
//	if(printPackCmd == 1)
//	{
//		comSendBuf(RS1_COM,szRecvBuf,rLen);
//	}
	for(i = 0;i < MAX_GB_LINK;i++)
	{
		if(gbSta[i].bLink == link && gbSta[i].bUse == 1)
		{
			pSta = &gbSta[i];
			break;
		}
	}
	if(pSta == NULL)//未配置此链路
		return;
	else if(rLen < 25 )//数据长度错误
  {
		return;
	}
	else if(szRecvBuf[0] != 0x23 || szRecvBuf[1] != 0x23)//标志错误
	{
		return;
	}
	else if(getBccCode(szRecvBuf, 2, rLen - 3) != szRecvBuf[rLen - 1])//BCC校验
	{
		return;
	}
//	if(pSta->extCfg != NULL)
//	{
//		//扩展协议自行判断车架号
		exSta = extUnpack(link,szRecvBuf,rLen);
//		if(exSta == 1)
//			return;
//	}
//	memcpy(vin,pSta->vin,17);
//	extGetVin(szRecvBuf[2],szRecvBuf[3],vin,(uint8_t*)&szRecvBuf[4]);
//	if(memcmp(vin,&szRecvBuf[4],17) != 0)//车架号错误
//	{
//		return;
//	}
	
	//接收存储平台响应报文
	saveHistory(link,1,(uint8_t*)szRecvBuf,rLen,0,LOG);
	
	switch(szRecvBuf[2])
	{
		case CMD_LOGIN:  //响应车辆登入
		{
			if(szRecvBuf[3] == 1 && pSta->bLogined == 0xFF)
			{
				pSta->sendOverTimeCnt = 0;
				pSta->bLogined = 1;
			}
		}
		break;
		case CMD_REALDATA:  		//响应实时数据
		case CMD_REISSUEDATA:   //响应补发数据
		case CMD_HEARTBEAT:  		//响应心跳
		{
			if(szRecvBuf[3] == 1)
				pSta->sendOverTimeCnt = 0;
		}
		break;
		case CMD_TIMING:  //响应校时
		{
			if(szRecvBuf[3] == 1)
			{
				pSta->sendOverTimeCnt = 0;
				if(rLen == 31 && pSta->bTiming > 0 && pSta->bTiming < 4)
				{
					//网络延时在范围内
					RTC_INFOR sysTime;
					sysTime.year = 2000+szRecvBuf[24];
					sysTime.month = szRecvBuf[25];
					sysTime.day = szRecvBuf[26];
					sysTime.hour = szRecvBuf[27];
					sysTime.minute = szRecvBuf[28];
					sysTime.second = szRecvBuf[29];
					RTC_Time_Set(&sysTime);	
				}
				pSta->bTiming = 0xFF;				
			}
		}
		break;
		case CMD_LOGOUT:  //响应登出
		{
			if(szRecvBuf[3] == 1 && pSta->bLogouted != 0)
				pSta->bLogouted = 1;
		}
		break;
		case 	CMD_QUERYPARAS:
		{
			uppackQueryParas(pSta,szRecvBuf,rLen);
		}
		break;
		case CMD_SETPARAS:
		{
			uppackSetParas(pSta,szRecvBuf,rLen);
		}
		break;
		case CMD_CTRL:
		{
			uppackCtrl(pSta,szRecvBuf,rLen);
		}
		break;
	}
	return;
}



__WEAK void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	return NULL;
}

__WEAK uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	return 1;
}
__WEAK uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	return 0;
}

__WEAK uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	return 0;
}

__WEAK uint8_t extCtrlNetSta(void* obj)
{
	return 0;
}

__WEAK uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
 return 0;
}
