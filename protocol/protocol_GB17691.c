#include "bsp_sys.h"
#include "protocol_GB17691.h"
#include "bsp_storage.h"
#include "bsp_rtc.h"
#include "string.h"
#include "algo_verify.h"
#include "Fun_Net.h"
#include "se.h"
#include "types.h"
#include "fun_can.h"

#define MAX_GB_LINK 2

enum COMMAND_CODE
{
	CMD_NONE = 0,
	CMD_LOGIN,                 			//车辆登入
	CMD_REALDATA,              			//实时数据
	CMD_REISSUEDATA,           			//补发数据
	CMD_LOGOUT,                			//车辆登出
	CMD_TIMING,               			//校时
	CMD_ANTITAMPER,						 			//防拆
	CMD_SETUP = 0x07,					 			//备案
	CMD_RSP_SETUP = 0x08,			 			//备案应答
	CMD_PARA_QUERY = 0x80,					//参数设置
};

//掉电保存链路参数
typedef struct _CFG
{
	uint32_t regTimeStamp;					//注册时间戳标志
	uint32_t realTimeStamp;					//实时信息时间戳标志
	uint32_t antiTimeStamp;					//防拆信息时间戳标志
	uint16_t regNo;									//注册流水号
	uint16_t realNo;								//实时流水号
	uint16_t antiNo;								//防拆流水号
}CFG;

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;										//是否使用
	char *vin;											//车架号
	char* domain;										//链路域名
	uint32_t* port;									//链路端口号
	uint8_t proc;										//燃油车平台协议类型 0:17691 1:环境部无签名加密 2:环境部数据签名 3:环境部数据加密
	//平台连接状态参数
	uint8_t bLogined;       				//登入标志位 0:未登入 1:已登入 0xFF:发送登入中
	uint8_t bLogouted;							//登出标志位 0:未登出 1:已登出
	uint8_t bTiming;								//校时标志
	uint8_t sendOverTimeCnt;				//发送超时计数
	uint8_t sendActiCnt;						//发送激活计数
	uint8_t reissueSta;							//补发状态，0：需要补发 1：补发完成
	//实时计时控制
	uint8_t realSec;								//实时秒针
	uint8_t realSecCnt;							//实时秒计数
	uint8_t sendObdFlag;						//发送OBD数据标志 0:不发送 1：发送了一次 2：发送了2次 其他：不发送
	//内部时间戳
	uint32_t antiStamp;							//防拆时间戳
	uint32_t reissueStamp;					//补发时间戳
	uint32_t heartbeatStamp;				//心跳时间戳
	uint32_t actitStamp;						//激活时间戳
	//其他参数
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffSize;							//发送数据缓冲区大小
	//补发文件名
	char reissueFileName[30];				//补发文件名
	uint8_t day;										//日
	//内部掉电保存参数
	CFG cfg;
}GBSTA;

static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc);			//车辆登入
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc);										//终端校时
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc);		//车辆登出
static uint16_t packAntiTamper(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc);	//终端防拆
static uint16_t packSetUp(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc);										//终端备案激活
static uint16_t buildRealData(uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc,uint8_t obdFlag);	//打包实时数据
static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen);																		//解包数据

static GBSTA gbSta[MAX_GB_LINK] = {0};

/*初始化配置*/
void* gb17691Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t proc)
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
	//外部参数
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].domain = domain;
	gbSta[objLinkIdx].port = port;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].proc = proc;
	//内部
	gbSta[objLinkIdx].bLogined = 0;
	gbSta[objLinkIdx].bLogouted = 0;
	gbSta[objLinkIdx].bTiming = 0;
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].sendActiCnt = 0;
	gbSta[objLinkIdx].reissueSta = 0;
	
	gbSta[objLinkIdx].realSec = 0xFF;
	gbSta[objLinkIdx].realSecCnt = 0;
	gbSta[objLinkIdx].antiStamp = 0;
	gbSta[objLinkIdx].reissueStamp = 0;
	gbSta[objLinkIdx].heartbeatStamp = 0;
	gbSta[objLinkIdx].actitStamp = 0;
	gbSta[objLinkIdx].buff = pBuff;
	gbSta[objLinkIdx].buffSize = pBuffSize;
	gbSta[objLinkIdx].day = g_system_dt.day;
	//掉电存储参数恢复
	readConfig(link,&gbSta[objLinkIdx].cfg,sizeof(gbSta[objLinkIdx].cfg),0);
	//同步日志文件
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),LOG);
	//同步补发文件
	syncHistory(gbSta[objLinkIdx].bLink,gbSta[objLinkIdx].reissueFileName,sizeof(gbSta[objLinkIdx].reissueFileName),REC);
	//返回当前链路参数状态指针
	return &gbSta[objLinkIdx];
}

/*参数：链路状态参数 控制 0:可以登录平台 1:登出平台*/
uint8_t gb17691Run(void* obj)
{
	static uint8_t ctrl = 0;
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
	if(Fun_Gprs_Tcp_Sta(pSta->bLink) == FUN_GPRS_CONNECTED)
	{
		svrSta = 1;
	}
	//if(fun_can_Get_State(BSP_CAN) > 0 && (gRealData.carState == 1 || gRealData.chargeState == 1))
	//if(1)
	if(fun_can_Get_State(BSP_CAN) > 0 && ((gRealData.engineSpeed <= 10 || gRealData.engineSpeed >= 0xFFFE) && (gRealData.speed == 0 || gRealData.speed >= 0xFFFE)) == 0)
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
			ctrl = 1;
		}
	}
	else
	{
		endTicks = ticks;
		if(gRealData.chargeState == 4)
		{
			if(ticks - startTicks >= 360000)
			{
				ctrl = 0;
			}
		}
		else
		{
			if(ticks - startTicks >= 10000)
			{
				ctrl = 0;
			}			
		}
		if(ctrl == 0 && svrSta == 1 && pSta->bLogined != 1)
		{
			Fun_Gprs_Tcp_disconnect(pSta->bLink);
		}
	}
	if(pSta->realSec != gRealData.second)
	{
		pSta->bTiming = (pSta->bTiming >= 1 && pSta->bTiming < 0xFF) ? pSta->bTiming + 1: pSta->bTiming;
		pSta->realSec = gRealData.second;
		//周期控制
		if(++pSta->realSecCnt >= 10)
		{
			uint32_t realTimeStamp = g_system_dt.year << 16 | g_system_dt.month << 8 | g_system_dt.day << 0;
			//实时数据流水号
			if(pSta->cfg.realTimeStamp != realTimeStamp)
			{
				pSta->cfg.realTimeStamp = realTimeStamp;
				pSta->cfg.realNo = 0;
			}
			pSta->cfg.realNo++;
			saveConfig(pSta->bLink,&pSta->cfg,sizeof(pSta->cfg),0);
			//发送OBD计数，24小时至少上送一次，当前获取后上送2次
			//gTerminalState.obdState = 1;
			pSta->sendObdFlag = pSta->sendObdFlag < 3 ? pSta->sendObdFlag + 1:pSta->sendObdFlag;
			pSta->sendObdFlag = gTerminalState.obdState == 0 ? 0 : pSta->sendObdFlag;
			pSta->realSecCnt = 0;
			dataLen =	buildRealData(pSta->bLogined == 0 ? CMD_REISSUEDATA : CMD_REALDATA,pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.realNo,pSta->proc,pSta->sendObdFlag >= 1 && pSta->sendObdFlag <= 2);
			if(dataLen > 0)
			{
				if(pSta->bLogined == 0)
				{
					//存储离线数据
					saveHistory(pSta->bLink,pSta->bLogined,pSta->buff,dataLen,0,REC);
				}
			}
		}
	}
	//平台交互
	if(svrSta == 1 && (ctrl == 1 || (pSta->bLogined == 1 && pSta->bLogouted == 0)))//网络正常，任务开启或终端已经登入且未登出
	{
		active = 1;
		if(pSta->sendOverTimeCnt >= 3 &&  ticks - pSta->heartbeatStamp >= 3000)
		{
			pSta->sendOverTimeCnt = 0;
			//连续3次超3秒响应超时,重播
			Fun_Gprs_Tcp_disconnect(pSta->bLink);
		}
		else if(pSta->bLogined == 1)
		{
			//登入后，正常交互报文
			if(dataLen > 0)//发送实时报文
			{
				pSta->heartbeatStamp = ticks;
				pSta->sendOverTimeCnt++;
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
				saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
			}
			else if(pSta->bTiming == 0)//校时
			{
				pSta->bTiming = 1;
				if((dataLen = packTiming(pSta->vin,pSta->buff,pSta->buffSize,pSta->proc)) > 0)
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			}
			else if(pSta->reissueFileName[0] != 0 && ticks - pSta->reissueStamp >= 500)//补发报文
			{
				pSta->reissueStamp = ticks;
				dataLen = readHistory(pSta->bLink,pSta->reissueFileName,pSta->buff,pSta->buffSize,REC);
				if(dataLen > 0)
				{
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
					saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
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
			else if(pSta->proc != 0 && gTerminalState.powerState == 0 && ticks - pSta->antiStamp >= 60000)//防拆
			{
				//流水号
				if(pSta->cfg.antiTimeStamp != g_system_dt.year)
				{
					pSta->cfg.antiTimeStamp = g_system_dt.year;
					pSta->cfg.antiNo = 0;
				}
				pSta->cfg.antiNo++;
				saveConfig(pSta->bLink,&pSta->cfg,sizeof(pSta->cfg),0);
				pSta->antiStamp = ticks;
				if((dataLen = packAntiTamper(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.antiNo,pSta->proc)) > 0)
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			}
			else if(ctrl == 0 && pSta->bLogouted == 0)//登出
			{
				pSta->bLogouted = 1;
				if((dataLen = packLogout(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo,pSta->proc)) > 0)
				{
					Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
					saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
				}
			}
		}
		else if(ticks - pSta->heartbeatStamp >= 20000 && pSta->bLogined != 1)//登入
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
			if((dataLen = packLogin(pSta->vin,pSta->buff,pSta->buffSize,pSta->cfg.regNo,pSta->proc)) > 0)
			{
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
				saveHistory(pSta->bLink,1,pSta->buff,dataLen,0,LOG);
			}
			pSta->sendOverTimeCnt++;
		}
		/*17691无需应答*/
		if(pSta->proc == 0)
		{
			pSta->sendOverTimeCnt = 0;
			if(pSta->bLogined == 0xFF)
			{
				pSta->bLogined = 1;
			}
		}
	}
	//离线参数清除
	if(svrSta == 0 || pSta->bLogined == 0  || pSta->bLogouted == 1)
	{
		//无法联网或链路未使用且完全停止，相关变量清零
		pSta->reissueSta = 0;
		pSta->bLogined = 0;
		pSta->bLogouted = 0;
		pSta->bTiming = 0;
		pSta->sendActiCnt = 0;
		pSta->heartbeatStamp = 0;
	}
	//防拆改激活备案
	if(pSta->proc == 2 && gTerminalState.obdState == 1 && CheckVin(gRealData.vin) == 1 && gFrimPara.setUpFlag != 0xAA)
	{
		const uint8_t setUpLink = 2;
		if(Fun_Gprs_Tcp_Sta(setUpLink) == FUN_GPRS_CONNECTED)
		{
			if(ticks - pSta->actitStamp >= 120000 && pSta->sendActiCnt < 3)
			{
				pSta->actitStamp = ticks;
				pSta->sendActiCnt++;
				if((dataLen = packSetUp(pSta->vin,pSta->buff,pSta->buffSize,pSta->proc)) > 0)
					Fun_Gprs_Tcp_send(setUpLink,pSta->buff,dataLen);		
			}
		}
		else
		{
			Fun_Gprs_Tcp_connect(setUpLink,"g6check.vecc.org.cn",19006);
			Fun_Gprs_Tcp_Set_RecvCallBack(setUpLink,unpack);
		}
	}
	return active;	
}

/*获取链路状态*/
uint8_t gb17691GetSta(uint8_t id)
{
	if(id < MAX_GB_LINK)
		return (gbSta[id].bLogined == 1);
	return 0;
}

//添加数据报头及校验
static uint16_t MakeCmd(uint8_t cmd,char * vin,uint8_t *wbuf,uint16_t dataLen,uint8_t proc)
{
	int rv = SUCCEED;
	uint32_t sm2Len = dataLen + 96;
		
//	if(cmd != CMD_SETUP)
//	{
		wbuf[0] = 0x23;
		wbuf[1] = 0x23;
		wbuf[2] = cmd;																										//命令单元
		memcpy(&wbuf[3],gSysPara.vinCode,17);         										//车架号	
		wbuf[20] = SW_VERSION_NUM;																				//终端软件版本号
//	}
//	else
//	{
//		wbuf[0] = 0x7E;
//		wbuf[1] = 0x7E;
//		wbuf[2] = cmd;																										//命令单元
//		wbuf[3] = 0xFE;																										//应答标志
//		memcpy(&wbuf[4],gSysPara.vinCode,17);         										//车架号	
//	}
	wbuf[21] = 0x01;//数据加密方式
	if(proc == 3)
	{
		wbuf[21] = 0x03;//SM2加密
		if(dataLen > 0)
		{
			if(cmd != CMD_SETUP)
			{
//				rv = tms_sm2_encrypt(&wbuf[24],dataLen,&wbuf[24],&sm2Len,0x0002);
			}
			else
			{
//				rv = tms_sm2_encrypt(&wbuf[24],dataLen,&wbuf[24],&sm2Len,0x0003);//备案国家平台公钥
			}
			if(rv == SUCCEED)
			{
				dataLen += 96;
			}
			else
			{
				//加密失败
				return 0;
			}
		}
	}
	wbuf[22] = dataLen >> 8;
	wbuf[23] = dataLen;
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

//打包登入数据
static uint16_t packLogin(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc)
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
	return MakeCmd(CMD_LOGIN,vin,buff,DataAreLen,proc);
}

//打包校时数据
static uint16_t packTiming(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc)
{
	return MakeCmd(CMD_TIMING,vin,buff,0,proc);	
}

//打包登出数据
static uint16_t packLogout(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t regNo,uint8_t proc)
{
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	buff[30] = (regNo) >> 8;
	buff[31] = (regNo);
	return MakeCmd(CMD_LOGOUT,vin,buff,8,proc);
}

//打包防拆数据
static uint16_t packAntiTamper(char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc)
{
	uint8_t index = 24;
	uint32_t n32Val;
	buff[index++] = (uint8_t)(g_system_dt.year - 2000);
	buff[index++] = (uint8_t)(g_system_dt.month);
	buff[index++] = (uint8_t)(g_system_dt.day);
	buff[index++] = (uint8_t)(g_system_dt.hour);
	buff[index++] = (uint8_t)(g_system_dt.minute);
	buff[index++] = (uint8_t)(g_system_dt.second);	
	buff[index++] = msgNo >> 8;         															//注册流水号H
	buff[index++] = msgNo;
	buff[index++] = gRealData.locationState;													//定位状态
	n32Val = gRealData.longd * 1000000;																//经度转换
	buff[index++] = n32Val >> 24;																			
	buff[index++] = n32Val >> 16;																			
	buff[index++] = n32Val >> 8;																			
	buff[index++] = n32Val >> 0;																		
	n32Val = gRealData.latd * 1000000;																//纬度转换
	buff[index++] = n32Val >> 24;																			
	buff[index++] = n32Val >> 16;																			
	buff[index++] = n32Val >> 8;																			
	buff[index++] = n32Val >> 0;						
	return MakeCmd(CMD_ANTITAMPER,vin,buff,index - 24,proc);
}

//打包备案数据
static uint16_t packSetUp(char* vin,uint8_t* buff,uint16_t maxLen,uint8_t proc)
{
	uint32_t sm2Len = 64;
	uint16_t DataAreLen = 0;
	uint8_t index = 24;
	uint8_t priKeyVal[64];
	/*时间*/
	buff[index++] = (uint8_t)(g_system_dt.year - 2000);
	buff[index++] = (uint8_t)(g_system_dt.month);
	buff[index++] = (uint8_t)(g_system_dt.day);
	buff[index++] = (uint8_t)(g_system_dt.hour);
	buff[index++] = (uint8_t)(g_system_dt.minute);
	buff[index++] = (uint8_t)(g_system_dt.second);
	DataAreLen += 6;
	if(strlen(gFrimPara.scyId) == 16)																			//获取芯片ID
	{
		memcpy(&buff[index],gFrimPara.scyId,16);
		index += 16;
		DataAreLen += 16;
//		if(tms_sm2_export_pubkey(0x0001,&buff[index],&sm2Len) == SUCCEED)																	//获取公钥A
//		{
//			index += sm2Len;
//			DataAreLen += sm2Len;
//			memcpy(&buff[index],vin,17);
//			index += 17;
//			DataAreLen += 17;
//			if(proc == 2 || proc == 3)
//			{
//				if(tms_sm2_with_sm3_signature(&buff[24],DataAreLen,priKeyVal,&sm2Len,0x0001,(uint8_t *)gFrimPara.scyId) == SUCCEED)
//				{
//					buff[index++] = 32;																				//签名R值长度
//					DataAreLen++;
//					
//					memcpy(&buff[index],priKeyVal,32);												//签名R值
//					index	 += 32;
//					DataAreLen += 32;
//					
//					
//					buff[index++] = 32;																				//签名S值长度
//					DataAreLen++;
//					
//					memcpy(&buff[index],&priKeyVal[32],32);										//签名S值
//					index += 32;
//					DataAreLen += 32;
//				}
//			}
//			return MakeCmd(CMD_SETUP,vin,buff,index - 24,proc);			
//		}
	}
	return 0;
}

/*打包OBD数据*/
static uint16_t PackObdData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = pRealData->obdDiagProt;							//OBD诊断协议
		bPackBuf[Idx++] = pRealData->milState;								//MIL状态
		/*诊断支持状态*/
		bPackBuf[Idx++] = pRealData->diagSpState.value >> 8;	//诊断支持状态高字节
		bPackBuf[Idx++] = pRealData->diagSpState.value;				//诊断支持状态低字节
		/*诊断就绪状态*/
		bPackBuf[Idx++] = pRealData->diagRdyState.value >> 8;	//诊断就绪状态高字节
		bPackBuf[Idx++] = pRealData->diagRdyState.value;			//诊断就绪状态低字节
		/*车辆识别号*/
		memcpy(&bPackBuf[Idx], pRealData->vin,17);						//实时VIN
		Idx	+=	17;
		/*软件标定识别号*/
		memcpy(&bPackBuf[Idx], pRealData->softCalId,18);			//软件标定识别号
		Idx	+=	18;	
		/*标定验证码（CVN）*/
		memcpy(&bPackBuf[Idx], pRealData->cvn,18);						//CVN
		Idx	+=	18;	
		/*IUPR值*/
		memcpy(&bPackBuf[Idx], pRealData->iuprVal,36);				//IUPR
		Idx	+=	36;	
		if(pRealData->faultCodeCnt <= MAX_FAULT_NUM)
		{
			uint8_t i = 0;
			/*故障码总数*/
			bPackBuf[Idx++] = pRealData->faultCodeCnt;					//故障码总数
			//故障码信息列表
			for(i=0;i<pRealData->faultCodeCnt;i++)
			{
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 24;	//故障码
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 16;	//故障码
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 8;		//故障码
				bPackBuf[Idx++] = pRealData->faultCode[i] >> 0;		//故障码
			}
		}
		else
		{
			bPackBuf[Idx++] = 0xFE;															//故障码总数无效
		}
	}
	return Idx;
}

/*打包数据流*/
static uint16_t PackVehicleData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	uint32_t n32Val = 0;
	if(bPackBuf != 0)
	{
		if(pRealData->speed != 0xFFFF)													//车速转换
		{
			n32Val = pRealData->speed * 256;					
			n32Val &= 0xFFC0;
		}
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;													//车速高字节
		bPackBuf[Idx++] = n32Val;																//车速低字节
		if(pRealData->barometric != 0xFF)
			bPackBuf[Idx++] = pRealData->barometric * 2;					//大气压力
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->engineTorque != 0xFF)
			bPackBuf[Idx++] = pRealData->engineTorque + 125;			//发动机输出扭矩/实际扭矩
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->frictionTorque != 0xFF)
			bPackBuf[Idx++] = pRealData->frictionTorque + 125;		//摩擦扭矩
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->engineSpeed != 0xFFFF)
			n32Val = pRealData->engineSpeed * 8;									//发送机转速转换
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;													//发送机转速高字节
		bPackBuf[Idx++] = n32Val;																//发送机转速低字节
		if(pRealData->engineFuelFlow != 0xFFFF)
			n32Val = pRealData->engineFuelFlow * 20;							//发动机燃料流量转换
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;													//发动机燃料流量高字节
		bPackBuf[Idx++] = n32Val;																//发动机燃料流量低字节
		
		if(pRealData->technology == 0)									//技术路线
		{
			if(pRealData->scrUpperNOxSensor != 0xFFFF)
				n32Val = (pRealData->scrUpperNOxSensor + 200) * 20;	//SCR上游NOx传感器输出值转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;												//SCR上游NOx传感器输出值高字节
			bPackBuf[Idx++] = n32Val;															//SCR上游NOx传感器输出值低字节	
			if(pRealData->scrLowerNOxSensor != 0xFFFF)
				n32Val = (pRealData->scrLowerNOxSensor + 200) * 20;	//SCR下游NOx传感器输出值转换
			else
				n32Val = 0xFFFF;		
			bPackBuf[Idx++] = n32Val >> 8;												//SCR下游NOx传感器输出值高字节
			bPackBuf[Idx++] = n32Val;															//SCR下游NOx传感器输出值低字节	
			if(pRealData->reagentSurplus != 0xFF)
				bPackBuf[Idx++] = (uint8_t)(pRealData->reagentSurplus * 2.5l);		//反应剂余量
			else
				bPackBuf[Idx++] = 0xFF;
			if(pRealData->intakeFlowrate != 0xFFFF)
				n32Val = pRealData->intakeFlowrate * 20;													//进气量转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//进气量高字节
			bPackBuf[Idx++] = n32Val;																						//进气量低字节
			if(pRealData->scrInletTemp != 0xFFFF)
				n32Val = (pRealData->scrInletTemp + 273) * 32;										//SCR入口温度转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//SCR入口温度高字节
			bPackBuf[Idx++] = n32Val;																						//SCR入口温度低字节	
			if(pRealData->scrOutletTemp != 0xFFFF)
				n32Val = (pRealData->scrOutletTemp + 273) * 32;										//SCR出口温度转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//SCR出口温度高字节
			bPackBuf[Idx++] = n32Val;																						//SCR出口温度低字节	
			if(pRealData->dpfPressDiff != 0xFFFF)
				n32Val = pRealData->dpfPressDiff * 10;														//dpf压差转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//dpf压差高字节
			bPackBuf[Idx++] = n32Val;																						//dpf压差低字节	
			if(pRealData->engineCoolantTemp != 0xFF)
				bPackBuf[Idx++] = pRealData->engineCoolantTemp + 40;							//发动机冷却液温度
			else
				bPackBuf[Idx++] = 0xFF;
			if(pRealData->tankLevel != 0xFF)
				bPackBuf[Idx++] = (uint8_t)(pRealData->tankLevel * 2.5l);					//油箱液位
			else
				bPackBuf[Idx++] = 0xFF;		
		}
		else if(pRealData->technology == 1)									//技术路线
		{
			if(gRealData.twcUpperOxySensor != 0xFFFF)
				n32Val = gRealData.twcUpperOxySensor / 0.0000305f;								//TWC上游Oxy传感器输出值
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC上游Oxy传感器输出值高字节
			bPackBuf[Idx++] = n32Val;																						//TWC上游Oxy传感器输出值低字节	
			bPackBuf[Idx++] = (uint8_t)(gRealData.twcLowerOxySensor * 100);			//TWC下游Oxy传感器输出值
			if(gRealData.intakeFlowrate != 0xFFFF)
				n32Val = gRealData.intakeFlowrate * 20;														//进气量转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//进气量高字节
			bPackBuf[Idx++] = n32Val;																						//进气量低字节
			if(gRealData.engineCoolantTemp != 0xFF)
				bPackBuf[Idx++] = gRealData.engineCoolantTemp + 40;								//发动机冷却液温度
			else
				bPackBuf[Idx++] = 0xFF;
			if(gRealData.twcTemp != 0xFFFF)
				n32Val = (gRealData.twcTemp + 273) * 32;													//TWC温度转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC温度高字节
			bPackBuf[Idx++] = n32Val;																						//TWC温度低字节	
		}
		if(pRealData->technology == 2)
		{
			if(gRealData.twcUpperOxySensor != 0xFFFF)
				n32Val = gRealData.twcUpperOxySensor / 0.0000305f;								//TWC上游Oxy传感器输出值
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC上游Oxy传感器输出值高字节
			bPackBuf[Idx++] = n32Val;																						//TWC上游Oxy传感器输出值低字节	
			bPackBuf[Idx++] = (uint8_t)(gRealData.twcLowerOxySensor * 100);			//TWC下游Oxy传感器输出值
			
			if(gRealData.twcLowerNOxSensor != 0xFFFF)
				n32Val = (gRealData.twcLowerNOxSensor + 200) * 20;								//TWC下游NOx传感器输出值转换
			else
				n32Val = 0xFFFF;		
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC下游NOx传感器输出值高字节
			bPackBuf[Idx++] = n32Val;																						//TWC下游NOx传感器输出值低字节				
			
			if(gRealData.intakeFlowrate != 0xFFFF)
				n32Val = gRealData.intakeFlowrate * 20;														//进气量转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//进气量高字节
			bPackBuf[Idx++] = n32Val;																						//进气量低字节
			if(gRealData.engineCoolantTemp != 0xFF)
				bPackBuf[Idx++] = gRealData.engineCoolantTemp + 40;								//发动机冷却液温度
			else
				bPackBuf[Idx++] = 0xFF;
			if(gRealData.twcTemp != 0xFFFF)
				n32Val = (gRealData.twcTemp + 273) * 32;													//TWC温度转换
			else
				n32Val = 0xFFFF;
			bPackBuf[Idx++] = n32Val >> 8;																			//TWC温度高字节
			bPackBuf[Idx++] = n32Val;																						//TWC温度低字节			
		}
		
		bPackBuf[Idx++] = pRealData->locationState;														//定位状态
		n32Val = pRealData->longd * 1000000;																	//经度转换
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;																		
		n32Val = pRealData->latd * 1000000;																		//纬度转换
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;																		
		n32Val = pRealData->totalMileage * 10;																//总里程转换
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;
		
		if(gSysPara.carType == 1 && pRealData->soc != 0 && pRealData->soc != 0xFF)
		{
			/*混合动力电动车辆附加数据流*/
			bPackBuf[Idx++] = 0x04;																			//混合动力电动车辆附加数据流
			/*时间*/
			bPackBuf[Idx++] = (uint8_t)(pRealData->year - 2000);
			bPackBuf[Idx++] = (uint8_t)(pRealData->month);
			bPackBuf[Idx++] = (uint8_t)(pRealData->day);
			bPackBuf[Idx++] = (uint8_t)(pRealData->hour);
			bPackBuf[Idx++] = (uint8_t)(pRealData->minute);
			bPackBuf[Idx++] = (uint8_t)(pRealData->second);
			bPackBuf[Idx++] = pRealData->motorData[0].motorSpeed >> 8;	//电机转速
			bPackBuf[Idx++] = pRealData->motorData[0].motorSpeed;				//电机转速
			bPackBuf[Idx++] = pRealData->motorData[0].motorLoad;				//电机负荷百分比
			n32Val = (uint16_t)(pRealData->total_volt);									//电池电压
			if(n32Val != 0xFFFF && n32Val != 0xFFFE)
				n32Val = (uint16_t)(pRealData->total_volt * 10);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 8);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 0);
			
			n32Val = (uint16_t)(pRealData->total_current);								//电池电流
			if(n32Val != 0xFFFF && n32Val != 0xFFFE)
				n32Val = (uint16_t)(pRealData->total_current * 10 + 10000);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 8);
			bPackBuf[Idx++] = (uint8_t)(n32Val >> 0);
			
			bPackBuf[Idx++] = (uint8_t)(pRealData->soc);									//SOC		
		}
	}
	return Idx;
}

/*打包 补充 数据流*/
#pragma diag_suppress 177
static uint16_t PackAuxData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t Idx = 0;
	uint32_t n32Val = 0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = pRealData->engineTorqueMode;										//发动机扭矩模式
		if(pRealData->acceleratorVal != 0xFF)
			bPackBuf[Idx++] = (uint8_t)(pRealData->acceleratorVal * 2.5l);	//油门踏板
		else
			bPackBuf[Idx++] = 0xFF;
		if(pRealData->EngTotalFuelUsed == 0xFFFFFFFF)
			n32Val= 0xFFFFFFFF;
		else
			n32Val = pRealData->EngTotalFuelUsed * 2;												//累计油耗
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;				
		if(pRealData->ureaTankTemp != 0xFF)
			bPackBuf[Idx++] = pRealData->ureaTankTemp + 40;									//尿素箱温度
		else
			bPackBuf[Idx++] = 0xFF;		
		if(pRealData->actAreaInjectVal == 0xFFFFFFFF)
			n32Val= 0xFFFFFFFF;
		else
			n32Val = pRealData->actAreaInjectVal * 100;											//实际尿素喷射量
		bPackBuf[Idx++] = n32Val >> 24;																			
		bPackBuf[Idx++] = n32Val >> 16;																			
		bPackBuf[Idx++] = n32Val >> 8;																			
		bPackBuf[Idx++] = n32Val >> 0;			
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 24;									//累计尿素消耗														
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 16;																			
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 8;																			
		bPackBuf[Idx++] = pRealData->totalUreaUsed >> 0;			
		if(pRealData->dpfExhaustTemp != 0xFFFF)
			n32Val = (pRealData->dpfExhaustTemp + 273) * 32;								//DPF排气温度转换
		else
			n32Val = 0xFFFF;
		bPackBuf[Idx++] = n32Val >> 8;																		//DPF排气温度高字节
		bPackBuf[Idx++] = n32Val;																					//DPF排气温度低字节
	}
	return Idx;
}

/*
*  函数说明: 打包实时数据用于存储
*  参数wbuf: 输入缓冲区 mode:0 17691格式 1:环境部格式 2:环境部格式加签名
*  返 回 值: 打包长度
*/
static uint16_t buildRealData(uint8_t cmd,char* vin,uint8_t* buff,uint16_t maxLen,uint16_t msgNo,uint8_t proc,uint8_t obdFlag)
{
	RealData* pRealData;
	uint8_t priKeyVal[64];
	uint16_t index,subLen,totalLen = 0;
	int8_t i;
	uint32_t sm2Len = 64;
	uint8_t outDataIdx;
	//环境部车辆报文起始,预留报文发送时间和流水号
	index = 32;
	for(i = 9;i >= 0;i--)
	{
		pRealData = getRealCache(i,&outDataIdx);
		i = outDataIdx;
		if(proc == 0)
		{
			//GB 17691打包方式
			uint8_t *pbuf;
			pbuf = &buff[totalLen];
			//数据发送时间
			index = 24;
			pbuf[index++] = (uint8_t)(pRealData->year - 2000);
			pbuf[index++] = (uint8_t)(pRealData->month);
			pbuf[index++] = (uint8_t)(pRealData->day);
			pbuf[index++] = (uint8_t)(pRealData->hour);
			pbuf[index++] = (uint8_t)(pRealData->minute);
			pbuf[index++] = (uint8_t)(pRealData->second);
			//发动机数据
			pbuf[index++] = 0x02;
			//流水号
			pbuf[index++] = ((msgNo * 10) + 9 - i) >> 8;																			
			pbuf[index++] = ((msgNo * 10) + 9 - i) >> 0;
			index += PackVehicleData(pRealData,&pbuf[index]);			//打包数据流
			subLen = MakeCmd(cmd,vin,pbuf,index - 24,proc);
			if(obdFlag == 1 && i == 0)
			{
				totalLen += subLen;
				pbuf = &buff[totalLen];
				//数据发送时间
				index = 24;
				pbuf[index++] = (uint8_t)(pRealData->year - 2000);
				pbuf[index++] = (uint8_t)(pRealData->month);
				pbuf[index++] = (uint8_t)(pRealData->day);
				pbuf[index++] = (uint8_t)(pRealData->hour);
				pbuf[index++] = (uint8_t)(pRealData->minute);
				pbuf[index++] = (uint8_t)(pRealData->second);
				//OBD数据
				pbuf[index++] = 0x01;
				//流水号
				pbuf[index++] = ((msgNo * 10) + i) >> 8;																			
				pbuf[index++] = ((msgNo * 10) + i) >> 0;
				index += PackObdData(pRealData,&pbuf[index]);
				subLen = MakeCmd(cmd,vin,pbuf,index - 24,proc);
			}
			totalLen += subLen;
			continue;
		}
		else
		{
			//环境部打包模式
			//发动机数据
			if(pRealData->technology == 0)
			{
				/*数据流信息*/
				buff[index++] = 0x02;										//数据流信息
			}
			if(pRealData->technology == 1)
			{
				/*数据流信息*/
				buff[index++] = 0x03;										//数据流信息
			}
			if(pRealData->technology == 2)
			{
				/*数据流信息*/
				buff[index++] = 0x05;										//数据流信息
			}
			//数据发送时间
			buff[index++] = (uint8_t)(pRealData->year - 2000);
			buff[index++] = (uint8_t)(pRealData->month);
			buff[index++] = (uint8_t)(pRealData->day);
			buff[index++] = (uint8_t)(pRealData->hour);
			buff[index++] = (uint8_t)(pRealData->minute);
			buff[index++] = (uint8_t)(pRealData->second);	
			//信息体
			index += PackVehicleData(pRealData,&buff[index]);
			if(obdFlag == 1 && i == 0)
			{
				//OBD数据
				buff[index++] = 0x01;	
				//数据发送时间
				buff[index++] = (uint8_t)(pRealData->year - 2000);
				buff[index++] = (uint8_t)(pRealData->month);
				buff[index++] = (uint8_t)(pRealData->day);
				buff[index++] = (uint8_t)(pRealData->hour);
				buff[index++] = (uint8_t)(pRealData->minute);
				buff[index++] = (uint8_t)(pRealData->second);	
				//信息体
				index += PackObdData(pRealData,&buff[index]);				
			}	
			if(gSysPara.linkSwitch & PACK_AUX_BIT)
			{
				//补充数据流
				buff[index++] = 0x80;
				//数据发送时间
				buff[index++] = (uint8_t)(pRealData->year - 2000);
				buff[index++] = (uint8_t)(pRealData->month);
				buff[index++] = (uint8_t)(pRealData->day);
				buff[index++] = (uint8_t)(pRealData->hour);
				buff[index++] = (uint8_t)(pRealData->minute);
				buff[index++] = (uint8_t)(pRealData->second);	
				//信息体
				index += PackAuxData(pRealData,&buff[index]);
			}

			if(i == 0)
			{
				//数据发送时间
				buff[24] = (uint8_t)(pRealData->year - 2000);
				buff[25] = (uint8_t)(pRealData->month);
				buff[26] = (uint8_t)(pRealData->day);
				buff[27] = (uint8_t)(pRealData->hour);
				buff[28] = (uint8_t)(pRealData->minute);
				buff[29] = (uint8_t)(pRealData->second);
				//流水号
				buff[30] = (uint8_t)(msgNo >> 8);
				buff[31] = (uint8_t)(msgNo >> 0);
				if(proc == 2 || proc == 3)
				{
					if(tms_sm2_with_sm3_signature(&buff[24],index - 24,priKeyVal,&sm2Len,0x0001,(uint8_t *)gFrimPara.scyId) == SUCCEED)
					{
						buff[index++] = 32;																				//签名R值长度
						memcpy(&buff[index],priKeyVal,32);												//签名R值
						index	 += 32;
						buff[index++] = 32;																				//签名S值长度
						memcpy(&buff[index],&priKeyVal[32],32);										//签名S值
						index += 32;
					}
				}
				totalLen = MakeCmd(cmd,vin,buff,index - 24,proc);
			}
		}
	}
	return totalLen;
}

uint8_t rspBuf[50];
static void unpack(uint8_t link,uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i;
	GBSTA* pSta = NULL;
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
	if(rLen < 25 )//数据长度不足
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
	else if(rLen != (szRecvBuf[22] << 8 | szRecvBuf[23]) + 25)//报文长度错误
  {
		return;
	}
	if(memcmp(pSta->vin,&szRecvBuf[3],17) != 0)//车架号错误
	{
		return;
	}
	switch(szRecvBuf[2])
	{
		case CMD_LOGIN:  //响应车辆登入
		{
			if(pSta->bLogined == 0xFF)
			{
				pSta->sendOverTimeCnt = 0;
				pSta->bLogined = 1;
			}
		}
		break;
		case CMD_REALDATA:  		//响应实时数据
		case CMD_REISSUEDATA:   //响应补发数据
		case CMD_ANTITAMPER:
		{
			pSta->sendOverTimeCnt = 0;
		}
		break;
		case CMD_TIMING:  //响应校时
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
		break;
		case CMD_LOGOUT:  //响应登出
		{
			if(pSta->bLogouted != 0)
				pSta->bLogouted = 1;
		}
		break;
		case CMD_RSP_SETUP:  //响应备案
		{
			//备案成功或重复备案
			//01成功 0201已经备案 0202VIN错误 0203未知错误
			if(szRecvBuf[24]==0x01 || (szRecvBuf[24]==0x02 && szRecvBuf[25]==0x01))
			{
				//备案成功，保存成功标志，重启正式上送数据
				gFrimPara.setUpFlag = 0xAA;
				Device_ID_Save();		
			}
		}
		break;
		case CMD_PARA_QUERY:  //参数
		{
			uint16_t resLen = 0;
			rspBuf[24] = (uint8_t)(g_system_dt.year - 2000);
			rspBuf[25] = (uint8_t)(g_system_dt.month);
			rspBuf[26] = (uint8_t)(g_system_dt.day);
			rspBuf[27] = (uint8_t)(g_system_dt.hour);
			rspBuf[28] = (uint8_t)(g_system_dt.minute);
			rspBuf[29] = (uint8_t)(g_system_dt.second);
			resLen = MakeCmd(CMD_PARA_QUERY,pSta->vin,rspBuf,6,pSta->proc);
			Fun_Gprs_Tcp_send(pSta->bLink,rspBuf,resLen);
		}
		break;
		default:
		{
			;
		}
		break;
	}
	return;
}

