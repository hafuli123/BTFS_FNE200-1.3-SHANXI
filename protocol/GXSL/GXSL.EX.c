#include "analyse_Bms_GXSL.h"

#define MAX_YZT_LINK 1
static uint16_t MakeCmd(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *wbuf,uint16_t dataLen);
extern uint16_t packUpdataVIN(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen);					//首次获取VIN 上报
static uint8_t sendNetDatas(uint8_t socket_fd, uint8_t *buf, uint16_t len);

extern char realVin[18];
uint8_t isChgVIN = 0;

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


/* 用户自定义参数 */
static GBSTA gbSta[MAX_YZT_LINK] = {0};
USER_DATA gUserDara;												//用户数据存储
SelfData0A* pSelfData0A;										//华晨鑫源地上铁 自定义数据				
static uint16_t p0Aoffset;									//0A指令存储位置偏移 

/* 联网控制参数 */
static uint32_t reConnectStamp = 0;					//重播计时
static uint8_t printPackCmd = 1;						//打印报文到串口控制位

/* 休眠唤醒参数 */
static int wakelock = -1;										//唤醒锁
static uint64_t wakeTime = 0;								//唤醒时间
static uint32_t wk_timesss = 0;							//唤醒倒计时
uint8_t wk_Excute = 0;											//唤醒标志控制
uint8_t allow_UserOnLine = 0;								//允许用户平台联网

/* VIN变更参数 */
RTC_INFOR getVINTime;												//首次获取VIN的时间 
double glongd;									            //经度
double glatd;									              //纬度
uint8_t fisrtGetVIN = 0;										//是否时首次获取VIN 1是，0否 
uint8_t getCanLogSwitch = 0;								//远程CAN日志采集开关
static uint32_t reportVINStamp = 0;					//上报首次获取VIN计时
static uint32_t reportVINInterval = 10000;				//默认上送间隔是10s
uint8_t reportVINfaildCnt = 0;										//上送VIN失败次数




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
		}
		else
		{
			//登出后重置 VIN码变更的控制参数
			isChgVIN = 0;
			reportVINfaildCnt = 0;
			reportVINInterval = 10000;
		}
			return 1;
	}
