#include "pdu.h"
#include "stdio.h"
#include "bsp_sys.h"
#include "fun_can.h"
#include "terminal_Server.h"
#include "protocol_GB32960.h"
#include "bsp_rtc.h"
#include "Fun_Net.h"
#include "bsp_uart_fifo.h"
#include "bsp_io.h"
#include "string.h"
#include "algo_verify.h"
#include "se.h"
#include "bsp_storage.h"
#include "types.h"
#include "bsp_power.h"
#include "rl_usb.h"
#include "rl_fs.h"
#include "fun_mon.h"
#include "bsp_gps.h"
#include "protocol_GB32960.h"
#include "protocol_GB17691.h"
#include "bsp_wdg.h"


#define TERM_SERVER 5
#define MAX_SENDLEN 1024
#define MAX_LOCK_TIME_CNT 60000


enum COMMAND_CODE
{
	CMD_NONE = 0,
	CMD_LOGIN = 0x71,                 //终端注册
	CMD_SCY_CHK = 0x72,               //安全校验
	CMD_HEARTBEAT = 0x07,     				//心跳
	CMD_QUERYPARAS = 0x80,     				//查询命令
	CMD_SETPARAS = 0x81,              //设置命令
	CMD_CTRL = 0x82,                  //控制命令
	CMD_SEND_FILE = 0x83, 						//上位机检测升级
};

enum PARA_ID
{
	PARA_INTERVAL_SAVE = 0x04,					//本地存储周期
	PARA_INTERVAL_REALSEND = 0x05,			//正常上报周期
	PARA_INTERVAL_WARNSEND = 0x06,			//报警上报周期
	PARA_INTERVAL_HEARDSEND = 0x07,			//心跳上报周期

	//终端参数
	PARA_DEV_VER = 0x80,								//硬件版本
	PARA_ICCID = 0x81,                 	//ICCID
	PARA_IMEI = 0x82,     				 			//IMEI
	PARA_IMSI = 0x83,     				 			//IMSI
	PARA_PUBKEY = 0x84,                	//终端公钥
	PARA_DEVID = 0x85,									//终端ID
	PARA_SIGNID = 0x86,									//签名ID
	PARA_VIN = 0x87,										//车架号
	PARA_QB_ID = 0x88,									//企标ID
	PARA_QB_URL = 0x89,									//企标地址
	PARA_QB_PORT = 0x8A,								//企标端口
	PARA_EV2_URL = 0x8B,								//EV第二链路地址
	PARA_EV2_PORT = 0x8C,								//EV第二链路端口
	PARA_EV3_URL = 0x8D,								//EV第三链路地址
	PARA_EV3_PORT = 0x8E,								//EV第三链路端口
	PARA_FVHJ_URL = 0x8F,								//FV环境部链路地址
	
	PARA_FVHJ_PORT = 0x90,							//FV环境部链路端口
	PARA_FV17691_URL = 0x91,						//FV17691链路地址
	PARA_FV17691_PORT = 0x92,						//FV17691链路端口
	PARA_QB_LINK_SW = 0x93,							//企标链路开启
	PARA_EV2LINK_SW = 0x94,							//EV第二链路开启
	PARA_EV3LINK_SW = 0x95,							//EV第三链路开启
	PARA_FVHJLINK_SW = 0x96,						//FV第一链路开启
	PARA_FV17691LINK_SW = 0x97,					//FV第二链路开启
	PARA_SIGN_SW = 0x98,								//终端签名开关
	PARA_PLAT_KEY = 0x99,								//企业平台公钥
	PARA_VECC_KEY = 0x9A,								//国家环保平台公钥
	PARA_SOFT_VER = 0x9B,								//软件版本
	
	/* 用户自定义数据 */
	PARA_WAKE_TIME = 0x9C,							//设置唤醒时间
	PARA_WAKE_INTERVAL = 0x9D,					//设置唤醒间隔
	PARA_WAKE_SENDTIME = 0x9E,					//设置唤醒后数据上送时间长度
	PARA_AUTOCHG_VOLT = 0x9F,						//需自动补电电压
	PARA_AUTOCHGOVER_VOLT  = 0xA0,			//停止自动补电电压
	PARA_UNDERVOLT = 0xA1,							//欠压	
	PARA_HTTPURL = 0xA2,								//httpURL
	PARA_HTTPUSERNAME = 0xA3,						//httpUserName
	PARA_HTTPPASSWORD = 0xA4,						//httppassword
	PARA_GETCANLOG = 0xA5,							//CAN日志开关

	
	//终端状态
	PARA_REAL_ICCID = 0xC0,             //ICCID
	PARA_CSQ = 0xC1,             				//CSQ
	PARA_GPS_VIEW_START = 0xC2,         //GPS可视卫星
	PARA_GPS_USE_START = 0xC3,         	//GPS定位卫星
	PARA_EMMC_SIZE = 0xC4,							//EMMC容量
	PARA_CAN1_CNT = 0xC5,								//CAN1接收条数，CAN测试
	PARA_CAN2_CNT = 0xC6,								//CAN2接收条数，CAN测试
	PARA_CAN3_CNT = 0xC7,								//CAN3接收条数，CAN测试
	PARA_DEV_TIME = 0xC8,								//设备时间
	PARA_PWR_VOLT = 0xC9,								//电源电压
	PARA_BAT_VOLT = 0xCA,								//电池电压
	PARA_4G_VER = 0xCB,									//4G模块版本
	PARA_NET_STA = 0xCC,								//4G联网状态
	PARA_USB_STA = 0xCD,								//USB连接状态
	PARA_EMMC_STA = 0xCE,								//EMMC测试状态
	PARA_FLASH_STA = 0xCF,							//FLASH状态
	
	PARA_CAR_STA = 0xD0,								//车辆状态
	PARA_CHG_STA = 0xD1,								//充电状态
	PARA_ACC_STA = 0xD2,								//ACC状态
	PARA_QB_LINK_STA = 0xD3,						//企标链路状态
	PARA_EV2_LINK_STA = 0xD4,						//EV第二链路状态
	PARA_EV3_LINK_STA = 0xD5,						//EV第三链路状态
	PARA_FV1_LINK_STA = 0xD6,						//FV第一链路状态	
	PARA_FV2_LINK_STA = 0xD7,						//FV第二链路状态	
	PARA_GPS_ANT_STA = 0xD8,						//GPS天线状态
	PARA_CAR_TYPE = 0xD9,								//车型
};

static uint32_t sendtimeStamp = 0;				//数据发送时间
static uint8_t sendedLoginCnt;						//发送登录次数
static uint8_t bLogined;									//平台登录标志
static uint8_t szSendBuf[MAX_SENDLEN];		//发送缓冲区
static uint8_t szRecvBuf[MAX_SENDLEN + 50];		//接收缓冲区
static char imei[16];											//IMEI

static uint32_t unlockTime;								//解锁时间
static uint32_t unlockFlag;								//解锁标志
static uint8_t usbState = 0;							//USB链接状态
static uint8_t isCheckUSBModel = 1;				//是否检测Typec
static int wakelock = -1;									//唤醒锁
static uint64_t wakeTime = 300;						//唤醒时间

static void ResetSeverPars(void);					//重置参数
static void RedialFunc(void);							//重新拨号
static void UnpackTermData(uint8_t link,uint8_t *rbuf, uint16_t rLen);																						//解压终端接收数据
static uint16_t PackTermData(uint8_t link,uint8_t *sbuf,uint8_t cmd);																							//打包终端发送数据
static void PacketAnalyze(const uint8_t* rBuf, uint16_t rLen,void (*SendCallBack)(uint8_t * buff,uint16_t len));	//统一解包终端控制数据

static void ResetSeverPars(void)
{
	bLogined = 0;
	sendedLoginCnt = 0;
}

static uint8_t wakeCallback(uint8_t type)
{
	uint8_t dataLen;
	if(type == 0)
	{
		//定时唤醒
		if(Fun_Gprs_Tcp_Sta(TERM_SERVER) == FUN_GPRS_CONNECTED)
		{
			Fun_Gprs_WakeUp();
			dataLen = PackTermData(TERM_SERVER,szSendBuf,CMD_HEARTBEAT);
			if(dataLen > 0)
			{
				Fun_Gprs_Tcp_send(TERM_SERVER,szSendBuf,dataLen);
			}
			Fun_Gprs_Sleep();
		}
	}
	
	return 0;
}

void InitTermServer(void)
{
	unlockFlag = 0;
	ResetSeverPars();
	wakelock = bsp_wakelock_create();
}

static void sendNetdata(uint8_t * buff,uint16_t len)
{
	Fun_Gprs_Tcp_send(TERM_SERVER,buff,len);
}

static void sendUartData(uint8_t * buff,uint16_t len)
{
	comSendBuf(RS1_COM,buff,len);
	whistle(100,1);
}

static uint32_t SeedToKeyLevel1(uint32_t seed)
{
	uint8_t xor_data[4] = {0x92,0xB0,0xD5,0x11};
	uint8_t cal_data[4];
	uint8_t key_data[4];
	uint8_t *pSeed = (uint8_t*)&seed;
	uint8_t i;
	for(i=0;i<4;i++)
	{
		cal_data[i] = pSeed[i] ^ xor_data[i];
	}
	key_data[0] = ((cal_data[3] & 0x0F) << 4) | (cal_data[0] & 0xFF); 
	key_data[1] = (cal_data[1] & 0xD8) | ((cal_data[2] & 0xF0) >> 4); 
	key_data[2] = (cal_data[2] & 0xCF) | (cal_data[0] & 0xF0); 
	key_data[3] = (cal_data[3] & 0xF0) | ((cal_data[1] & 0xD0) >> 4); 
	return *((uint32_t*)key_data);
}

void termServerTask(void)
{
	uint16_t dataLen;
	/*网络管理*/
	if(Fun_Gprs_Tcp_Sta(TERM_SERVER) == FUN_GPRS_CONNECTED)
	{
		if(sendedLoginCnt >= 3 && (osKernelGetTickCount() - sendtimeStamp) > 3000)
		{
			RedialFunc();
			return;
		}
		else if(bLogined == 0)
		{
			if(osKernelGetTickCount() - sendtimeStamp > 20000)
			{
				sendtimeStamp = osKernelGetTickCount();
				dataLen = PackTermData(TERM_SERVER,szSendBuf, CMD_LOGIN);
				if(dataLen > 0)
				{
					Fun_Gprs_Tcp_send(TERM_SERVER,szSendBuf,dataLen);
				}
				sendedLoginCnt++;
			}
		}
		else if(osKernelGetTickCount() - sendtimeStamp > 300000 || ((osKernelGetTickCount() - sendtimeStamp) > 20000 && sendedLoginCnt > 0))
		{
			wakeTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
			sendtimeStamp = osKernelGetTickCount();
			dataLen = PackTermData(TERM_SERVER,szSendBuf,CMD_HEARTBEAT);
			if(dataLen > 0)
			{
				Fun_Gprs_Tcp_send(TERM_SERVER,szSendBuf,dataLen);
			}
			sendedLoginCnt++;
		}
	}
	else//没有连接上服务器
	{
		Fun_Gprs_Tcp_connect(TERM_SERVER,"mobilebeta.fsxytep.com",8086);
		Fun_Gprs_Tcp_Set_RecvCallBack(TERM_SERVER,UnpackTermData);
		ResetSeverPars();
	}
	/*串口管理*/
	if(getBufLength(RS1_COM))
	{
		dataLen = comGetBufDelay(RS1_COM, sizeof(szRecvBuf), szRecvBuf,10);
		if(dataLen > 0)
		{
			UnpackTermData(0xFF,szRecvBuf, dataLen);
		}
	}
	//USB权限控制
	if(osKernelGetTickCount() - unlockTime > MAX_LOCK_TIME_CNT)
	{
		unlockFlag = 1;//1:上位机EMMC锁打开  0:上位机EMMC锁关闭
	}
	if(osKernelGetTickCount() > 15000 && unlockFlag == 1 && BSP_Iostatus(IN_USB_ID) == ON && usbState == 0 && fmedia("M0:") == fsOK )
	{
			usbState = 1;
			USBD_Initialize(0U);
			USBD_Connect(0U);
			osDelay(500);
	}
	else if(usbState == 1)
	{
		USBD_STATE sta = USBD_GetState(0);
		if(sta.active == 0)
		{
			osDelay(200);
			BoardReset();
		}
	}
//	bsp_wakelock_para(wakelock,wakeTime,120,wakeCallback,TERM_SERVER,1);	
}

static void RedialFunc(void)
{
	ResetSeverPars();
	Fun_Gprs_Tcp_disconnect(TERM_SERVER);
}

//添加数据报头及校验
static uint16_t MakeCmd(uint8_t cmd,uint8_t replyFlag,uint8_t *wbuf,uint16_t dataLen)
{
	wbuf[0] = 0x23;
	wbuf[1] = 0x23;
	wbuf[2] = cmd;
	wbuf[3] = replyFlag;
	memset(&wbuf[4],0,17);
	Fun_Gprs_getIMEI(imei,15);
	memcpy(&wbuf[4],imei,15);
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;                   						 //dataLen_H
	wbuf[23] = dataLen;																		 //dataLen_L
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

static uint16_t PackTermData(uint8_t link,uint8_t *sbuf,uint8_t cmd)
{
	uint16_t dataLen = 0;
	switch(cmd)
	{
		case CMD_LOGIN://登入
		{
			uint16_t index = 24;
			sbuf[index++] = (uint8_t)(g_system_dt.year - 2000);      //年
			sbuf[index++] = (uint8_t)(g_system_dt.month);
			sbuf[index++] = (uint8_t)(g_system_dt.day);
			sbuf[index++] = (uint8_t)(g_system_dt.hour);
			sbuf[index++] = (uint8_t)(g_system_dt.minute);
			sbuf[index++] = (uint8_t)(g_system_dt.second);
			//终端类型
			strcpy((char*)&sbuf[index],"FNE200");
			index += strlen("FNE200") + 1;
			//配置文件版本		
			strcpy((char*)&sbuf[index],"v2.01");
			index += strlen("v2.01") + 1;	
			//终端编号
			if(strlen(gFrimPara.terminalId) == 0)
			{
				Fun_Gprs_getIMEI(imei,15);
				strncpy(gFrimPara.terminalId,&imei[3],12);
				//Device_ID_Save();
			}
			strcpy((char*)&sbuf[index],gFrimPara.terminalId);
			index += strlen(gFrimPara.terminalId) + 1;
			sbuf[index++] = 1;
			sbuf[index++] = PARA_VIN;
			strcpy((char*)&sbuf[index],gSysPara.vinCode);
			index += strlen(gSysPara.vinCode) + 1;			
			dataLen =  MakeCmd(CMD_LOGIN,0xFE,sbuf,index - 24) ;
		}
		break;
		case CMD_HEARTBEAT://心跳
		{
			uint16_t index = 24;
			sbuf[index++] = 0;
			dataLen = MakeCmd(CMD_HEARTBEAT,0xFE,sbuf,index-24);			
		}
		break;
	}
	return dataLen;
}

static void UnpackTermData(uint8_t link,uint8_t *rbuf, uint16_t rLen)
{
	uint8_t platCheckFlag = 0;
	if(rLen < 25 || (rLen > 2 && (rbuf[0] != rbuf[1] || rbuf[0] != 0x23)))
  {
		return;
	}
	if(link!= 0xFF && memcmp(imei,&rbuf[4],15)== 0)
	{
		platCheckFlag = 1;
	}
	if(getBccCode(rbuf, 2, rLen - 3) == rbuf[rLen - 1])
	{
		switch(rbuf[2])
		{
			case CMD_LOGIN:  //响应车辆登入
			{
				if(platCheckFlag == 0x01 && rbuf[3] == 0x01)
				{
					sendedLoginCnt = 0;
					//立即上报心跳
					sendtimeStamp = 0;
					bLogined = 1;
				}
			}
			break;
			case CMD_HEARTBEAT:  //响应心跳
			{
				if(platCheckFlag && bLogined==1)
				{
					sendedLoginCnt = 0;
				}
			}
			break;
			default:
			{
				static uint32_t key,seed,recvKey;
				if(platCheckFlag == 0x01 && bLogined == 1)
				{
					sendedLoginCnt = 0;
					PacketAnalyze(rbuf, rLen,sendNetdata);
				}
				else if(link == 0xFF)
				{
					recvKey = rbuf[4] << 0 | rbuf[5] << 8 | rbuf[6] << 16 | rbuf[7] << 24;
					//上位机安全校验
					if(key != 0 && recvKey == key)
					{
						unlockFlag = 1;
						unlockTime = osKernelGetTickCount();
						PacketAnalyze(rbuf, rLen,sendUartData);
					}
					//请求校验
					else
					{
						unlockFlag = 0;
						uint16_t index = 24,dataLen;
						szSendBuf[index++] = (uint8_t)(g_system_dt.year - 2000);      //年
						szSendBuf[index++] = (uint8_t)(g_system_dt.month);
						szSendBuf[index++] = (uint8_t)(g_system_dt.day);
						szSendBuf[index++] = (uint8_t)(g_system_dt.hour);
						szSendBuf[index++] = (uint8_t)(g_system_dt.minute);
						szSendBuf[index++] = (uint8_t)(g_system_dt.second);
						szSendBuf[index++] = 0x01;
						//生成种子
						RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
						RNG_Cmd(ENABLE);
						while(!RNG_GetFlagStatus(RNG_FLAG_DRDY));
						seed = RNG_GetRandomNumber();
						//生成KEY
						key = SeedToKeyLevel1(seed);
						//填充种子
						szSendBuf[index++] = seed >> 0;
						szSendBuf[index++] = seed >> 8;
						szSendBuf[index++] = seed >> 16;
						szSendBuf[index++] = seed >> 24;
						dataLen = MakeCmd(CMD_SCY_CHK,0xFE,szSendBuf,index-24);
						if(dataLen > 0)
						{
							sendUartData(szSendBuf,dataLen);
						}
					}
				}
			}
			break;
		}
	}
	return;
}

/* APP数据包解析 */
static void PacketAnalyze(const uint8_t* rBuf, uint16_t rLen,void (*SendCallBack)(uint8_t * buff,uint16_t len))
{
	uint16_t index = 0,i,j,paraCnt,packIdx,sendLen,packParaCnt;
	uint8_t buffTemp[50];
	const uint8_t *pData = &rBuf[24];
	memset(szSendBuf,0,sizeof(szSendBuf));
	packIdx = 24;
	//统一填充时间
	szSendBuf[packIdx++] = (uint8_t)(g_system_dt.year - 2000);      //年
	szSendBuf[packIdx++] = (uint8_t)(g_system_dt.month);
	szSendBuf[packIdx++] = (uint8_t)(g_system_dt.day);
	szSendBuf[packIdx++] = (uint8_t)(g_system_dt.hour);
	szSendBuf[packIdx++] = (uint8_t)(g_system_dt.minute);
	szSendBuf[packIdx++] = (uint8_t)(g_system_dt.second);
	switch(rBuf[2])
	{
		case CMD_QUERYPARAS:  //查询查询
		{
			static uint8_t testCnt = 0;
			szSendBuf[packIdx++] = 0;//预留参数个数
			//解析跳过时间
			index += 6;
			paraCnt = pData[index++];
			packParaCnt = 0;
			for(i = 0;i< paraCnt;i++)
			{
				packParaCnt++;
				//应答终端ID
				szSendBuf[packIdx++] = pData[index];
				switch(pData[index++])
				{
					case PARA_DEV_VER:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.hardWareVer,strlen(gSysPara.hardWareVer));
						packIdx += strlen(gSysPara.hardWareVer);
						szSendBuf[packIdx++] = 0;
						
						isCheckUSBModel = 0;					//查询硬件版本时放开Type-c 权限
					}
					break;
					case PARA_ICCID:
					{
						Fun_Gprs_getICCID((char*)&szSendBuf[packIdx],20);
						packIdx += strlen((char*)&szSendBuf[packIdx]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_IMEI:
					{
						memset(buffTemp,0,sizeof(buffTemp));
						Fun_Gprs_getIMEI((char*)buffTemp,sizeof(buffTemp));
						memcpy(&szSendBuf[packIdx],buffTemp,strlen((char*)buffTemp));
						packIdx += strlen((char*)buffTemp);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_IMSI:
					{
						memset(buffTemp,0,sizeof(buffTemp));
						Fun_Gprs_getIMSI((char*)buffTemp,sizeof(buffTemp));
						memcpy(&szSendBuf[packIdx],buffTemp,strlen((char*)buffTemp));
						packIdx += strlen((char*)buffTemp);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_PUBKEY:
					{
						uint32_t seLen = 64;
						if(tms_sm2_export_pubkey(0x0001,gFrimPara.chipKey,&seLen) == SUCCEED)
						{
							for(seLen = 0;seLen < 64;seLen++)
							{
								sprintf((char*)&szSendBuf[packIdx],"%02X",gFrimPara.chipKey[seLen]);
								packIdx += 2;			
							}
						}
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_DEVID:
					{
						memcpy(&szSendBuf[packIdx],gFrimPara.terminalId,strlen((char*)gFrimPara.terminalId));
						packIdx += strlen((char*)gFrimPara.terminalId);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_SIGNID:
					{
						memcpy(&szSendBuf[packIdx],gFrimPara.scyId,strlen((char*)gFrimPara.scyId));
						packIdx += strlen((char*)gFrimPara.scyId);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_VIN:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.vinCode,strlen(gSysPara.vinCode));
						packIdx += strlen(gSysPara.vinCode);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_QB_ID:
					{
						memcpy(&szSendBuf[packIdx],gFrimPara.qbId,strlen(gFrimPara.qbId));
						packIdx += strlen(gFrimPara.qbId) ;
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_QB_URL:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.domain[0],strlen(gSysPara.domain[0]));
						packIdx += strlen(gSysPara.domain[0]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_QB_PORT:
					{
						szSendBuf[packIdx++] = gSysPara.port[0] >> 8;
						szSendBuf[packIdx++] = gSysPara.port[0];						
					}
					break;
					case PARA_EV2_URL:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.domain[1],strlen(gSysPara.domain[1]));
						packIdx += strlen(gSysPara.domain[1]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_EV2_PORT:
					{
						szSendBuf[packIdx++] = gSysPara.port[1] >> 8;
						szSendBuf[packIdx++] = gSysPara.port[1];						
					}
					break;
					case PARA_EV3_URL:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.domain[2],strlen(gSysPara.domain[2]));
						packIdx += strlen(gSysPara.domain[2]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_EV3_PORT:
					{
						szSendBuf[packIdx++] = gSysPara.port[2] >> 8;
						szSendBuf[packIdx++] = gSysPara.port[2];						
					}
					break;
					case PARA_FVHJ_URL:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.domain[3],strlen(gSysPara.domain[3]));
						packIdx += strlen(gSysPara.domain[3]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_FVHJ_PORT:
					{
						szSendBuf[packIdx++] = gSysPara.port[3] >> 8;
						szSendBuf[packIdx++] = gSysPara.port[3];						
					}
					break;
					case PARA_FV17691_URL:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.domain[4],strlen(gSysPara.domain[4]));
						packIdx += strlen(gSysPara.domain[4]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_FV17691_PORT:
					{
						szSendBuf[packIdx++] = gSysPara.port[4] >> 8;
						szSendBuf[packIdx++] = gSysPara.port[4];						
					}
					break;
					case PARA_INTERVAL_REALSEND:			//实时数据上报周期
					{
						szSendBuf[packIdx++] = gSysPara.realDataInterval;	
					}
					break;
					case PARA_INTERVAL_HEARDSEND:			//心跳周期
					{
						szSendBuf[packIdx++] = gSysPara.heartInterval;	
					}
					break;
					case PARA_QB_LINK_SW:
					{
						szSendBuf[packIdx++] = ((gSysPara.linkSwitch & 1 << 0) > 0);
					}
					break;
					case PARA_EV2LINK_SW:
					{
						szSendBuf[packIdx++] = ((gSysPara.linkSwitch & 1 << 1) > 0);
					}
					break;
					case PARA_EV3LINK_SW:
					{
						szSendBuf[packIdx++] = ((gSysPara.linkSwitch & 1 << 2) > 0);
					}
					break;
					case PARA_FVHJLINK_SW:
					{
						szSendBuf[packIdx++] = ((gSysPara.linkSwitch & 1 << 3) > 0);
					}
					break;
					case PARA_FV17691LINK_SW:
					{
						szSendBuf[packIdx++] = ((gSysPara.linkSwitch & 1 << 4) > 0);
					}
					break;
					case PARA_SIGN_SW:
					{
						szSendBuf[packIdx++] = ((gSysPara.linkSwitch & 1 << 5) > 0);
					}
					break;
//					case PARA_PLAT_KEY:
//					{
//						uint32_t seLen = 64;
//						if(tms_sm2_export_pubkey(0x0002,gFrimPara.realKey,&seLen) == SUCCEED)
//						{
//							for(seLen = 0;seLen < 64;seLen++)
//							{
//								sprintf((char*)&szSendBuf[packIdx],"%02X",gFrimPara.realKey[seLen]);
//								packIdx += 2;			
//							}
//						}
//						szSendBuf[packIdx++] = 0;
//					}
//					break;
//					case PARA_VECC_KEY:
//					{
//						uint32_t seLen = 64;
//						if(tms_sm2_export_pubkey(0x0003,gFrimPara.setUpKey,&seLen) == SUCCEED)
//						{
//							for(seLen = 0;seLen < 64;seLen++)
//							{
//								sprintf((char*)&szSendBuf[packIdx],"%02X",gFrimPara.setUpKey[seLen]);
//								packIdx += 2;			
//							}
//						}
//						szSendBuf[packIdx++] = 0;
//					}
//					break;
					case PARA_SOFT_VER:
					{
						memcpy(&szSendBuf[packIdx],SW_VERSION,strlen(SW_VERSION));
						packIdx += strlen(SW_VERSION);
						szSendBuf[packIdx++] = 0;						
					}
					break;
//					case PARA_WAKE_TIME:
//					{
//						szSendBuf[packIdx++] = 0;
//					}
//					break;
//					case PARA_WAKE_INTERVAL:
//					{
//						szSendBuf[packIdx++] = 0;
//					}
//					break;
					case PARA_REAL_ICCID:
					{
						memset(buffTemp,0,sizeof(buffTemp));
						Fun_Gprs_getICCID((char*)buffTemp,sizeof(buffTemp));
						memcpy(&szSendBuf[packIdx],buffTemp,strlen((char*)buffTemp));
						packIdx += strlen((char*)buffTemp);
						szSendBuf[packIdx++] = 0;
					}
					break;	
					case PARA_CSQ:
					{
						szSendBuf[packIdx++] = Fun_Gprs_Csq();
					}
					break;
					case PARA_GPS_VIEW_START:
					{
						szSendBuf[packIdx++] = g_tGPS.ViewNumber;
					}
					break;
					case PARA_GPS_USE_START:
					{
						szSendBuf[packIdx++] = g_tGPS.UseNumber;
					}
					break;
					case PARA_EMMC_SIZE:
					{
						uint64_t sdCapacity = bsp_storage_capacity();
						szSendBuf[packIdx++] = sdCapacity >> 56;
						szSendBuf[packIdx++] = sdCapacity >> 48;
						szSendBuf[packIdx++] = sdCapacity >> 40;
						szSendBuf[packIdx++] = sdCapacity >> 32;
						szSendBuf[packIdx++] = sdCapacity >> 24;
						szSendBuf[packIdx++] = sdCapacity >> 16;
						szSendBuf[packIdx++] = sdCapacity >> 8;
						szSendBuf[packIdx++] = sdCapacity >> 0;
					}
					break;
					case PARA_CAN1_CNT:
					{
						uint32_t CanGetCnt = fun_can_Get_recvCnt(1);
						CAN_msg msg_buf = {0x1FFFFFF0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
						szSendBuf[packIdx++] = CanGetCnt >> 24;
						szSendBuf[packIdx++] = CanGetCnt >> 16;
						szSendBuf[packIdx++] = CanGetCnt >> 8;
						szSendBuf[packIdx++] = CanGetCnt >> 0;
						if(gSysPara.can1_used == 1 && SendCallBack != sendNetdata)
							CAN_send(1,&msg_buf,100);
					}
					break;
					case PARA_CAN2_CNT:
					{
						uint32_t CanGetCnt = fun_can_Get_recvCnt(2);
						CAN_msg msg_buf = {0x1FFFFFF1,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
						szSendBuf[packIdx++] = CanGetCnt >> 24;
						szSendBuf[packIdx++] = CanGetCnt >> 16;
						szSendBuf[packIdx++] = CanGetCnt >> 8;
						szSendBuf[packIdx++] = CanGetCnt >> 0;
						if(gSysPara.can2_used == 1 && SendCallBack != sendNetdata)
							CAN_send(2,&msg_buf,100);
					}
					break;
					case PARA_CAN3_CNT:
					{
						uint32_t CanGetCnt = fun_can_Get_recvCnt(3);
						CAN_msg msg_buf = {0x1FFFFFF2,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
						szSendBuf[packIdx++] = CanGetCnt >> 24;
						szSendBuf[packIdx++] = CanGetCnt >> 16;
						szSendBuf[packIdx++] = CanGetCnt >> 8;
						szSendBuf[packIdx++] = CanGetCnt >> 0;
						if(gSysPara.can3_used == 1 && SendCallBack != sendNetdata)
							CAN_send(3,&msg_buf,100);
					}
					break;
					case PARA_DEV_TIME:
					{
						static uint8_t second = 0;
						if(g_system_dt.second != second)
						{
							second = g_system_dt.second;
							sprintf((char*)&szSendBuf[packIdx],"%04d-%02d-%02d %02d:%02d:%02d",g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
							packIdx += 19;
						}
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_PWR_VOLT:
					{
						uint8_t* pFval = (uint8_t*)(&gTerminalState.pwrVolt);
						szSendBuf[packIdx++] = pFval[0];
						szSendBuf[packIdx++] = pFval[1];
						szSendBuf[packIdx++] = pFval[2];
						szSendBuf[packIdx++] = pFval[3];
					}
					break;
					case PARA_BAT_VOLT:
					{
						uint8_t* pFval = (uint8_t*)(&gTerminalState.batVolt);
						szSendBuf[packIdx++] = pFval[0];
						szSendBuf[packIdx++] = pFval[1];
						szSendBuf[packIdx++] = pFval[2];
						szSendBuf[packIdx++] = pFval[3];
					}
					break;
					case PARA_4G_VER:
					{						
						memset(buffTemp,0,sizeof(buffTemp));
						Fun_Gprs_getVer((char*)buffTemp,sizeof(buffTemp));
						memcpy(&szSendBuf[packIdx],buffTemp,strlen((char*)buffTemp));
						packIdx += strlen((char*)buffTemp);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_NET_STA:
					{
						szSendBuf[packIdx++] = (Fun_Gprs_GetSta() == FUN_GPRS_MON ? 1 : 0);
					}
					break;
					case PARA_USB_STA:
					{
						uint8_t emmcSta = 0;
						USBD_STATE sta = {0};
						//USB已连接，直接上报状态
						if(usbState == 1)
						{
							sta = USBD_GetState(0);
							emmcSta = sta.active;
						}
						//EMMC测试次数不足，等待EMMC测试完成
						else if(testCnt < 20 && fmedia("M0:") == fsOK)
						{
							emmcSta = 2;
						}
						//EMMC测试正常，提示插入USB
						else if(fmedia("M0:") == fsOK)
						{
							emmcSta = 3;
						}
						//提示EMMC异常
						else
						{
							emmcSta = 4;							
						}
						szSendBuf[packIdx++] = emmcSta;
						isCheckUSBModel = 1;
					}
					break;
					case PARA_EMMC_STA://EMMC读写
					{
						uint8_t sta = 0;
						if(testCnt < 20 && usbState == 1)
						{
							//重启终端测试
							unlockFlag = 0x02;
						}
						//USB未连接,测试不足
						else if(usbState == 0 && testCnt < 20)
						{
							unlockFlag = 0x00;
							FILE* fil = fopen("M0:test.tmp","ab");
							if(fil != NULL)
							{
								uint8_t writeCnt;
								fsFileInfo info;
								//循环写
								for(writeCnt = 0;writeCnt < 50;writeCnt++)
								{
									sta = 2;
									if(fwrite(szRecvBuf,1,1024,fil) != 1024)
									{
										sta = 0;
										break;
									}
									fflush(fil);
									osDelay(5);
								}
								fclose(fil);
								memset(&info,0,sizeof(info));
								if(ffind("M0:test.tmp",&info) == 0)
								{
									fdelete("M0:test.tmp",NULL);
								}
								else
								{
									sta = 0;
								}
								if(sta == 2)
								{
									testCnt++;
								}
							}
						}
						//USB已连接,或测试已经完成
						else
						{
							if(fmedia("M0:") == fsOK)
							{
								sta = 1;
							}
						}
						szSendBuf[packIdx++] = sta;
					}
					break;
					case PARA_FLASH_STA://FLASH识别
					{
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_CAR_STA://启动状态
					{
						szSendBuf[packIdx++] = gRealData.carState;
					}
					break;
					case PARA_CHG_STA://充电状态
					{
						szSendBuf[packIdx++] = gRealData.chargeState;
					}
					break;
					case PARA_ACC_STA://ACC状态
					{
						szSendBuf[packIdx++] = (BSP_Iostatus(IN_ACC_ID) == ON);
					}
					break;
					case PARA_QB_LINK_STA://平台连接状态
					{
						szSendBuf[packIdx++] = gb32960GetSta(0);
					}
					break;
					case PARA_EV2_LINK_STA://平台连接状态
					{
						szSendBuf[packIdx++] = gb32960GetSta(1);
					}
					break;
					case PARA_EV3_LINK_STA://平台连接状态
					{
						szSendBuf[packIdx++] = gb32960GetSta(2);
					}
					break;
					case PARA_FV1_LINK_STA://平台连接状态
					{
						szSendBuf[packIdx++] = gb17691GetSta(0);
					}
					break;
					case PARA_FV2_LINK_STA://平台连接状态
					{
						szSendBuf[packIdx++] = gb17691GetSta(1);
					}
					break;
					case PARA_GPS_ANT_STA://天线状态
					{
						szSendBuf[packIdx++] = g_tGPS.antSta;
					}
					break;
					case PARA_CAR_TYPE://车型
					{
						memcpy(&szSendBuf[packIdx],CAR_TYPE,strlen(CAR_TYPE));
						packIdx += strlen(CAR_TYPE);
						szSendBuf[packIdx++] = 0;
					}
					break;
					default:
						{
							uint8_t extParaLen = 0;
							extParaLen = extReadPara(pData[index-1],&szSendBuf[packIdx]);					
							if(extParaLen > 0)
								packIdx += extParaLen;
							else
								packIdx = (packIdx > 0) ? (packIdx - 1): 0;
						}
						break;
				}
				if(packIdx == 0)
					break;
			}
			szSendBuf[30] = packParaCnt;
			if(packIdx >= 24)
			{
				sendLen = MakeCmd(CMD_QUERYPARAS,0x01,szSendBuf,packIdx - 24);
				if(sendLen > 0)
				{
					SendCallBack(szSendBuf,sendLen);
				}
			}
			if(unlockFlag == 2 && isCheckUSBModel == 0)
			{
				osDelay(10);
				BoardReset();
			}
			if(sendUartData == SendCallBack && BSP_Iostatus(IN_USB_ID) == ON && isCheckUSBModel == 0)
			{
				RTC_WriteBackupRegister(RTC_BKP_DR1, 0xA5A5);
				osDelay(10);
				BoardReset();
			}
		}
		break;
		case CMD_SETPARAS://参数设置
		{
			static uint8_t verId = 0;
			for(j=0;j<2;j++)//第一次校验数据，第二次设置
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
						case PARA_DEV_VER:
						{
							if(j == 1)
							{
								verId = 1;
								memset(gSysPara.hardWareVer,0,sizeof(gSysPara.hardWareVer));
								strcpy((char*)gSysPara.hardWareVer,(char*)&pData[index]);
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_ICCID:
						{
							if(j == 1)
							{
								memset(gSysPara.iccid,0,sizeof(gSysPara.iccid));
								strcpy((char*)gSysPara.iccid,(char*)&pData[index]);
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_DEVID:
						{
							if(j == 1)
							{
								memset(gFrimPara.terminalId,0,sizeof(gFrimPara.terminalId));
								strcpy(gFrimPara.terminalId,(char*)&pData[index]);
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_SIGNID:
						{
							if(j == 1)
							{
								memset(gFrimPara.scyId,0,sizeof(gFrimPara.scyId));
								strcpy(gFrimPara.scyId,(char*)&pData[index]);
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_VIN:
						{
							if(j == 1)
							{
								memset(gSysPara.vinCode,0,sizeof(gSysPara.vinCode));
								strcpy(gSysPara.vinCode,(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_QB_ID:
						{
							if(j == 1)
							{
								gFrimPara.qbActicFlag = 0;
								memset(gFrimPara.qbId,0,sizeof(gFrimPara.qbId));
								strcpy(gFrimPara.qbId,(char*)&pData[index]);
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_QB_URL:
						{
							if(j == 1)
							{
								memset(gSysPara.domain[0],0,sizeof(gSysPara.domain[0]));
								strcpy(gSysPara.domain[0],(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_QB_PORT:
						{
							if(j == 1)
								gSysPara.port[0] = (pData[index] << 8) | pData[index + 1];
							index += 2;
						}
						break;
						case PARA_EV2_URL:
						{
							if(j == 1)
							{
								memset(gSysPara.domain[1],0,sizeof(gSysPara.domain[1]));
								strcpy(gSysPara.domain[1],(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_EV2_PORT:
						{
							if(j == 1)
								gSysPara.port[1] = (pData[index] << 8) | pData[index + 1];
							index += 2;					
						}
						break;
						case PARA_EV3_URL:
						{
							if(j == 1)
							{
								memset(gSysPara.domain[2],0,sizeof(gSysPara.domain[2]));
								strcpy(gSysPara.domain[2],(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_EV3_PORT:
						{
							if(j == 1)
								gSysPara.port[2] = (pData[index] << 8) | pData[index + 1];
							index += 2;			
						}
						break;
						case PARA_FVHJ_URL:
						{
							if(j == 1)
							{
								memset(gSysPara.domain[3],0,sizeof(gSysPara.domain[3]));
								strcpy(gSysPara.domain[3],(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_FVHJ_PORT:
						{
							if(j == 1)
								gSysPara.port[3] = (pData[index] << 8) | pData[index + 1];
							index += 2;			
						}
						break;
						case PARA_FV17691_URL:
						{
							if(j == 1)
							{
								memset(gSysPara.domain[4],0,sizeof(gSysPara.domain[4]));
								strcpy(gSysPara.domain[4],(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_FV17691_PORT:
						{
							if(j == 1)
								gSysPara.port[4] = (pData[index] << 8) | pData[index + 1];
							index += 2;					
						}
						break;
						case PARA_INTERVAL_REALSEND:			//实时数据上报周期
						{
							if(j == 1)
								gSysPara.realDataInterval = pData[index];
							index ++;					
						}
						break;
						case PARA_INTERVAL_HEARDSEND:			//心跳周期
						{
							if(j == 1)
								gSysPara.heartInterval = pData[index];
							index ++;					
						}
						break;
						case PARA_QB_LINK_SW:
						{
							if(j == 1)
							{
								gSysPara.linkSwitch &= ~(1 << 0);
								gSysPara.linkSwitch |= ((pData[index] & 0x01) << 0);
							}
							index++;
						}
						break;
						case PARA_EV2LINK_SW:
						{
							if(j == 1)
							{
								gSysPara.linkSwitch &= ~(1 << 1);
								gSysPara.linkSwitch |= ((pData[index] & 0x01) << 1);
							}
							index++;
						}
						break;
						case PARA_EV3LINK_SW:
						{
							if(j == 1)
							{
								gSysPara.linkSwitch &= ~(1 << 2);
								gSysPara.linkSwitch |= ((pData[index] & 0x01) << 2);
							}
							index++;
						}
						break;
						case PARA_FVHJLINK_SW:
						{
							if(j == 1)
							{
								gSysPara.linkSwitch &= ~(1 << 3);
								gSysPara.linkSwitch |= ((pData[index] & 0x01) << 3);
							}
							index++;
						}
						break;
						case PARA_FV17691LINK_SW:
						{
							if(j == 1)
							{
								gSysPara.linkSwitch &= ~(1 << 4);
								gSysPara.linkSwitch |= ((pData[index] & 0x01) << 4);
							}
							index++;
						}
						break;
						case PARA_SIGN_SW:
						{
							if(j == 1)
							{
								gSysPara.linkSwitch &= ~(1 << 5);
								gSysPara.linkSwitch |= ((pData[index] & 0x01) << 5);
							}
							index++;
						}
						break;
					case PARA_PLAT_KEY:
					{
						uint32_t sm2Len = strlen((char*)&pData[index]);
						if(j == 1 && sm2Len == 128)
						{
							for(sm2Len = 0;sm2Len < 64;sm2Len++)
							{
								sscanf((char*)&pData[sm2Len * 2 + index],"%2hhX",&gFrimPara.realKey[sm2Len]);
							}
//							tms_sm2_import_pubkey(0x0002,gFrimPara.realKey,64);
						}
						index += strlen((char*)&pData[index]) + 1;
					}
					break;		
					case PARA_VECC_KEY:
					{
						uint32_t sm2Len = strlen((char*)&pData[index]);
						if(j == 1 && sm2Len == 128)
						{
							uint32_t sm2Len = 64;
							for(sm2Len = 0;sm2Len < 64;sm2Len++)
							{
								sscanf((char*)&pData[sm2Len * 2 + index],"%2hhX",&gFrimPara.setUpKey[sm2Len]);
							}
//							tms_sm2_import_pubkey(0x0003,gFrimPara.setUpKey,64);
						}
						index += strlen((char*)&pData[index]) + 1;
					}
					break;
//					case PARA_WAKE_TIME://设置唤醒时间
//					{
//						
//						index += strlen((char*)&pData[index]) + 1;
//					}
//					break;
//					case PARA_WAKE_INTERVAL://设置唤醒间隔
//					{
//						index += strlen((char*)&pData[index]) + 1;
//					}
//					break;
					default:
						{
							uint8_t extParaLen = 0;
							extParaLen = extSetPara(j,pData[index-1],&pData[index]);					
							if(extParaLen > 0)
								index += extParaLen;
							else
									packParaCnt = 0;
						}
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
				if(verId == 1)
				{
					gFrimPara.factoryDate[0] = 0x20;
					gFrimPara.factoryDate[1] = ((pData[0] % 100 / 10) << 4) | (pData[0] % 10);
					gFrimPara.factoryDate[2] = ((pData[1] % 100 / 10) << 4) | (pData[1] % 10);
					gFrimPara.factoryDate[3] = ((pData[2] % 100 / 10) << 4) | (pData[2] % 10);
				}
//				gUpdateParaFlag = 0x1F;
				System_Pare_Save();
				Device_ID_Save();
			}
			sendLen = MakeCmd(CMD_SETPARAS,packParaCnt == paraCnt ? 0x01 : 0x02,szSendBuf,packIdx - 24);
			if(sendLen > 0)
				SendCallBack(szSendBuf,sendLen);
		}
		break;
		case CMD_CTRL://终端控制
		{
			uint8_t rspCode = 2;
			uint8_t isReboot = 0;
			//解析跳过时间
			index = 6;
			szSendBuf[packIdx++] = pData[index];
			switch(pData[index])
			{
				case 0x01: //远程升级
				{
					char *hostname;
					uint16_t hostport;
					char *username = NULL;
					char *password = NULL;
					char *filePach = NULL;
					char* fileName = NULL;
					char *pSplit = NULL;
					rspCode = 2;
					//用户名
					username = strstr((char*)&pData[index + 2],"FTP://");
					if(username == NULL)
					{
						username = strstr((char*)&pData[index + 2],"ftp://");
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
						//开始升级
						rspCode = 1;
						funMonSetFtpParas(hostname,hostport,username,password,filePach,fileName,NULL);
					}
				}
				break;
				case 0x03: //终端重启
				{
					rspCode = 1;
					isReboot = 1;
				}
				break;
				case 0x90: //登入测试
				{
					if(Fun_Gprs_GetSta() >= FUN_GPRS_MON && gRealData.carState == 1 && gb32960GetSta(0xFF) == 0)
					{
						rspCode = 1;
					}
					else
					{
						strcpy((char*)&szSendBuf[packIdx],"车辆启动、网络正常、先登出才能操作！");
						packIdx += 36;		
					}
				}
				break;
				case 0x91: //登出测试
				{
					if(Fun_Gprs_GetSta() >= FUN_GPRS_MON && gRealData.carState == 1 && gb32960GetSta(0xFF) == 1)
					{
						rspCode = 1;
					}
					else
					{
						strcpy((char*)&szSendBuf[packIdx],"车辆启动、网络正常、先登入才能操作！");
						packIdx += 36;		
					}
				}
				break;
				case 0x92: //预警测试
				{
					if(Fun_Gprs_GetSta() >= FUN_GPRS_MON && gRealData.carState == 1)
					{
						rspCode = 1;				
					}
					else
					{
						strcpy((char*)&szSendBuf[packIdx],"车辆启动、网络正常才能操作！");
						packIdx += 36;		
					}
				}
				break;
				case 0x93: //离线测试
				{
					if(Fun_Gprs_GetSta() >= FUN_GPRS_MON && gRealData.carState == 1)
					{
						rspCode = 1;
					}
					else
					{
						strcpy((char*)&szSendBuf[packIdx],"车辆启动、网络正常、先登入才能操作！");
						packIdx += 36;		
					}
				}
				break;
				case 0x94: //取消预警
				{
					rspCode = 1;
				}
				break;
				case 0x95: //取消断网
				{
					rspCode = 1;
				}
				break;
				case 0x96: //实时报文解析
				{
					rspCode = 1;
				}
				break;
				case 0x97: //取消实时报文解析
				{
					rspCode = 1;				
				}
				break;
				case 0x98: //开启USB权限
				{
					rspCode = 1;				
				}
				break;
				case 0x99: //格式化U盘
				{
					rspCode = 1;				
				}
				break;
				default:;break;
			}
			szSendBuf[packIdx++] = 0;
			sendLen = MakeCmd(CMD_CTRL,rspCode,szSendBuf,packIdx - 24);
			if(sendLen > 0)
				SendCallBack(szSendBuf,sendLen);
			osDelay(1000);
			if(rspCode == 1)
			{
				if(pData[index] == 0x90)
					gb32960Ctrl(GB32960_TEST_LOGIN);//登入
				else if(pData[index] == 0x91)
					gb32960Ctrl(GB32960_TEST_LOGOUT);//登出
				else if(pData[index] == 0x92)
					gb32960Ctrl(GB32960_TEST_ALARM);//预警测试
				else if(pData[index] == 0x93)
					gb32960Ctrl(GB32960_TEST_OFFLINE);//离线测试
				else if(pData[index] == 0x94)
					gb32960Ctrl(GB32960_TEST_UNALARM);//取消预警
				else if(pData[index] == 0x95)
					gb32960Ctrl(GB32960_TEST_ONLINE);//取消断网
				else if(pData[index] == 0x96)
					gb32960Ctrl(GB32960_TEST_PRINT);//报文打印
				else if(pData[index] == 0x97)
					gb32960Ctrl(GB32960_TEST_UNPRINT);//取消打印
				else if(pData[index] == 0x98)
				{
					unlockFlag = 1;										//开启USB权限
					unlockTime = osKernelGetTickCount();
					RTC_WriteBackupRegister(RTC_BKP_DR1, 0xA5A5);
					osDelay(10);
					BoardReset();
				}
				else if(pData[index] == 0x99)				//格式化U盘
				{
					bsp_formatU_state();
				}
			}
			if(isReboot == 1)
			{
				BoardReset();
			}
		}
		break;
		case CMD_SEND_FILE:
		{
			uint8_t download_finished = 0;
			const static uint16_t maxRecvSize = 1024;
			static uint16_t pageNumIdx = 0;
			static uint32_t fileSize = 0,fileRecvSize = 0;
			uint16_t pageNum,pageSize;
			uint8_t rspCode = 1;
			while(1)
			{
				uint8_t tryCnt = 0;
				//解析跳过时间
				index = 6;
				//文件类型
				szSendBuf[packIdx++] = pData[index++];
				//页面编号
				pageNum =  pData[index] << 8 |  pData[index + 1];
				szSendBuf[packIdx++] = pData[index + 0];
				szSendBuf[packIdx++] = pData[index + 1];
				index += 2;
				//页面大小
				pageSize =  pData[index] << 8 |  pData[index + 1];
				//首页，文件描述
				if(pageNum == 0 && pageSize == 4)
				{
					index += 2;
					fileSize =   (pData[index + 0] << 24 |  pData[index + 1] << 16 | pData[index + 2] << 8 |  pData[index + 3]);
					pageNumIdx = 1;
					fileRecvSize = 0;
					szSendBuf[packIdx++] = (uint8_t)((uint16_t)maxRecvSize >> 8);
					szSendBuf[packIdx++] = (uint8_t)((uint16_t)maxRecvSize >> 0);
				}
				else if(fileRecvSize < fileSize && pageNum > 0 && pageNum == pageNumIdx && pageSize <= maxRecvSize && fileSize <= 0x20000)
				{
					pageNumIdx = (pageNumIdx % 65535) + 1;
					szSendBuf[packIdx++] = pData[index + 0];
					szSendBuf[packIdx++] = pData[index + 1];
					index += 2;
					
					if(saveBinFile(fileRecvSize,maxRecvSize,fileSize,(uint8_t*)&pData[index]) == 0)
					{
						rspCode = 2;
					}
					fileRecvSize += pageSize;
					if(rspCode != 2 && fileRecvSize == fileSize)
					{
						download_finished = 1;
					}
				}
				else
				{
					rspCode = 2;
				}
				sendLen = MakeCmd(CMD_SEND_FILE,rspCode,szSendBuf,packIdx - 24);
				if(sendLen > 0)
					comSendBuf(RS1_COM,szSendBuf,sendLen);
				if(rspCode == 2 || download_finished == 1)
				{
					break;
				}
				while(1)
				{
					sendLen = comGetBufDelay(RS1_COM, sizeof(szRecvBuf), szRecvBuf,10);
					if(sendLen > 0)
					{
						if(sendLen < 25 || szRecvBuf[0] != 0x23 || szRecvBuf[1] != 0x23 || getBccCode(szRecvBuf, 2, sendLen - 3) != szRecvBuf[sendLen - 1])
						{
							rspCode = 2;
							break;
						}
						pData = &szRecvBuf[24];
						memset(szSendBuf,0,sizeof(szSendBuf));
						packIdx = 24;
						szSendBuf[packIdx++] = (uint8_t)(g_system_dt.year - 2000);      //年
						szSendBuf[packIdx++] = (uint8_t)(g_system_dt.month);
						szSendBuf[packIdx++] = (uint8_t)(g_system_dt.day);
						szSendBuf[packIdx++] = (uint8_t)(g_system_dt.hour);
						szSendBuf[packIdx++] = (uint8_t)(g_system_dt.minute);
						szSendBuf[packIdx++] = (uint8_t)(g_system_dt.second);	
						break;
					}
					else if(tryCnt++ > 100)
					{
						return;
					}
				}
				if(rspCode == 2)
				{
					break;
				}
				WDG_Feed(3);
			}
			if(download_finished == 1)
			{
				osDelay(5);
				BoardReset();
			}
		}
		break;
		
	}
}

__WEAK uint8_t extSetPara(uint8_t check,uint8_t setParaId,const uint8_t* setBuff)
{
	return 0;
}
__WEAK uint8_t extReadPara(uint8_t readParaId,uint8_t* readBuff)
{
	return 0;
}



