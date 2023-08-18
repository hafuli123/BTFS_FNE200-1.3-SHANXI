/*
文件：terminal_Server.c
功能：终端管理
日期：2021/7/2
公司：佛山新源
作者：csj
*/
#include "pdu.h"
#include "bsp_sys.h"
#include "newant_Bms.h"
#include "terminal_Server.h"
#include "protocol_GB32960.h"

#define LOG_TAG    "Test_TermServer"
#include "elog.h"
static void testLogger(void);

static uint8_t testss = 0,timeTmp = 0;
static void testLogger(void)
{
	if(g_system_dt.second != timeTmp)
	{
		timeTmp = g_system_dt.second;
				
		switch(testss)
		{
			case 4:	log_a("Log_AllMsg!");break;
//			case 2:	log_e("Log_Error_Printf!");break;
//			case 3:	log_w("Log_Warning_Printf !");break;
//			case 4:	log_i("Log_Info_Printf !");break;
			case 5:	log_d("Log_Debug_Printf !");break;
			case 6:	log_v("Log_Verbose_Printf !");break;
		}
		testss++;
		if(testss>=6)
			testss = 1;	
	}
}

#define MAX_SENDLEN 1024

enum COMMAND_CODE
{
	CMD_NONE = 0,
	CMD_LOGIN = 0x71,                 //终端注册
	CMD_HEARTBEAT = 0x07,     				//心跳
	CMD_QUERYPARAS = 0x80,     				//查询命令
	CMD_SETPARAS = 0x81,              //设置命令
	CMD_CTRL = 0x82,                  //控制命令
};

enum PARA_ID
{
	PARA_DEV_VER = 0x80,				//硬件版本
	PARA_ICCID,                 //ICCID
	PARA_IMEI,     				 			//IMEI
	PARA_IMSI,     				 			//IMSI
	PARA_PUBKEY,                //终端公钥
	PARA_DEVID,									//终端ID
	PARA_SIGNID,								//签名ID
	PARA_VIN,										//车架号
	
	PARA_YZTID,									//云智通ID
	PARA_YZT_URL,								//云智通地址
	PARA_YZT_PORT,							//云智通端口
	PARA_EV2_URL,								//EV第二链路地址
	PARA_EV2_PORT,							//EV第二链路端口
	PARA_EV3_URL,								//EV第三链路地址
	PARA_EV3_PORT,							//EV第三链路端口
	PARA_FVHJ_URL,							//FV环境部链路地址
	PARA_FVHJ_PORT,							//FV环境部链路端口
	PARA_FV17691_URL,						//FV17691链路地址
	PARA_FV17691_PORT,					//FV17691链路端口
	
	PARA_YZTLINK_SW,						//云智通链路开启
	PARA_EV2LINK_SW,						//EV第二链路开启
	PARA_EV3LINK_SW,						//EV第三链路开启
	PARA_FVHJLINK_SW,						//FV第一链路开启
	PARA_FV17691LINK_SW,				//FV第二链路开启
	
	PARA_SIGN_SW,								//终端签名开关
	PARA_PLAT_KEY,							//企业平台公钥
	PARA_VECC_KEY,							//国家环保平台公钥
	PARA_WAKE_TIME,							//设置唤醒时间
};



static uint32_t sendtimeStamp = 0;
static uint8_t sendedLoginCnt;
static uint8_t bLogined;
static uint8_t szSendBuf[MAX_SENDLEN];
static uint8_t szRecvBuf[MAX_SENDLEN];

static uint8_t testEmmcCnt = 0xFF;		//EMMC测试次数
static uint8_t testEmmcReboot = 0;		//EMMC测试重启

static void ResetSeverPars(void);
static void RedialFunc(void);
static uint8_t UnpackTermData(uint8_t link,const uint8_t *rbuf, uint16_t rLen);
static uint16_t PackTermData(uint8_t link,uint8_t *sbuf,uint8_t cmd);
typedef void (*SendCallBack)(uint8_t * buff,uint16_t len);
static void PacketAnalyze(const uint8_t* rBuf, uint16_t rLen,SendCallBack CallBack);

static void ResetSeverPars(void)
{
	bLogined = 0;
	sendedLoginCnt = 0;
}

void InitTermServer(void)
{
	ResetSeverPars();
}

void sendNetdata(uint8_t * buff,uint16_t len)
{
	simSendData(TERM_SERVER,buff,len);
}

void sendUartData(uint8_t * buff,uint16_t len)
{
	BSP_Uart_Write(RS1_COM,buff,len);
	whistle(100,1);
}

void termServerTask(void)
{
	/*网络管理*/
	if(simQueryLinkStatus(TERM_SERVER) == 1)//查询主链路连接情况
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
				PackTermData(TERM_SERVER,szSendBuf, CMD_LOGIN);
				sendedLoginCnt++;
			}
		}
		else if(osKernelGetTickCount() - sendtimeStamp > 300000 || ((osKernelGetTickCount() - sendtimeStamp) > 20000 && sendedLoginCnt > 0))
		{
			sendtimeStamp = osKernelGetTickCount();
			PackTermData(TERM_SERVER,szSendBuf,CMD_HEARTBEAT);
			sendedLoginCnt++;
		}
	}
	else//没有连接上服务器
	{
		simSetUnpackdataFun(TERM_SERVER,"mobilebeta.fsxytep.com",19004,UnpackTermData,1);
		ResetSeverPars();
	}
	/*串口管理*/
	if(getBufLength(RS1_COM))
	{
		uint16_t len = 0;
		len = BSP_Uart_Read(RS1_COM, sizeof(szRecvBuf), szRecvBuf,10);
		if(len > 0)
		{
			PacketAnalyze(szRecvBuf, len,sendUartData);
		}
	}
	if(gIsPendingUpdate == 0x55/* && can_State == 0*/)
	{
		gIsPendingUpdate = 0;
		osDelay(5000);
		ftpUpdateApp();
	}
	if(osKernelGetTickCount() > 5000 && !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) && gTerminalState.sdState == 1 && gTerminalState.sdSwSta == 0 && gTerminalState.usbState == 0 && testEmmcCnt > 20)
	{
		gTerminalState.usbState = 1;
		USBD_Initialize(0U);                  /* USB Device 0 Initialization        */;
		USBD_Connect(0U);
		osDelay(500);
	}
	if(gTerminalState.usbState == 1)
	{
		USBD_STATE sta = USBD_GetState(0);
		gTerminalState.usbState = 1;
		if(sta.active == 0 || testEmmcReboot == 1)
		{
			PowerOff();
			osDelay(200);
			BoardReset();
		}
	}
	
//	testLogger();
}

static void RedialFunc(void)
{
	ResetSeverPars();
	reConnect(TERM_SERVER);
}

//添加数据报头及校验
static uint16_t MakeCmd(uint8_t cmd,uint8_t replyFlag,uint8_t *wbuf,uint16_t dataLen)
{
	wbuf[0] = 0x23;
	wbuf[1] = 0x23;
	wbuf[2] = cmd;
	wbuf[3] = replyFlag;
	memset(&wbuf[4],0,17);
	memcpy(&wbuf[4],gTerminalState.imei,15); 
	wbuf[21] = 0x01;
	wbuf[22] = dataLen >> 8;                   						 //dataLen_H
	wbuf[23] = dataLen;																		 //dataLen_L
	wbuf[24 + dataLen] = getBccCode(wbuf,2,22 + dataLen); //BCC
	return 24 + dataLen + 1;
}

static uint16_t PackTermData(uint8_t link,uint8_t *sbuf,uint8_t cmd)
{
	uint16_t dataLen = 0;
	memset(sbuf,0,MAX_SENDLEN);
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
			strcpy((char*)&sbuf[index],"T1.00");
			index += strlen("T1.00") + 1;			
			//终端编号
			if(strlen(gFrimPara.terminalId) == 0)
				strncpy(gFrimPara.terminalId,(char*)gTerminalState.imei,12);
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
	if(dataLen > 0)
	{
		simSendData(link,sbuf,dataLen);
	}
	return dataLen;
}

static uint8_t UnpackTermData(uint8_t link,const uint8_t *rbuf, uint16_t rLen)
{
	if(rLen < 25 || (rLen > 2 && (rbuf[0] != rbuf[1] || rbuf[0] != 0x23)))
  {
		return 0;
	}
	
//	printfData((char*)rbuf,rLen,0);
	
	if(memcmp(gTerminalState.imei,&rbuf[4],15)== 0 && getBccCode(rbuf, 2, rLen - 3) == rbuf[rLen - 1])
	{
		switch(rbuf[2])
		{
			case CMD_LOGIN:  //响应车辆登入
			{
				if(rbuf[3] == 0x01)
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
				if(bLogined==1)
				{
					sendedLoginCnt = 0;
				}
			}
			break;
			default:
			{
				if(bLogined == 1)
				{
					sendedLoginCnt = 0;
					PacketAnalyze(rbuf, rLen,sendNetdata);
				}
			}
			break;
		}
	}
	return 0;
}

/* APP数据包解析 */
static void PacketAnalyze(const uint8_t* rBuf, uint16_t rLen,SendCallBack CallBack)
{
	uint16_t index = 0,i,j,paraCnt,packIdx,sendLen,packParaCnt;
	const uint8_t *pData = &rBuf[24];
	if(rLen < 25 )//数据长度错误
  {
		return;
	}
	else if(rBuf[0] != 0x23 || rBuf[1] != 0x23)//标志错误
	{
		return;
	}
	else if(getBccCode(rBuf, 2, rLen - 3) != rBuf[rLen - 1])//BCC校验
	{
		return;
	}
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
		case 0x80:  //查询查询
		{
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
					}
					break;
					case PARA_ICCID:
					{
						simGetCCID(&szSendBuf[packIdx],20);
						packIdx += strlen((char*)&szSendBuf[packIdx]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_IMEI:
					{
						memcpy(&szSendBuf[packIdx],gTerminalState.imei,strlen((char*)gTerminalState.imei));
						packIdx += strlen((char*)gTerminalState.imei);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_IMSI:
					{
						memcpy(&szSendBuf[packIdx],gTerminalState.imsi,strlen((char*)gTerminalState.imsi));
						packIdx += strlen((char*)gTerminalState.imsi);
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
					case PARA_YZTID:
					{
						memcpy(&szSendBuf[packIdx],gFrimPara.yztDevId,strlen(gFrimPara.yztDevId));
						packIdx += strlen(gFrimPara.yztDevId) ;
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_YZT_URL:
					{
						memcpy(&szSendBuf[packIdx],gSysPara.domain[0],strlen(gSysPara.domain[0]));
						packIdx += strlen(gSysPara.domain[0]);
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_YZT_PORT:
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
					case PARA_YZTLINK_SW:
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
					case PARA_PLAT_KEY:
					{
						uint32_t seLen = 64;
						if(tms_sm2_export_pubkey(0x0002,gFrimPara.realKey,&seLen) == SUCCEED)
						{
							for(seLen = 0;seLen < 64;seLen++)
							{
								sprintf((char*)&szSendBuf[packIdx],"%02X",gFrimPara.realKey[seLen]);
								packIdx += 2;			
							}
						}
						szSendBuf[packIdx++] = 0;
					}
					break;
					case PARA_VECC_KEY:
					{
						uint32_t seLen = 64;
						if(tms_sm2_export_pubkey(0x0003,gFrimPara.setUpKey,&seLen) == SUCCEED)
						{
							for(seLen = 0;seLen < 64;seLen++)
							{
								sprintf((char*)&szSendBuf[packIdx],"%02X",gFrimPara.setUpKey[seLen]);
								packIdx += 2;			
							}
						}
						szSendBuf[packIdx++] = 0;
					}
					break;		
					case 0xC0:
					{
						memcpy(&szSendBuf[packIdx],gTerminalState.iccid,strlen((char*)gTerminalState.iccid));
						packIdx += strlen((char*)gTerminalState.iccid);
						szSendBuf[packIdx++] = 0;
					}
					break;	
					case 0xC1:
					{
						gIsTest = 1;
						szSendBuf[packIdx++] = gTerminalState.csq;
					}
					break;
					case 0xC2:
					{
						szSendBuf[packIdx++] = gTerminalState.gpsViewNum;
					}
					break;
					case 0xC3:
					{
						szSendBuf[packIdx++] = gTerminalState.gpsUseNum;
					}
					break;
					case 0xC4:
					{
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 56;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 48;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 40;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 32;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 24;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 16;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 8;
						szSendBuf[packIdx++] = gTerminalState.sdCapacity >> 0;
					}
					break;
					case 0xC5:
					{
						CAN_msg msg_buf = {0x1FFFFFF0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
						szSendBuf[packIdx++] = gTerminalState.Can1GetCnt >> 24;
						szSendBuf[packIdx++] = gTerminalState.Can1GetCnt >> 16;
						szSendBuf[packIdx++] = gTerminalState.Can1GetCnt >> 8;
						szSendBuf[packIdx++] = gTerminalState.Can1GetCnt >> 0;
						if(gSysPara.can1_used == 1 && CallBack != sendNetdata)
							CAN_send(1,&msg_buf,100);
					}
					break;
					case 0xC6:
					{
						CAN_msg msg_buf = {0x1FFFFFF1,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
						szSendBuf[packIdx++] = gTerminalState.Can2GetCnt >> 24;
						szSendBuf[packIdx++] = gTerminalState.Can2GetCnt >> 16;
						szSendBuf[packIdx++] = gTerminalState.Can2GetCnt >> 8;
						szSendBuf[packIdx++] = gTerminalState.Can2GetCnt >> 0;
						if(gSysPara.can2_used == 1 && CallBack != sendNetdata)
							CAN_send(2,&msg_buf,100);
					}
					break;
					case 0xC7:
					{
						CAN_msg msg_buf = {0x1FFFFFF2,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
						szSendBuf[packIdx++] = gTerminalState.Can3GetCnt >> 24;
						szSendBuf[packIdx++] = gTerminalState.Can3GetCnt >> 16;
						szSendBuf[packIdx++] = gTerminalState.Can3GetCnt >> 8;
						szSendBuf[packIdx++] = gTerminalState.Can3GetCnt >> 0;
						if(gSysPara.can3_used == 1 && CallBack != sendNetdata)
							CAN_send(3,&msg_buf,100);
					}
					break;
					case 0xC8:
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
					case 0xC9:
					{
						uint8_t* pFval = (uint8_t*)(&gTerminalState.pwrVolt);
						szSendBuf[packIdx++] = pFval[0];
						szSendBuf[packIdx++] = pFval[1];
						szSendBuf[packIdx++] = pFval[2];
						szSendBuf[packIdx++] = pFval[3];
					}
					break;
					case 0xCA:
					{
						uint8_t* pFval = (uint8_t*)(&gTerminalState.batVolt);
						szSendBuf[packIdx++] = pFval[0];
						szSendBuf[packIdx++] = pFval[1];
						szSendBuf[packIdx++] = pFval[2];
						szSendBuf[packIdx++] = pFval[3];
					}
					break;
					case 0xCB:
					{
						memcpy(&szSendBuf[packIdx],gTerminalState.gprsVer,strlen(gTerminalState.gprsVer));
						packIdx += strlen(gTerminalState.gprsVer) ;
						szSendBuf[packIdx++] = 0;
					}
					break;
					case 0xCC:
					{
						szSendBuf[packIdx++] = (gprs_State > 0 ? 1 : 0);
					}
					break;
					case 0xCD:
					{
						uint8_t emmcSta = 0;
						USBD_STATE sta = {0};
						if(gTerminalState.usbState == 1)
						{
							sta = USBD_GetState(0);
							emmcSta = sta.active;
						}
						else if(testEmmcCnt < 20 && gTerminalState.sdSwSta == 0)
						{
							//等待EMMC测试
							emmcSta = 2;
						}
						else if(gTerminalState.sdSwSta == 0)
						{
							//提示插入EMMC
							emmcSta = 3;
						}
						else
						{
							//提示EMMC异常，请先检查EMMC
							emmcSta = 4;							
						}
						szSendBuf[packIdx++] = emmcSta;
					}
					break;
					case 0xCE://EMMC读写
					{
						fsFileInfo info;
						FILE* fil;
						uint8_t sta = 0;
						uint8_t writeCnt = 0;
						if(gTerminalState.sdState == 1 && gTerminalState.usbState == 0)
						{
							if(testEmmcCnt == 0xFF || testEmmcCnt == 0xFE)
								testEmmcCnt = 0;
							fil = fopen("M0:test.tmp","ab");
							if(fil != NULL)
							{
								for(writeCnt = 0;writeCnt < 50;writeCnt++)
								{
									if(fwrite(szRecvBuf,1,1024,fil) != 1024)
									{
										break;
									}
									fflush(fil);
									osDelay(5);
								}
								fclose(fil);
								memset(&info,0,sizeof(info));
								if(ffind("M0:test.tmp",&info) == 0)
								{
									sta = 1;
									fdelete("M0:test.tmp",NULL);
									testEmmcCnt++;
								}
							}
							if(sta != 1)
							{
								//写异常
								gTerminalState.sdSwSta = 1;
							}
						}
						if(gTerminalState.sdSwSta == 0 && gTerminalState.usbState == 1)
						{
							sta = 1;
						}
						else if(sta == 1)
						{
							sta = 2;
						}
						szSendBuf[packIdx++] = sta;
						if(testEmmcCnt == 0xFF)
						{
							testEmmcReboot = 1;
						}
					}
					break;
					case 0xCF://FLASH识别
					{
						szSendBuf[packIdx++] = 0;
					}
					break;
					case 0xD0://启动状态
					{
						szSendBuf[packIdx++] = gRealData.carState;
					}
					break;
					case 0xD1://充电状态
					{
						szSendBuf[packIdx++] = gRealData.chargeState;
					}
					break;
					case 0xD2://充电状态
					{
						szSendBuf[packIdx++] = gTerminalState.accState;
					}
					break;
					case 0xD3://平台连接状态
					{
						szSendBuf[packIdx++] = gTerminalState.platfSta[0];
					}
					break;
					case 0xD4://平台连接状态
					{
						szSendBuf[packIdx++] = gTerminalState.platfSta[1];
					}
					break;
					case 0xD5://平台连接状态
					{
						szSendBuf[packIdx++] = gTerminalState.platfSta[2];
					}
					break;
					case 0xD6://平台连接状态
					{
						szSendBuf[packIdx++] = gTerminalState.platfSta[3];
					}
					break;
					case 0xD7://平台连接状态
					{
						szSendBuf[packIdx++] = gTerminalState.platfSta[4];
					}
					break;
					case 0xD8://天线状态
					{
						szSendBuf[packIdx++] = gTerminalState.gpsState;
					}
					break;
					case 0xD9://车型
					{
						memcpy(&szSendBuf[packIdx],CAR_TYPE,strlen(CAR_TYPE));
						packIdx += strlen(CAR_TYPE);
						szSendBuf[packIdx++] = 0;
					}
					break;
					default:packIdx = (packIdx > 0) ? (packIdx - 1): 0;
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
					CallBack(szSendBuf,sendLen);
				}
			}
		}
		break;
		case 0x81://参数设置
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
						case PARA_YZTID:
						{
							if(j == 1)
							{
								gFrimPara.yztActicFlag = 0;
								memset(gFrimPara.yztDevId,0,sizeof(gFrimPara.yztDevId));
								strcpy(gFrimPara.yztDevId,(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_YZT_URL:
						{
							if(j == 1)
							{
								memset(gSysPara.domain[0],0,sizeof(gSysPara.domain[0]));
								strcpy(gSysPara.domain[0],(char*)&pData[index]);								
							}
							index += strlen((char*)&pData[index]) + 1;
						}
						break;
						case PARA_YZT_PORT:
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
						case PARA_YZTLINK_SW:
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
							tms_sm2_import_pubkey(0x0002,gFrimPara.realKey,64);
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
							tms_sm2_import_pubkey(0x0003,gFrimPara.setUpKey,64);
						}
						index += strlen((char*)&pData[index]) + 1;
					}
					break;	
					case PARA_WAKE_TIME:
					{
					
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
				if(verId == 1)
				{
					gFrimPara.factoryDate[0] = 0x20;
					gFrimPara.factoryDate[1] = ((pData[0] % 100 / 10) << 4) | (pData[0] % 10);
					gFrimPara.factoryDate[2] = ((pData[1] % 100 / 10) << 4) | (pData[1] % 10);
					gFrimPara.factoryDate[3] = ((pData[2] % 100 / 10) << 4) | (pData[2] % 10);
				}
				gUpdateParaFlag = 0x1F;
				System_Pare_Save();
				Device_ID_Save();
			}
			sendLen = MakeCmd(CMD_SETPARAS,packParaCnt == paraCnt ? 0x01 : 0x02,szSendBuf,packIdx - 24);
			if(sendLen > 0)
				CallBack(szSendBuf,sendLen);
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
					char *pSplit = NULL;
					rspCode = 2;
					memset(&gFtpParas, 0, sizeof(gFtpParas));
					if(strncmp((char*)&pData[index + 2],"FTP://",6) == 0 || strncmp((char*)&pData[index + 2],"ftp://",6)==0)
					{
						if(sscanf((char*)&pData[index + 8],"%[^:]:%[^@]@%[^/;]%*c%[^;]",gFtpParas.ftpUser,gFtpParas.ftpPwd,gFtpParas.ftpUrl,gFtpParas.ftpDir) >= 3)
						{
							//拆分路径和文件名
							if((pSplit = strrchr(gFtpParas.ftpDir,'/')) != NULL){
								pSplit++;
							}else{
								pSplit = gFtpParas.ftpDir;
							}
							strcpy(gFtpParas.ftpName,pSplit);
							*pSplit = '\0';
							//拆分地址和端口
							if((pSplit = strstr(gFtpParas.ftpUrl,":")) != NULL){
								sscanf(pSplit,":%hu",&gFtpParas.ftpPort);
								*pSplit = '\0';
							}else{
								gFtpParas.ftpPort = 21;
							}
							rspCode = 1;
							gIsPendingUpdate = 0x55;
						}
					}
				}
				break;
				case 0x03: //终端重启
				{
					rspCode = 1;
					isReboot = 1;
				}
				break;
				case 0xE0:
				case 0x90: //登入测试
				{
					if(gprs_State && gRealData.carState == 1 && gLogoutCmd == 1)
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
				case 0xE1:
				case 0x91: //登出测试
				{
					if(gprs_State && gRealData.carState == 1 && gLogoutCmd == 0)
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
				case 0xE2:
				case 0x92: //预警测试
				{
					if(gprs_State && gRealData.carState == 1 && gLogoutCmd == 0)
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
				case 0xE3:
				case 0x93: //离线测试
				{
					if(gprs_State && gRealData.carState == 1 && gLogoutCmd == 0)
					{
						rspCode = 1;
						gTerminalState.ctrlOfflineSta = 1;
						gTerminalState.ctrlOfflineStaTimeStamp = osKernelGetTickCount();
					}
					else
					{
						strcpy((char*)&szSendBuf[packIdx],"车辆启动、网络正常、先登入才能操作！");
						packIdx += 36;		
					}
				}
				break;
				case 0xE4:
				case 0x94: //取消预警
				{
					rspCode = 1;
					gRealData.alarmLevel = 0;
					gRealData.dc2dcStateAlert = 0;
					gTerminalState.ctrlAlarmSta = 0;
				}
				break;
				case 0xE5:
				case 0x95: //取消断网
				{
					rspCode = 1;
				}
				break;
				case 0xE6:
				case 0x96: //实时报文解析
				{
					rspCode = 1;
				}
				break;
				case 0xE7:
				case 0x97: //取消实时报文解析
				{
					rspCode = 1;				
					gTerminalState.gbDebug = 0;
				}
				break;
				default:;break;
			}
			szSendBuf[packIdx++] = 0;
			sendLen = MakeCmd(CMD_CTRL,rspCode,szSendBuf,packIdx - 24);
			if(sendLen > 0)
				CallBack(szSendBuf,sendLen);
			osDelay(1000);
			if(rspCode == 1)
			{
				if(pData[index] == 0x90)
					gLogoutCmd = 0;
				else if(pData[index] == 0x91)
					gLogoutCmd = 1;
				else if(pData[index] == 0x92)
				{
					gTerminalState.ctrlAlarmSta = 1;
					gTerminalState.ctrlAlarmStaTimeStamp = osKernelGetTickCount();							
				}
				else if(pData[index] == 0x95)
				{
					gTerminalState.ctrlOfflineSta = 0;
				}
				else if(pData[index] == 0x96)
				{
					gTerminalState.gbDebug = 1;
				}
			}
			if(isReboot == 1)
			{
				BoardReset();
			}
		}
		break;
	}
}

