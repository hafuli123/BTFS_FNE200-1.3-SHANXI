#include "bsp_gprs.h"
#include "bsp_io.h"
#include "bsp_uart_fifo.h"
#include "bsp_gprs_EC20.h"
#include "cmsis_os2.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "stdio.h"

static const osMutexAttr_t mutex = { NULL, osMutexRecursive|osMutexPrioInherit, NULL, 0 };//串口使用互斥信号，防止多线程同时调用
static osMutexId_t mutexId;

static uint8_t gprs_abort = 0;					                                			//GPRS断开标识
static uint8_t isFirstLoad = 1;					                                			//GPRS启动初始化
static uint8_t getIccidFlag = 0;																							//获取ICCID标识
uint8_t gprsRecvBuf[2048]; 																										//接收数据缓冲区
char ipbuf[100];												                                			//模块发送缓冲区
static char iccid[20];								                                				//ICCID
                    			
TCP_CLIENT_PARA tcpClientPara[MAX_LINK];																			//TCP客户端参数

static void simInit(void);																										//芯片初始化
static uint8_t simGetCCID2buf(char* rbuf, uint8_t rLen);										//获取ICCID保持到ICCID缓冲区
static uint8_t simGetIMEI(char* rbuf, uint8_t rLen);
static uint8_t simGetIMSI(char* rbuf, uint8_t rLen);
static uint8_t simGetNetworkMode(void);
static uint16_t simReadData(void);																						//读取并解包数据
static uint8_t chkSimModel(void);

uint8_t gFtp_Mode = 0;
uint8_t gprs_State = 0;
FTP_PARS gFtpParas;

/* GPRS断开标志 */
void gprsRedial(void)
{
	uint8_t i;
	for(i=0;i<MAX_LINK;i++)
	{
		tcpClientPara[i].linkState = 0;
	}
	simIPClose();
	gprs_abort = 1;   /* GPRS断开 */
	comClearRxFifo(RS2_COM);
}

/* GPRS链路断开重连 */
void reConnect(uint8_t link)
{
	if(link < MAX_LINK)
	{
		simLinkClose(link);
		tcpClientPara[link].linkState = 0;   /* GPRS断开 */
	}
}

uint8_t getNAckTm(uint8_t link)
{
	return tcpClientPara[link].nAckTm;
}


void ftpUpdateApp(void)
{
	gFtp_Mode = 1;   /* FTP升级模式 */
}

/*
检测链路状态
返回值 0：全部链路未连接  1：全部链路正常连接 2：部分链路正常
*/
static uint8_t checktcpClientState()
{
	uint8_t i;
	uint8_t state1 = 0,state2 = 0 ,ret = 0;
	for(i=0;i<MAX_LINK;i++)
	{
		if(tcpClientPara[i].used == 1 && tcpClientPara[i].linkState == 1)
		{
			//检测到正常的链路
			state1 = 1;
		}
		if(tcpClientPara[i].used == 1 && tcpClientPara[i].linkState == 0)
		{
			//检测到链路不正常
			state2 = 1;
		}
	}
	if(state1==0)
	{
		//所有链路异常
		ret = 0;
		gprs_State = 1;
	}
	else if(state2 == 0)
	{
		//所有链路正常
		ret = 1;
		gprs_State = 3;
	}
	else
	{
		//部分链路异常
		ret = 2;
		gprs_State = 2;
	}
	return ret;
}

static int16_t ConnectInterval = -1;			//连接周期计数
static uint16_t ConnectCnt = 0;						//连接次数计数
static uint8_t st = 0;
static int8_t sttmp = -1;
static uint8_t pppdCallCnt = 0;   //拨号次数，超过10次，则重启终端
static uint8_t getSimFaultCnt = 0;//获取SIM卡状态失败计算，如果超过10都没有数据，则反转GPRS开关
static uint8_t getSimModeled = 0; //获取到SIM模块标识

void gprsPppdask(void)
{
		int8_t ret = 0, i = 0;	
		if(1 == isFirstLoad )
		{
			/* 创建互斥信号量 */
			mutexId = osMutexNew (&mutex);
			simInit();
			isFirstLoad = 0;
			return;
		}
		
		if(1 == gSysPara.isDebug)
		{
			if(sttmp!= st)
			{
				sttmp = st;
			}
		}
	
		if(st == 0)      /* 1.开始拨号前准备，检测SIM卡状态，如果模块有GPS功能则打开GPS */
		{
			gTerminalState.networkMode = 0;
			if(getSimModeled == 1)
			{
				//模块正常，发送休眠指令
				simOpenSleepModel();						//EC20开启睡眠模式

				//模块正常，开始检查
				ret = simGetSIMCardStatus();//检查SIM卡,延时1秒
				if(2 == ret)//无SIM或异常
				{
					gTerminalState.simCardState = 0;
				}
				else if(1 == ret)
				{
					gTerminalState.simCardState = 1;
					getSimFaultCnt = 0;
					//检测到SIM卡
					if(getIccidFlag == 0)
					{
						if(simGetCCID2buf(iccid,20!=0))
						{
							memcpy(gTerminalState.iccid,iccid,20);
							osDelay(10);
							if(simGetIMSI(gTerminalState.imsi,16)!=0)
							{
								getIccidFlag = 1;
								st = 1;
							}
						}
					}
					else
					{
						st = 1;
					}
					osDelay(10);
					gTerminalState.csq = simGetSignalQuality();
				}
			}
			else
			{
				//模块检测
				getSimModeled = chkSimModel();
				if(getSimModeled == 1)
				{
					getSimFaultCnt = 0;
				}
			}
			if(++getSimFaultCnt > 30)
			{
				getSimFaultCnt = 0;
				simReset();
			}
		}
		
		else if(st==1) /*检查网络状态*/
		{			
			ret = simGetGPRSNetStatus();
			gTerminalState.csq = simGetSignalQuality();
			if(ret == 1)
			{
				pppdCallCnt = 0;
				st = 2;
			}
			else if(++pppdCallCnt > 30)
			{
				st = 0;
				pppdCallCnt = 0;
				simReset();
			}				
		}
		
		else if(st == 2)  /* 2.PPPD拨号连接服务器 */
		{
			if(1 == simPppdCallGPRS())
			{//拔号成功
				gTerminalState.csq = simGetSignalQuality();
				osDelay(100);
				gTerminalState.networkMode = simGetNetworkMode();
				pppdCallCnt = 0;
				st = 3;
				gprs_State = 1;
				ConnectCnt = 0;
				for(i=0;i<MAX_LINK;i++)
				{
					//默认连接时间5秒
					tcpClientPara[i].linkState = 0;
					tcpClientPara[i].connectCnt = 25;
					tcpClientPara[i].connectHZ = 25;
				}
			  comClearRxFifo(RS2_COM);	/* 清零串口接收缓冲区 */
			}
			else
			{
				if(++pppdCallCnt > 50)
				{//2秒拨号没有成功
					st = 0;
					pppdCallCnt = 0;
					simReset();
				}
			}
		}
		else if(st == 3) /* 4.判断GPRS连接状态，随时准备重拨 */
		{
			if(++ConnectInterval >= 450)			//90秒，查询一次
			{
				ConnectInterval = 0;
				//查询网络状态
				simReadData();						 //先读取完数据再发送命令
				simQueryConnectionStatus();//发送网络查询命令
				simReadData();						 //读取数据和网络状态
			}
			if(checktcpClientState()!=1)
			{	//有链路不正常
				for(i=0;i<MAX_LINK;i++)
				{
					if(tcpClientPara[i].linkState == 0 && tcpClientPara[i].used==1)//链路已使用，且不正常
					{
						if(++tcpClientPara[i].connectCnt >= tcpClientPara[i].connectHZ)//链路重连时间到
						{
							if(tcpClientPara[i].connectCnt == 300)//1分钟未连接
							{
								simLinkClose(i);
							}
							tcpClientPara[i].connectCnt = 0;
							if(tcpClientPara[i].connectHZ <= 450)//下次重连时间延长5秒，最长90秒
							{
								tcpClientPara[i].connectHZ += 25;
							}
							simTcpConnect(i,tcpClientPara[i].address,tcpClientPara[i].port);
							simReadData();
							if(ConnectInterval < 435)//450的时候查询，预留15，即3秒
								ConnectInterval = 435;
						}
					}
				}
			}
			simReadData();
			if(checktcpClientState()==0)
			{
				//全部未连接,连接周期6秒
				ConnectCnt++;//重播计时
				if(ConnectCnt >= 90)
				{//重试3次连接网络
					gprs_abort = 1;
					ConnectCnt = 0;
				}
			}
			else
			{
				//有一条链路连接成功
				ConnectCnt = 0;
			}
			if(gprs_abort == 1 && gFtp_Mode == 0)
			{
				/* GPRS断开后，复位模块 重新进入拨号流程 */
				gprs_abort = 0;
				simReset();
				st = 0;
			}
		}
		if(gFtp_Mode == 1)//加入条件
		{
			//关闭外设
			USART_Cmd(USART3,DISABLE);	//关闭GPS串口
			BSP_Ioctl(CAN_STB_ID,OFF);	//CAN睡眠
			osDelay(1000);
			gprsRedial();
			osDelay(1000);
			simFtpUpdate();
		}
		if(gTerminalState.csq > 14 && gTerminalState.csq <= 31)
			gTerminalState.signalCnt = 5;
		else if(gTerminalState.csq >= 9 && gTerminalState.csq <= 14)
			gTerminalState.signalCnt = 4;
		else if(gTerminalState.csq >= 5 && gTerminalState.csq < 9)
			gTerminalState.signalCnt = 3;
		else if(gTerminalState.csq >= 3 && gTerminalState.csq < 5)
			gTerminalState.signalCnt = 2;
		else if(gTerminalState.csq > 0 && gTerminalState.csq < 3)
			gTerminalState.signalCnt = 1;
		else
			gTerminalState.signalCnt = 0;
}
	
void simInit(void)
{
	osDelay(2000);	
	comClearRxFifo(RS2_COM);
}

/*
*********************************************************************************************************
*	函 数 名: simReset
*	功能说明: 模块检查
*	形    参: 无
*	返 回 值: 模块检查成功
*********************************************************************************************************
*/
static uint8_t chkSimModel(void)
{
	uint8_t ret = 0;
	memset(gprsRecvBuf,0,sizeof(gprsRecvBuf));
	comSendBuf(RS2_COM,(uint8_t*)"ATI\r\n",5);	/* 发送 AT 命令 */	
	osDelay(200);
	if(comGetBufDelay(RS2_COM,sizeof(gprsRecvBuf),(uint8_t*)gprsRecvBuf,200)>0)	/* 超时 200ms */
	{
		if(strstr((char*)gprsRecvBuf,"OK")!=NULL)
		{
			sscanf(strstr((char*)gprsRecvBuf,"Revision: "),"Revision: %s",gTerminalState.gprsVer);
			if(simGetIMEI(gTerminalState.imei,16) != 0)
			{
				ret = 1;
			}
		}
	}
	return ret;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitGprs
*	功能说明: SIM模块初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitGprs(void)
{		
	BSP_Ioctl(PWR_GPRS_IO,OFF);
	osDelay(100);
	BSP_Ioctl(PWR_GPRS_IO,ON);	
}

/*
*********************************************************************************************************
*	函 数 名: simReset
*	功能说明: 给SIM模块复位
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void simReset(void)
{
	uint8_t i;
	for(i=0;i<MAX_LINK;i++)
	{
		tcpClientPara[i].linkState = 0;
	}
	gprs_State = 0;
	EC20_Reset();
}


/*
*********************************************************************************************************
*	函 数 名: simGetSIMCardStatus
*	功能说明: 查询SIM卡状态
*	形    参: 无
*	返 回 值: SIM卡状态
*********************************************************************************************************
*/
uint8_t simGetSIMCardStatus(void)
{
	return EC20_SIMCardStatus();
}

/*
*********************************************************************************************************
*	函 数 名: simQueryLinkStatus
*	功能说明: 查询GPRS 链路连接情况
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
uint8_t simQueryLinkStatus(uint8_t link)
{
	if(tcpClientPara[link].linkState == 1)//连接正常，断开重连时间3秒
	{
		tcpClientPara[link].connectCnt = 10;
		tcpClientPara[link].connectHZ = 25;
	}
	//主动控制离线
	if(gTerminalState.ctrlOfflineSta == 1 && link != 5)
	{
		if(osKernelGetTickCount() - gTerminalState.ctrlOfflineStaTimeStamp > 610000)
		{
			gTerminalState.ctrlOfflineSta = 0;
		}
		else
		{
			return 0;
		}
	}
//	//测试定时离线
//	else
//	{
//		if(osKernelGetTickCount() - gTerminalState.ctrlOfflineStaTimeStamp > 1200000)
//		{
//			gTerminalState.ctrlOfflineSta = 1;
//			gTerminalState.ctrlOfflineStaTimeStamp = osKernelGetTickCount();
//		}
//	}
	return tcpClientPara[link].linkState;
}

/*
*********************************************************************************************************
*	函 数 名: simQueryConnectionStatus
*	功能说明: 查询GPRS TCP/IP连接情况
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
void simQueryConnectionStatus(void)
{
	osMutexAcquire(mutexId,osWaitForever);
	osDelay(200);
	EC20_QueryConnectionStatus();
	osMutexRelease(mutexId);
}
/*
*********************************************************************************************************
*	函 数 名: simGetSignalQuality
*	功能说明: 查询信号强度
*	形    参: 无
*	返 回 值: 信号强度
*********************************************************************************************************
*/
uint8_t simGetSignalQuality (void)
{
	return EC20_GetSignalQuality();
}
/*
*********************************************************************************************************
*	函 数 名: simIPClose
*	功能说明: 关闭TCP连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void simIPClose(void)
{
	EC20_IPClose();
}

/*
*********************************************************************************************************
*	函 数 名: simLinkClose
*	功能说明: 关闭TCP链路
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void simLinkClose(uint8_t link)
{
	EC20_linkClose(link);
}

/*
*********************************************************************************************************
*	函 数 名: simTcpConnect
*	功能说明: 创建TCP连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void simTcpConnect(uint8_t linkIndex,char *domain,uint16_t port)
{
	osMutexAcquire(mutexId,osWaitForever);
	osDelay(200);
	EC20_TCP_Connect(linkIndex,domain,port);
	osMutexRelease(mutexId);
}

/*
*********************************************************************************************************
*	函 数 名: simPppdCallGPRS
*	功能说明: GPRS拨号连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t simPppdCallGPRS(void)
{
	return EC20_PppdCallGPRS();
}

/*
*********************************************************************************************************
*	函 数 名: simOpenSleepModel
*	功能说明: 开启睡眠模式
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t simOpenSleepModel(void)
{
	return EC20_OpenSleepModel();
}

/*
*********************************************************************************************************
*	函 数 名: simFtpUpdate
*	功能说明: ftp升级时数据解析
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t simFtpUpdate(void)
{
	return EC20_ftpDownload();	
}

/*
*********************************************************************************************************
*	函 数 名: simSendData
*	功能说明: GPRS发送数据
*	形    参: 无
*	返 回 值: 2：网络错误 0：发送成功 1：发送失败
*********************************************************************************************************
*/
uint8_t simSendData(uint8_t linkIndex,uint8_t* wbuf,uint16_t len)
{
	uint8_t ret;
	if(simQueryLinkStatus(linkIndex)==0)
	{
		return 2;
	}
	osMutexAcquire(mutexId,osWaitForever);
	ret = EC20_SendData(linkIndex,wbuf,len);
	osMutexRelease(mutexId);
	if(ret == 2)
	{//网络错误,重连
		reConnect(linkIndex);
	}
	return ret;
}

/*
*********************************************************************************************************
*	函 数 名: simReadData
*	功能说明: GPRS接收数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint16_t simReadData(void)
{
	return EC20_ReadData();
}

/*
*********************************************************************************************************
*	函 数 名: simReadData
*	功能说明: GPRS接收数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t simGetNetworkMode(void)
{
	return EC20_GetNetworkMode();
}

uint8_t simSetUnpackdataFun(uint8_t link,char *address,uint16_t port,tcpUnpackDataCallBack UnpackDataFunc,uint8_t used)
{
	if(address != NULL && strlen(address) >= 4 && strcmp(address,"127.0.0.1") != 0 && port > 0)
	{
		tcpClientPara[link].used = used;
		tcpClientPara[link].address = address;
		tcpClientPara[link].port = port;
		tcpClientPara[link].UnpackData = UnpackDataFunc;
	}
	else
	{
		tcpClientPara[link].used = 0;
		tcpClientPara[link].address = 0;
		tcpClientPara[link].port = 0;
		tcpClientPara[link].UnpackData = 0;		
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: simGetCCID
*	功能说明: 获取CCID
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t simGetCCID(uint8_t* rbuf, uint8_t rLen)
{
	if(memcmp(gSysPara.iccid,"8986",4) == 0)
	{
		memcpy(rbuf,gSysPara.iccid,20);
	}
	else
		memcpy(rbuf,iccid,20);
	return 20;
}

/*
*********************************************************************************************************
*	函 数 名: simGetCCID2buf
*	功能说明: 获取CCID到缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t simGetCCID2buf(char* rbuf, uint8_t rLen)
{
	return EC20_GetCCID(rbuf,rLen);
}

/*
*********************************************************************************************************
*	函 数 名: simGetCCID2buf
*	功能说明: 获取CCID到缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t simGetIMEI(char* rbuf, uint8_t rLen)
{
	return EC20_GetIMEI(rbuf,rLen);
}

/*
*********************************************************************************************************
*	函 数 名: simGetCCID2buf
*	功能说明: 获取CCID到缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t simGetIMSI(char* rbuf, uint8_t rLen)
{
	return EC20_GetIMSI(rbuf,rLen);
}

/*
*********************************************************************************************************
*	函 数 名: simGetGPRSNetStatus
*	功能说明: GPRS拨号连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t simGetGPRSNetStatus(void)
{
	return EC20_GetGPRSNetStatus();
}

