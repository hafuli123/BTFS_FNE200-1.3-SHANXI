#include "Fun_Net.h"

#include "bsp_sys.h"
#include "bsp_ec20.h"
#include "string.h"

static	char moduleVer[33] = {0};					//模块版本
static	char iccid[21] = {0};							//ICCID
static	char imei[16] = {0};							//IMEI
static	char imsi[16] = {0};							//IMSI
static uint8_t csq = 0;										//CSQ
static uint8_t netMode = 0;								//网络模式 1-2G 2-3G 3-4G
static char ip[16];												//本机IP
static uint8_t step = 0;									//拨号步骤
static uint32_t timeOutTimeStamp = 0;			//用于超时判断，超时复位或返回上一步
static uint32_t	operationTimeStamp = 0;		//当前步骤重试间隔判断
static const osMutexAttr_t mutex = { NULL, osMutexRecursive|osMutexPrioInherit, NULL, 0 };	//串口使用互斥信号，防止多线程同时调用
static osMutexId_t mutexId;

void Fun_Net_Init(void)
{
	/* 创建互斥信号量 */
	mutexId = osMutexNew (&mutex);
	//GPRS模块初始
	Chip_Ec20Init();
	step = FUN_GPRS_GET_MOD;
	timeOutTimeStamp = 0;
	operationTimeStamp = 0;
}

void Fun_Net_Run(void)
{
	uint32_t sysTicks =	Bsp_GetSysTicks_ms();
	uint32_t timeCoutVal = 0xFFFFFFFF;
	uint8_t ret = 0;
	//睡眠状态
	if((step & 0x80) > 0)
	{
		Bsp_Delay_ms(100);
		return;
	}
	//检测模块
	if(step == FUN_GPRS_GET_MOD)
	{
		if(sysTicks - operationTimeStamp >= 2000)
		{
			operationTimeStamp = sysTicks;
			osMutexAcquire(mutexId,osWaitForever);
			//版本
			ret = Chip_Ec20getVer(moduleVer,sizeof(moduleVer));
			if(ret > 0)
			{
				Bsp_Delay_ms(100);
				//IMEI
				ret = Chip_Ec20getIMEI(imei,sizeof(imei));
			}
			osMutexRelease(mutexId);
			if(ret > 0)
			{
				ret = 1;
			}
			if(ret == 0)
			{
				timeCoutVal = 30000;
			}
		}
	}
	//检测SIM卡，ICCID，IMSI，信号
	else if(step == FUN_GPRS_GET_SIM)
	{
		if(sysTicks - operationTimeStamp >= 2000)
		{
			operationTimeStamp = sysTicks;
			osMutexAcquire(mutexId,osWaitForever);
			//ICCID
			ret = Chip_Ec20getICCID(iccid,sizeof(iccid));
			if(ret > 0)
			{
				Bsp_Delay_ms(1000);
				//IMSI
				ret = Chip_Ec20getIMSI(imsi,sizeof(imsi));
			}
			osMutexRelease(mutexId);
			if(ret > 0)
			{
				ret = 1;
			}
			csq = Chip_Ec20GetSignal();
			if(ret == 0)
			{
				timeCoutVal = 20000;
			}
		}
	}
	//检测网络，信号，网络模式
	else if(step == FUN_GPRS_GET_NET)
	{
		if(sysTicks - operationTimeStamp >= 2000)
		{
			operationTimeStamp = sysTicks;
			osMutexAcquire(mutexId,osWaitForever);
			ret = Chip_Ec20GetNetStatus();
			Bsp_Delay_ms(1000);
			csq = Chip_Ec20GetSignal();
			Bsp_Delay_ms(1000);
			netMode = Chip_Ec20GetNetworkMode();
			osMutexRelease(mutexId);
			if(ret == 0)
			{
				timeCoutVal = 60000;
			}
		}
	}
	//拨号，本机IP，信号
	else if(step == FUN_GPRS_DIAL)
	{
		if(sysTicks - operationTimeStamp >= 2000)
		{
			operationTimeStamp = sysTicks;
			osMutexAcquire(mutexId,osWaitForever);
			ret = Chip_Ec20PppdCall(ip);
			csq = Chip_Ec20GetSignal();
			osMutexRelease(mutexId);
			if(ret != 0)
			{
				timeCoutVal = 30000;
			}
		}
	}
	//拨号成功，检测状态，网络模式
	else
	{
		timeCoutVal = 0xFFFFFFFF;
		//检测异常
		osMutexAcquire(mutexId,osWaitForever);
		if(Chip_Ec20Mon(&csq,&netMode) == 0)
		{
			//重拨
			timeCoutVal = 0;
		}
		osMutexRelease(mutexId);
	}
	//成功，进入下一步
	if(ret == 1)
	{
		operationTimeStamp = 0;
		timeOutTimeStamp = sysTicks;
		step++;
	}
	//超时，返回上一步
	if(sysTicks - timeOutTimeStamp >= timeCoutVal)
	{
		operationTimeStamp = 0;
		timeOutTimeStamp = sysTicks;
		if(step > FUN_GPRS_GET_SIM)
		{
			step--;
		}
		if(step <= FUN_GPRS_GET_SIM)
		{
			osMutexAcquire(mutexId,osWaitForever);
			Chip_Ec20Reset();
			osMutexRelease(mutexId);
		}
	}
	Bsp_Delay_ms(100);
}

//GPRS模块获取状态
FUN_GPRS_STA Fun_Gprs_GetSta(void)
{
	return (FUN_GPRS_STA)step;
}

//GPRS模块信号
uint8_t Fun_Gprs_Csq(void)
{
	return csq;
}

//GPRS模块睡眠
void Fun_Gprs_Sleep(void)
{
	osMutexAcquire(mutexId,osWaitForever);
	step |= 0x80;
	Chip_Ec20Sleep();
	osMutexRelease(mutexId);
}

//GPRS模块唤醒
void Fun_Gprs_WakeUp(void)
{
	osMutexAcquire(mutexId,osWaitForever);
	Chip_Ec20WakeUp();
	step &= 0x7F;
	osMutexRelease(mutexId);
}

//GPRS模块获取ICCID
uint8_t Fun_Gprs_getICCID(char *buf, uint16_t len)
{
	if(len >= strlen(iccid))
	{
		strcpy(buf,iccid);
		return strlen(iccid);
	}
	return 0;
}

//GPRS模块获取IMEI
uint8_t Fun_Gprs_getIMEI(char *buf, uint16_t len)
{
	if(len >= strlen(imei))
	{
		strcpy(buf,imei);
		return strlen(imei);
	}
	return 0;	
}

//GPRS模块获取IMSI
uint8_t Fun_Gprs_getIMSI(char *buf, uint16_t len)
{
	if(len >= strlen(imsi))
	{
		strcpy(buf,imsi);
		return strlen(imsi);
	}
	return 0;	
}

//GPRS模块获取版本
uint8_t Fun_Gprs_getVer(char *buf, uint16_t len)
{
	if(len >= strlen(moduleVer))
	{
		strcpy(buf,moduleVer);
		return strlen(moduleVer);
	}
	return 0;	
}

//GPRS模块TCP连接
uint8_t Fun_Gprs_Tcp_connect(uint8_t socket_fd,char *hostname,uint16_t hostport)
{
	return Chip_Ec20TcpConnect(socket_fd,hostname,hostport);
}

//GPRS模块TCP断开
uint8_t Fun_Gprs_Tcp_disconnect(uint8_t socket_fd)
{
	return Chip_Ec20TcpClose(socket_fd);
}

//GPRS模块TCP接收回调
FUN_GPRS_RET Fun_Gprs_Tcp_Set_RecvCallBack(uint8_t socket_fd,void (*unpackCallBack)(uint8_t,uint8_t*,uint16_t len))
{
	if(Chip_Ec20Tcp_Set_RecvCallBack(socket_fd,unpackCallBack))
	{
		return FUN_GPRS_SUCCEED;
	}
	else
	{
		return FUN_GPRS_FAILED;
	}
}

//GPRS模块TCP状态
FUN_GPRS_RET Fun_Gprs_Tcp_Sta(uint8_t socket_fd)
{
	if(Chip_Ec20TcpSta(socket_fd) == 1)
	{
		return FUN_GPRS_CONNECTED;
	}
	else
	{
		return FUN_GPRS_NONE;
	}
}

//GPRS模块TCP发送
FUN_GPRS_RET Fun_Gprs_Tcp_send(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	osMutexAcquire(mutexId,osWaitForever);
	if(Chip_Ec20TcpSend(socket_fd,buf,len) == 1)
	{
		osMutexRelease(mutexId);
		return FUN_GPRS_SUCCEED;
	}
	else
	{
		osMutexRelease(mutexId);
		return FUN_GPRS_FAILED;
	}
}

//GPRS模块TCP接收
uint16_t Fun_Gprs_Tcp_recv(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	osMutexAcquire(mutexId,osWaitForever);
	uint8_t ret = Chip_Ec20TcpRecv(socket_fd,buf,len);
	osMutexRelease(mutexId);
	return ret;
}

//GPRS模块FTP连接
uint8_t Fun_Gprs_Ftp_connect(char *hostname,uint16_t hostport,char *username,char *password)
{
	osMutexAcquire(mutexId,osWaitForever);
	uint8_t ret = Chip_Ec20Ftp_connect(hostname,hostport,username,password);
	osMutexRelease(mutexId);
	return ret;
}

//GPRS模块FTP断开
FUN_GPRS_RET Fun_Gprs_Ftp_disconnect(void)
{
	osMutexAcquire(mutexId,osWaitForever);
	if(Chip_Ec20Ftp_disconnect() == 1)
	{
		osMutexRelease(mutexId);
		return FUN_GPRS_SUCCEED;
	}
	else
	{
		osMutexRelease(mutexId);
		return FUN_GPRS_FAILED;
	}
}

//GPRS获取文件大小
uint32_t Fun_Gprs_Ftp_GetFileSize(char *filePach,char* fileName)
{
	osMutexAcquire(mutexId,osWaitForever);
	uint32_t ret = Chip_Ec20Ftp_GetFileSize(filePach,fileName);
	osMutexRelease(mutexId);
	return ret;
}

//GPRS获取文件内容
uint32_t Fun_Gprs_Ftp_ReadFile(char* fileName,uint32_t filePos,uint32_t getSize,uint8_t* buff)
{
	osMutexAcquire(mutexId,osWaitForever);
	uint32_t ret = Chip_Ec20Ftp_ReadFile(fileName,filePos,getSize,buff);
	osMutexRelease(mutexId);
	return ret;
}
