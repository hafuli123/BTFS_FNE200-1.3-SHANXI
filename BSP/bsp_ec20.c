#include "bsp_ec20.h"
#include "bsp_io.h"
#include "bsp_uart_fifo.h"
#include "bsp_sys.h"
#include "stdio.h"
#include "string.h"
#include "algo_string.h"

#define EC20_SEND_BUF_SIZE			384						//EC20缓存
#define EC20_RECV_BUF_SIZE			1024					//EC20缓存
#define GPRS_COM 					RS2_COM							//EC20串口号
#define EC20_TCP_CNT 			6										//EC20 TCP 数量

typedef struct
{
	uint8_t sta;																//0 - 断开 1 - 连接
	char* domin;																//域名
	uint16_t port;															//端口
	uint8_t oper;																//0-无操作 1-连接 2-断开
	uint32_t operTime;													//操作时间
	uint32_t Interval;													//操作间隔
	uint16_t localPort;													//本地端口
	void (*unpackCallBack)(uint8_t,uint8_t*,uint16_t len);
}TCP_PARA_T;

static char ec20SendBuf[EC20_SEND_BUF_SIZE];	//EC20缓存
static char ec20RecvBuf[EC20_RECV_BUF_SIZE];	//EC20接收缓存
static uint8_t netCsq;												//网络信号
static TCP_PARA_T tcpPara[EC20_TCP_CNT];			//TCP参数
static uint8_t netMode;												//网络模式

//发送命令
static uint8_t comSendChk(char *cmd,char* chk,uint16_t _usTimeOut)
{

	comSendBuf(GPRS_COM,(uint8_t*)cmd,strlen(cmd));
	comSendBuf(GPRS_COM,(uint8_t*)"\r\n",2);
	if(comChkBufDelay(GPRS_COM,chk,_usTimeOut))
	{
//		comSendBuf(RS1_COM,(uint8_t*)cmd,strlen(cmd));
		return 1;
	}

	return 0;
}

//接收响应
static uint16_t comGetResponse(char *_pBuf, uint16_t _usBufSize, uint16_t _usTimeOut)
{
	uint16_t len;
	len  = comGetBufDelay(GPRS_COM,_usBufSize,(uint8_t*)_pBuf,_usTimeOut);
	_pBuf[len] = 0;
	return len;
}

static void Chip_Ec20ResetTcpPara(void)
{
	uint8_t socket_fd;
	for(socket_fd = 0; socket_fd < EC20_TCP_CNT;socket_fd++)
	{
		tcpPara[socket_fd].sta = 0;
		tcpPara[socket_fd].oper = 0;
	}
}

//EC20初始化
void Chip_Ec20Init()
{
	BSP_Ioctl(PWR_GPRS_IO,ON);
	BSP_Ioctl(GPRS_DTR_ID,ON);
}

//EC20复位
void Chip_Ec20Reset()
{
	BSP_Ioctl(PWR_GPRS_IO,OFF);
	Bsp_Delay_ms(100);
	BSP_Ioctl(PWR_GPRS_IO,ON);
}

void Chip_Ec20Sleep(void)
{
	uint8_t i;
	for(i = 0;i < 3;i++)
	{
		if(comSendChk("AT+QINDCFG=\"csq\",0","OK",1000) == 1)
		{
			break;
		}
	}
	for(i = 0;i < 3;i++)
	{
		if(comSendChk("AT+QSCLK=1","OK",1000) == 1)
		{
			break;
		}
	}
	BSP_Ioctl(GPRS_DTR_ID,OFF);
	comClearRxFifo(GPRS_COM);
	Bsp_Delay_ms(2000);
}

void Chip_Ec20WakeUp(void)
{
	uint8_t i;
	BSP_Ioctl(GPRS_DTR_ID,ON);
	Bsp_Delay_ms(2000);
	Chip_Ec20Mon(&netCsq,&netMode);
	for(i = 0;i < 3;i++)
	{
		if(comSendChk("AT+QSCLK=0","OK",1000) == 1)
		{
			break;
		}
	}
	Chip_Ec20Mon(&netCsq,&netMode);
	for(i = 0;i < 3;i++)
	{
		if(comSendChk("AT+QINDCFG=\"csq\",1,0","OK",1000) == 1)
		{
			break;
		}
	}
	Chip_Ec20Mon(&netCsq,&netMode);
}

//EC20 TCP连接
uint8_t Chip_Ec20TcpConnect(uint8_t socket_fd,char *domain,uint16_t port)
{
	if(socket_fd < EC20_TCP_CNT)
	{
		if(tcpPara[socket_fd].oper != 2)
		{
			tcpPara[socket_fd].domin = domain;
			tcpPara[socket_fd].port = port;
			tcpPara[socket_fd].oper = 1;
		}
		return 1;
	}
	return 0;
}

//EC20 TCP关闭
uint8_t Chip_Ec20TcpClose(uint8_t socket_fd)
{
	if(socket_fd < EC20_TCP_CNT)
	{
		tcpPara[socket_fd].oper = 2;
		return 1;
	}
	return 0;
}

uint8_t Chip_Ec20Tcp_Set_RecvCallBack(uint8_t socket_fd,void (*unpackCallBack)(uint8_t,uint8_t*,uint16_t len))
{
	tcpPara[socket_fd].unpackCallBack = unpackCallBack;
	return 1;
}

//EC20 TCP状态
uint8_t Chip_Ec20TcpSta(uint8_t socket_fd)
{
	if(socket_fd < EC20_TCP_CNT)
		return tcpPara[socket_fd].sta;
	return 0;
}

//GPRS模块TCP发送
uint8_t Chip_Ec20TcpSend(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	uint8_t i;
	sprintf(ec20SendBuf,"AT+QISEND=%d,%d",socket_fd,len);
	for(i = 0;i < 3;i++)
	{
		if(comSendChk(ec20SendBuf,">",150))
		{
			comSendBuf(GPRS_COM,buf,len);
			Bsp_Delay_ms(200);
			return 1;
		}
		else
		{
			comSendBuf(GPRS_COM,buf,len);
			Bsp_Delay_ms(100);
		}
	}
	return 0;
}

//GPRS模块TCP接收
uint16_t Chip_Ec20TcpRecv(uint8_t socket_fd, uint8_t *buf, uint16_t len)
{
	return 0;
}

//获取版本
uint8_t Chip_Ec20getVer(char* rbuf, uint8_t rLen)
{
	if(comSendChk("ATE0","OK",1000))
	{
		if(comSendChk("ATI","Revision: ",1000))
		{
			char* pGetPos;
			memset(ec20SendBuf,sizeof(ec20SendBuf),0);
			comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
			pGetPos = strstr(ec20SendBuf,"Revision: ");
			if(pGetPos != NULL)
			{
				pGetPos += 10;
				sscanf(pGetPos,"%s",rbuf);
				return strlen(rbuf);
			}
		}
	}
	return 0;
}

//获取IMEI
uint8_t Chip_Ec20getIMEI(char* rbuf, uint8_t rLen)
{
	if(rLen >= 15)
	{
		if(comSendChk("AT+CGSN","\n",1000))
		{
			memset(ec20SendBuf,sizeof(ec20SendBuf),0);
			comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
			if(strstr(ec20SendBuf,"ERROR"))
				return 0;
			if(sscanf(ec20SendBuf,"%*[^0123456789]%15s",rbuf) == 1)
			{
				if(strlen(rbuf)>=15)
				{
					return 15;
				}
			}
		}
	}
	return 0;
}

//获取IMSI
uint8_t Chip_Ec20getIMSI(char* rbuf, uint8_t rLen)
{
	if(rLen >= 15)
	{
		if(comSendChk("AT+CIMI","\n",1000))
		{
			memset(ec20SendBuf,sizeof(ec20SendBuf),0);
			comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
			if(strstr(ec20SendBuf,"ERROR"))
				return 0;
			if(sscanf(ec20SendBuf,"%*[^0123456789]%15s",rbuf) == 1)
			{
				if(strlen(rbuf)>=15)
				{
					return 15;
				}
			}
		}
	}
	return 0;
}


//获取ICCID
uint8_t  Chip_Ec20getICCID(char* rbuf, uint8_t rLen)
{
	if(rLen >= 20)
	{
		if(comSendChk("AT+CPIN?","OK",1000))
		{
			if(comSendChk("AT+QCCID","+QCCID: ",1000))
			{
				char* pGetPos;
				memset(ec20SendBuf,sizeof(ec20SendBuf),0);
				comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
				pGetPos = strstr(ec20SendBuf,"+QCCID: ");
				if(pGetPos != NULL)
				{
					pGetPos += 8;
					sscanf(pGetPos,"%20s",rbuf);
					return 20;
				}
			}
		}
	}
	return 0;
}

//EC20 获取信号
uint8_t Chip_Ec20GetSignal(void)
{
	if(comSendChk("AT+CSQ","+CSQ: ",1000))
	{
		char *pGetPos;
		memset(ec20SendBuf,sizeof(ec20SendBuf),0);
		comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
		pGetPos = strstr(ec20SendBuf,"+CSQ: ");
		if(pGetPos != NULL)
		{
			pGetPos += 6;
			sscanf(pGetPos,"%hhu",&netCsq);
		}
	}
	return netCsq;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_GetGPRSNetStatus
*	功能说明: 查询网络状态
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
uint8_t Chip_Ec20GetNetStatus(void)
{
	/*
		AT+CGATT?
		+CGATT: 1
		
		OK				
	*/	
	comSendChk("AT+CGATT=1","OK",1000);
	if(comSendChk("AT+CGATT?","+CGATT: 1",1000))
	{
		return 1;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_GetGPRSNetStatus
*	功能说明: 查询网络状态
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
uint8_t Chip_Ec20GetNetworkMode(void)
{
	if(comSendChk("AT+QNWINFO","\n",1000))
	{
		memset(ec20SendBuf,sizeof(ec20SendBuf),0);
		comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
		if(strstr((char*)ec20SendBuf,"GSM")!=NULL)
		{
			netMode = 1;
		}
		else if(strstr((char*)ec20SendBuf,"WCDMA")!=NULL || strstr((char*)ec20SendBuf,"TDSCDMA")!=NULL)
		{
			netMode = 2;
		}
		else if(strstr((char*)ec20SendBuf,"LTE")!=NULL)
		{
			netMode = 3;
		}
	}
	return netMode;
}
static uint8_t first = 1;
/*
*********************************************************************************************************
*	函 数 名: EC20_PppdCallGPRS
*	功能说明: GPRS拨号连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t Chip_Ec20PppdCall(char* ip)
{
	/* URC输出口 */
	if(comSendChk("AT+QURCCFG=\"urcport\",\"uart1\"","OK",1000) == 0)
	{
		//return 0;//ec200T联网异常屏蔽
	}
			osDelay(10);
	/* CSQ自动上上报 */
	if(comSendChk("AT+QINDCFG=\"csq\",1,0","OK",1000) == 0)
	{
	//	return 0;//ec200T联网异常屏蔽
	}
			osDelay(10);
	/* 设置APN */
	memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	sprintf(ec20SendBuf,"AT+QICSGP=1,1,\"%s\",\"\",\"\",1","CMNET");
	if(comSendChk(ec20SendBuf,"OK",1000) == 0)
	{
	//	return 0;	//ec200T联网异常屏蔽
	}
	
  if(first)
  {
	  memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	  sprintf(ec20SendBuf,"AT+CFUN=0");
	  comSendChk(ec20SendBuf,"OK",5000);
	  memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	  sprintf(ec20SendBuf,"AT+CFUN=1");
	  comSendChk(ec20SendBuf,"OK",5000);
	  first = 0;
  }	
	/* GPRS拨号 */
	comSendChk("AT+QIACT=1","OK",1000);
	comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
	/* 获取本地地址 */
	if(comSendChk("AT+QIACT?","OK",1000) != 0)
	{
		comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
		if(sscanf(ec20SendBuf,"%*[^\"]\"%[0123456789.]",ip)==1)
		{
			return 1;
		}
	}
	return 0;
}

uint8_t Chip_Ec20Analysis(char* recvBuf,uint16_t dataLen,uint8_t *getCsq,uint8_t *getNetMode)
{
	char *pGetPos;
	uint8_t connectID;
	uint16_t tcpdataLen;
	if(dataLen > 0)
	{
		//TCP状态监测
		pGetPos = recvBuf;
		while((pGetPos = strstr(pGetPos,"+QISTATE")) != NULL)
		{
			uint16_t localPort;
			uint8_t socket_state;
			pGetPos += 10;
			if(sscanf(pGetPos,"%hhu,\"TCP\",%*[^,],%*d,%hu,%hhd",&connectID,&localPort,&socket_state) == 3)
			{
				if(socket_state == 2)
				{
					tcpPara[connectID].Interval = 0;
					tcpPara[connectID].sta = 1;
					tcpPara[connectID].localPort = localPort;
					if(tcpPara[connectID].oper == 1)
					{
						tcpPara[connectID].oper = 0;
					}
				}
				else
				{
					if(socket_state == 4)
					{
						sprintf(ec20SendBuf,"AT+QICLOSE=%d,1\r\n",connectID);
						comSendBuf(GPRS_COM,(uint8_t*)ec20SendBuf,strlen(ec20SendBuf));					
					}
					tcpPara[connectID].sta = 0;
				}
			}
		}
		//信号监测
		pGetPos = strstr(recvBuf,"+QIND: \"csq\"");
		if(pGetPos != NULL)
		{
			pGetPos += 13;
			sscanf(pGetPos,"%hhu",&netCsq);
		}
		//TCP断开监测
		pGetPos = strstr(recvBuf, "\"closed\"");
		if(pGetPos != NULL)
		{
			uint8_t socket_fd;
			pGetPos += 9;
			if(sscanf(pGetPos,"%hhu",&socket_fd)==1)
			{
				tcpPara[socket_fd].sta = 0;
			}
		}
		//+QIURC: \"recv\""
		pGetPos = (char*)recvBuf;
		while((pGetPos = memstr(pGetPos,dataLen - (pGetPos - (char*)recvBuf),"cv\"")) != NULL)
		{
			pGetPos += 4;
			if(sscanf(pGetPos,"%hhu,%hu",&connectID,&tcpdataLen)==2)
			{
				if((pGetPos = strstr(pGetPos,"\r\n"))!=NULL)
				{
					pGetPos += 2;
					if((uint8_t*)pGetPos + tcpdataLen <= (uint8_t*)recvBuf + dataLen)
					{
						if(tcpPara[connectID].unpackCallBack != NULL)
						{
							tcpPara[connectID].unpackCallBack(connectID,(uint8_t*)pGetPos,tcpdataLen);
						}
						pGetPos += tcpdataLen;
					}
				}
			}
		}
		if(getCsq != NULL)
			*getCsq = netCsq;
		if(getNetMode != NULL)
			*getNetMode = netMode;
		//网络断开监测
		if(strstr(recvBuf, "+CPIN") || strstr(recvBuf, "\"pdpdeact\",1"))
		{
			Chip_Ec20ResetTcpPara();
			return 0;
		}
	}
	return 1;
}

//EC20监视
uint8_t Chip_Ec20Mon(uint8_t *getCsq,uint8_t *getNetMode)
{
	static uint32_t operTime = 0;
	static uint8_t oper = 0;
	static uint8_t tcpIdx = 0;
	uint8_t ret = 1,i;
	memset(ec20RecvBuf,0,sizeof(ec20RecvBuf));
	uint16_t dataLen = comGetResponse(ec20RecvBuf,sizeof(ec20RecvBuf),10);
	if(dataLen > 0)
	{
		ret = Chip_Ec20Analysis(ec20RecvBuf,dataLen,getCsq,getNetMode);
	}
	//TCP操作
	if(tcpIdx >= EC20_TCP_CNT)
	{
		tcpIdx = 0;
		if(oper == 1)
		{
			oper = 0;
			comSendChk("AT+QISTATE?","OK",1000);
		}
	}
	//连接
	if(tcpPara[tcpIdx].sta == 0 && tcpPara[tcpIdx].oper == 1)
	{
		if(Bsp_GetSysTicks_ms() - tcpPara[tcpIdx].operTime >= tcpPara[tcpIdx].Interval)
		{
			tcpPara[tcpIdx].operTime = Bsp_GetSysTicks_ms();
			oper = 1;
			memset(ec20SendBuf,0,sizeof(ec20SendBuf));
			sprintf(ec20SendBuf,"AT+QIOPEN=1,%d,\"TCP\",\"%s\",%d,0,1",tcpIdx,tcpPara[tcpIdx].domin,tcpPara[tcpIdx].port);
			if(comSendChk(ec20SendBuf,",0\r\n",1000) != 0)
			{
				tcpPara[tcpIdx].Interval = 0;
				tcpPara[tcpIdx].sta = 1;
				tcpPara[tcpIdx].oper = 0;				
			}
			if(tcpPara[tcpIdx].Interval < 120000)
			{
				tcpPara[tcpIdx].Interval += 15000;
			}
		}
	}
	for(i=0;i<EC20_TCP_CNT;i++)
	{
		//断开
		if(tcpPara[i].sta == 1 && tcpPara[i].oper == 2)
		{
			tcpPara[i].Interval = 0;
			tcpPara[i].oper = 0;
			tcpPara[i].sta = 0;
			memset(ec20SendBuf,0,sizeof(ec20SendBuf));
			sprintf(ec20SendBuf,"AT+QICLOSE=%d,1",i);
			comSendChk(ec20SendBuf,"OK",1000);
		}
	}
	if(Bsp_GetSysTicks_ms() - operTime >= 3000)
	{
		operTime = Bsp_GetSysTicks_ms();
		tcpIdx++;
	}
	return ret;
}

uint8_t Chip_Ec20Ftp_connect(char *hostname,uint16_t hostport,char *username,char *password)
{
	comSendChk("AT+QIACT=1","OK",1000);
	
	comSendChk("AT+QFTPCFG=\"contextid\",1","OK",1000);
	
	comSendChk("AT+QFTPCFG=\"filetype\",0","OK",1000);
	
	comSendChk("AT+QFTPCFG=\"transmode\",1","OK",1000);
	
	comSendChk("AT+QFTPCFG=\"rsptimeout\",90","OK",1000);
	
	memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	sprintf(ec20SendBuf,"AT+QFTPCFG=\"account\",\"%s\",\"%s\"",username,password);
	comSendChk(ec20SendBuf,"OK",1000);
	
	memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	sprintf(ec20SendBuf,"AT+QFTPOPEN=\"%s\",%d",hostname,hostport);
	if(comSendChk(ec20SendBuf,"+QFTPOPEN: 0,0",10000) != 0)
	{
		return 1;
	}
	return 0;
}

uint8_t Chip_Ec20Ftp_disconnect(void)
{
	if(comSendChk("AT+QFTPCLOSE","OK",2000) != 0)
	{
		return 1;
	}
	return 0;
}

uint32_t Chip_Ec20Ftp_GetFileSize(char *filePach,char* fileName)
{
	char* pGetPos;
	uint32_t fileSize = 0;
	memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	sprintf(ec20SendBuf,"AT+QFTPCWD=\"/%s\"",filePach);
	if(comSendChk(ec20SendBuf,"+QFTPCWD: 0,0",10000) != 0)
	{
		memset(ec20SendBuf,0,sizeof(ec20SendBuf));
		sprintf(ec20SendBuf,"AT+QFTPSIZE=\"%s\"",fileName);	
		if(comSendChk(ec20SendBuf,"+QFTPSIZE: 0",10000) != 0)
		{
			fileSize = comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
			Chip_Ec20Analysis(ec20RecvBuf,fileSize,NULL,NULL);
			
			pGetPos = strstr((char*)ec20SendBuf,"+QFTPSIZE: 0");
			pGetPos += 13;
			sscanf(pGetPos,"%d",&fileSize);			
		}
	}
	return fileSize;
}

uint32_t Chip_Ec20Ftp_ReadFile(char* fileName,uint32_t filePos,uint32_t getSize,uint8_t* buff)
{
	uint32_t bufPosition = 0;
	uint8_t ucData = 0,startFlag = 0;
	uint32_t getCharTime = 0;
	comSendChk("AT+QINDCFG=\"csq\",0","OK",1000);
	
	bufPosition = comGetResponse(ec20SendBuf,sizeof(ec20SendBuf),10);
	Chip_Ec20Analysis(ec20RecvBuf,bufPosition,NULL,NULL);
	
	bufPosition = 0;
	comClearRxFifo(GPRS_COM);
	memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	sprintf(ec20SendBuf,"AT+QFTPGET=\"%s\",\"COM:\",%d,%d",fileName,filePos,getSize);
	if(comSendChk(ec20SendBuf,"CONNECT\r\n",10000) != 0)
	{
		memset(buff,0,getSize);
		while(1)
		{
			if(comGetChar(GPRS_COM,&ucData))
			{
				getCharTime = 0;
				buff[bufPosition++] = ucData;
				if(startFlag == 0)
				{
					if(strstr((char*)buff,"CONNECT\r\n"))
					{
						startFlag = 1;
						bufPosition = 0;
					}
				}
			}
			else
			{
				Bsp_Delay_ms(1);
				getCharTime++;
			}
			if(getCharTime >= 20000 || bufPosition >= getSize)
			{
				break;
			}
		}
	}
	comSendChk("AT+QINDCFG=\"csq\",1,0","OK",1000);
	return bufPosition;
}

/* base64 encode */
static void Chip_Ec20_base64enc(char *out, const char *in)
{
    const char code[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=" ;
    int i = 0, x = 0, l = 0;
    for (; *in; in++) {
        x = x << 8 | *in;
        for (l += 8; l >= 6; l -= 6) {
            out[i++] = code[(x >> (l - 6)) & 0x3f];
        }
    }
    if (l > 0) {
        x <<= 6 - l;
        out[i++] = code[x & 0x3f];
    }
    for (; i % 4;) {
        out[i++] = '=';
    }
    out[i] = '\0' ;
}

const uint8_t HTTP_CLIENT_AUTHB_SIZE = 50;	//HTTP授权大小
const char* boundary = "13X8dcFR";					//边界
static uint32_t Post_File_Input_Len = 0;		//上传文件已输入长度
static uint32_t Post_File_Size = 0;					//上传文件大小
static uint8_t httpSetFlag = 0;
uint8_t Chip_Ec20Http_Post_File_Start(char *url,char *username,char *password,char*fileName,uint32_t fileSize)
{
	char cmd[50];
	char* host;
	char* uri;
	uint16_t headIdx = 0,Content_Length = 0;
	char base64buff[HTTP_CLIENT_AUTHB_SIZE + 3]  = {0};
	char b_auth[(HTTP_CLIENT_AUTHB_SIZE + 3) * 4 / 3 + 3] = {0};
	Post_File_Input_Len = 0;
	Post_File_Size = fileSize;
	if(httpSetFlag == 0)
	{
		httpSetFlag = 1;
		//配置PDP上下文
		comSendChk("AT+QHTTPCFG=\"contextid\",1","OK",2000);
		//开启自定义请求头
		comSendChk("AT+QHTTPCFG=\"requestheader\",1","OK",2000);
		//开启输出响应头
		comSendChk("AT+QHTTPCFG=\"responseheader\",0","OK",2000);
		//HTTPS选择 sslctxid
		comSendChk("AT+QHTTPCFG=\"sslctxid\",1","OK",2000);
		//TLS 1.2
		comSendChk("AT+QSSLCFG=\"sslversion\",1,3","OK",2000);
		//支持所有加密套件
		comSendChk("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF","OK",2000);
		//认证模式：不进行验证
		comSendChk("AT+QSSLCFG=\"seclevel\",1,0","OK",2000);
	}
	//设置要访问的URL
	memset(ec20SendBuf,0,sizeof(ec20SendBuf));
	host = strstr(url,"://") + 3;
	uri = strstr(host,"/");
	if(uri == 0)
	{
		uri =url + strlen(url);
	}
	sprintf(ec20SendBuf,"AT+QHTTPURL=%d,80",uri - url);	
	if(comSendChk(ec20SendBuf,"CONNECT",2000) == 1)
	{
		comSendBuf(GPRS_COM,(uint8_t*)url,uri - url);
		//设置http头信息
		headIdx = 0;
		memset(ec20SendBuf,0,sizeof(ec20SendBuf));
		headIdx += sprintf(&ec20SendBuf[headIdx],"POST %s HTTP/1.1\r\n",uri);
		headIdx += sprintf(&ec20SendBuf[headIdx],"Host: %.*s\r\n",uri - host,host);
//		headIdx += sprintf(&ec20SendBuf[headIdx],"Host: ");
//		while(host < uri)
//		{
//			ec20SendBuf[headIdx++] = *host;
//			host++;
//		}
//		headIdx += sprintf(&ec20SendBuf[headIdx],"\r\n");
		if(username != NULL)
		{
			sprintf(base64buff,"%s:%s",username,password);
			Chip_Ec20_base64enc(b_auth,base64buff);
			headIdx += sprintf(&ec20SendBuf[headIdx],"Authorization: Basic %s\r\n",b_auth);
		}
		headIdx += sprintf(&ec20SendBuf[headIdx],"Content-Type: multipart/form-data; boundary=%s\r\n",boundary);
		Content_Length = strlen(boundary) * 2 + 12 + 58 + strlen(fileName) + 42;
		headIdx += sprintf(&ec20SendBuf[headIdx],"Content-Length: %d\r\n\r\n",Content_Length + fileSize);
		
		//Content
		headIdx += sprintf(&ec20SendBuf[headIdx],"--%s\r\n",boundary);																														//长度4+boundary长度
		headIdx += sprintf(&ec20SendBuf[headIdx],"Content-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\n",fileName);	//长度58+文件名长度
		headIdx += sprintf(&ec20SendBuf[headIdx],"Content-Type: application/octet-stream\r\n\r\n");																//长度42
		//POST启动
		memset(cmd,0,sizeof(cmd));
		sprintf(cmd,"AT+QHTTPPOST=%d,80,80",headIdx + fileSize + strlen(boundary) + 8);
		Bsp_Delay_ms(100);
		if(comSendChk(cmd,"CONNECT",2000) == 1)		
		{
			comSendBuf(GPRS_COM,(uint8_t*)ec20SendBuf,headIdx);
			return 1;
		}
	}
	return 0;
}

uint32_t Chip_Ec20Http_Post_File_Input(void* buff,uint32_t len)
{
	if(Post_File_Input_Len + len > Post_File_Size)
		len = Post_File_Size - Post_File_Input_Len;
	comSendBuf(GPRS_COM,(uint8_t*)buff,len);
	if(Post_File_Input_Len >= Post_File_Size)
	{
		len = sprintf(ec20SendBuf,"\r\n--%s--\r\n",boundary);																																				//长度6+boundary长度
		comSendBuf(GPRS_COM,(uint8_t*)ec20SendBuf,len);
//		osDelay(1000);
//		comSendChk("AT+QHTTPREAD=80","OK",2000);
		return 0;
	}
	Post_File_Input_Len += len;
	return len;
}
