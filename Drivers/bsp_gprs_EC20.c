#include "bsp_gprs.h"
#include "bsp_uart_fifo.h"
#include "bsp_gprs_EC20.h"

extern TCP_CLIENT_PARA tcpClientPara[6];//TCP客户端参数

static void gprsBuf_clr(void)
{
	memset(gprsRecvBuf,0,sizeof(gprsRecvBuf));
}

/*
*********************************************************************************************************
*	函 数 名: EC20_SendAT
*	功能说明: 向GSM模块发送AT命令。 本函数自动在AT字符串口增加<CR>字符
*	形    参: _Str : AT命令字符串，不包括末尾的回车<CR>. 以字符0结束
*	返 回 值: 无
*********************************************************************************************************
*/
void EC20_SendAT(char *_Cmd)
{
	char cmd[100] = {0};
	sprintf(cmd,"%s\r\n",_Cmd);
	comSendBuf(RS2_COM, (uint8_t *)cmd, strlen(cmd));
}

/*
*********************************************************************************************************
*	函 数 名: EC20_ReadResponse
*	功能说明: 读取SIM800返回应答字符串。该函数根据字符间超时判断结束。 本函数需要紧跟AT命令发送函数。
*	形    参: _pBuf : 存放模块返回的完整字符串
*			  _usBufSize : 缓冲区最大长度
*			 _usTimeOut : 命令执行超时，0表示一直等待. >0 表示超时时间，单位1ms
*	返 回 值: 0 表示错误（超时）  > 0 表示应答的数据长度
*********************************************************************************************************
*/
uint16_t EC20_ReadResponse(char *_pBuf, uint16_t _usBufSize, uint16_t _usTimeOut)
{
	uint16_t ret;
	ret = comGetBufDelay(RS2_COM,_usBufSize,(uint8_t*)_pBuf,_usTimeOut);
	return ret;
}

static uint8_t resetFailCnt = 0;
/*
*********************************************************************************************************
*	函 数 名: SIM800_Reset
*	功能说明: 给SIM800复位
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void EC20_Reset(void)
{
	comClearRxFifo(RS2_COM);
	EC20_SendAT("AT+CFUN=1,1");
	if(EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 2000)==0)
	{
		if(resetFailCnt++ > 20)
		{
//			BSP_Ioctl(PWR_GPRS_IO,OFF);
//			osDelay(100);
//			BSP_Ioctl(PWR_GPRS_IO,ON);
		}
	}
	else
	{
		if(strstr((char*)gprsRecvBuf,"OK")!=NULL)
		{
			resetFailCnt = 0;
		}
	}
}


/*
*********************************************************************************************************
*	函 数 名: SIM800_QueryConnectionStatus
*	功能说明: 查询GPRS TCP/IP连接情况
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
void EC20_QueryConnectionStatus(void)//
{
	EC20_SendAT("AT+QISTATE?");	/* 发送 AT 命令 */
	osDelay(50);
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_GetSignalQuality
*	功能说明: 查询信号强度
*	形    参: 无
*	返 回 值: 信号强度
*********************************************************************************************************
*/
uint8_t EC20_GetSignalQuality (void)
{
	char *pGetPos;
	uint8_t signal = gTerminalState.csq;
	EC20_SendAT("AT+CSQ");	/* 发送 AT 命令 */	
	osDelay(100);
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 20);
	pGetPos = strstr((char*)gprsRecvBuf,"+CSQ: ");
	if(pGetPos!=NULL)
	{
		pGetPos += 6;
		sscanf(pGetPos,"%hhu",&signal);
	}
	return signal;
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_IPClose
*	功能说明: 关闭TCP连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void EC20_IPClose(void)
{
	EC20_SendAT("AT+QIDEACT=1");		
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 5000);
	osDelay(200);
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_linkClose
*	功能说明: 关闭TCP链路
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void EC20_linkClose(uint8_t link)
{
	osDelay(100);
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QICLOSE=%d,1",link);
	EC20_SendAT(ipbuf);	/* 发送 AT 命令 */
	osDelay(100);
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_TCP_Connect
*	功能说明: 创建TCP连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void EC20_TCP_Connect(uint8_t linkIndex,char *domain,uint16_t port)
{
	memset(ipbuf,0,sizeof(ipbuf));
	/* 支持服务器的IP地址或域名 */
	sprintf(ipbuf,"AT+QIOPEN=1,%d,\"TCP\",\"%s\",%d,0,1",linkIndex,domain,port);
	/* 发起TCP连接 */
	EC20_SendAT(ipbuf);
	osDelay(1000);
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_SendData
*	功能说明: GPRS发送数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_SendData(uint8_t linkIndex,uint8_t* wbuf,uint16_t len)//做兼容处理
{
	uint8_t CTRL_Z = 0x1a;
	uint8_t sendCnt = 0;
	uint8_t ret = 2;
	if(linkIndex != TERM_SERVER && gTerminalState.gbDebug == 1)
	{
		//printfData("send:",5,0);
		//printfData(wbuf,len,1);
//		printfData((char*)wbuf,len,0);
	}
	for(sendCnt = 0;sendCnt < 3;sendCnt++)
	{
		memset(ipbuf,0,sizeof(ipbuf));
		sprintf(ipbuf,"AT+QISEND=%d,%d",linkIndex,len);
		EC20_SendAT(ipbuf);
		if(comChkBufDelay(RS2_COM,">",150,0)==1)
		{//检查到">"字符,可发送数据包到串口
			ret = 0;
		}
		comSendBuf(RS2_COM, wbuf, len);	
		comSendBuf(RS2_COM, &CTRL_Z, 1);
		if(ret == 0)
		{
			osDelay(200);//小于200，大数据1秒频率上报容易丢包
			break;
		}
		else
		{
			osDelay(100);
		}
	}
	return ret;	
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_ReadData
*	功能说明: GPRS接收数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint16_t EC20_ReadData()
{
	uint16_t len = 0;	
	uint16_t datalen = 0;
	uint8_t linkIndex = 0;
	/* 获取SIM800数据 */
	memset(gprsRecvBuf,0,sizeof(gprsRecvBuf));
	len = comGetBufDelay(RS2_COM,sizeof(gprsRecvBuf),gprsRecvBuf,5);
	if(len > 0)
	{
		char *pGetPos = (char*)gprsRecvBuf;
		while((pGetPos=strstr(pGetPos,"+QISTATE"))!=NULL)
		{
			uint8_t state;
			pGetPos += 10;
			if(sscanf(pGetPos,"%hhu,\"TCP\",%*[^,],%*d,%*d,%hhd",&linkIndex,&state)==2)
			{
				if(state==2)
				{
					tcpClientPara[linkIndex].linkState = 1;
				}
				else
				{
					sprintf(ipbuf,"AT+QICLOSE=%d,1",linkIndex);
					EC20_SendAT(ipbuf);	/* 发送 AT 命令 */
					tcpClientPara[linkIndex].linkState = 0;
				}
			}
			pGetPos++;
		}
		if((pGetPos = strstr((const char*)gprsRecvBuf,"+QIND: \"csq\""))!=NULL)
		{
			pGetPos += 13;
			sscanf(pGetPos,"%hu",&datalen);
			gTerminalState.csq = datalen;
		}
		//网络报文提取
		pGetPos = (char*)gprsRecvBuf;
		while((pGetPos = memstr(pGetPos,len - (pGetPos - (char*)gprsRecvBuf),"+QIURC: \"recv\""))!=NULL)
		{
			pGetPos += 15;
			if(sscanf(pGetPos,"%hhu,%hu",&linkIndex,&datalen)==2)
			{
				if((pGetPos = strstr(pGetPos,"\r\n"))!=NULL)
				{
					pGetPos += 2;
					if((uint8_t*)pGetPos + datalen <= gprsRecvBuf + len)
					{
						if(tcpClientPara[linkIndex].UnpackData!=NULL)
						{
							if(linkIndex != TERM_SERVER && gTerminalState.gbDebug == 1)
							{
//								printfData("recv:",5,0);
//								printfData(pGetPos,datalen,1);
//								printfData(pGetPos,datalen,0);
							}
							tcpClientPara[linkIndex].UnpackData(linkIndex,(uint8_t*)pGetPos,datalen);
						}
						pGetPos += datalen;
					}
				}
			}
		}
		
		//双链路网络断开检测
		if((pGetPos=strstr((char*)gprsRecvBuf, "\"closed\"")) != NULL)
		{
			pGetPos+=9;
			if(sscanf(pGetPos,"%hhu",&linkIndex)==1)
				reConnect(linkIndex);
		}
		if(strstr((char*)gprsRecvBuf, "+CPIN") != NULL||\
			strstr((char*)gprsRecvBuf, "\"pdpdeact\",1") != NULL)
		{//检查到连接断开
			gprsRedial();				
		}
	}
	return 0;	
}

/*
*********************************************************************************************************
*	函 数 名: SIM800_GetCCID
*	功能说明: 获取CCID
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_GetCCID(uint8_t* rbuf, uint8_t rLen)
{
	char * pGetPos;
	gprsBuf_clr();
	EC20_SendAT("AT+QCCID");	/* 发送 AT 命令 */
	osDelay(200);
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 200);	/* 超时 200ms */
	if((pGetPos=strstr((char*)gprsRecvBuf,"89"))!=NULL)
	{
		memcpy(rbuf,pGetPos,20);
		return 20;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_GetIMEI
*	功能说明: 获取IMEI
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_GetIMEI(uint8_t* rbuf, uint8_t rLen)
{
	int ssRet;
	gprsBuf_clr();
	EC20_SendAT("AT+CGSN");	/* 发送 AT 命令 */
	osDelay(200);
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 100);	/* 超时 200ms */
	ssRet = sscanf((char*)gprsRecvBuf,"%*[^0123456789]%[0123456789]",rbuf);
	if(ssRet == 1)
	{
		return ssRet;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_GetIMSI
*	功能说明: 获取IMSI
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_GetIMSI(uint8_t* rbuf, uint8_t rLen)
{
	int ssRet;
	gprsBuf_clr();
	EC20_SendAT("AT+CIMI");	/* 发送 AT 命令 */
	osDelay(200);
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 100);	/* 超时 200ms */
	ssRet = sscanf((char*)gprsRecvBuf,"%*[^0123456789]%[0123456789]",rbuf);
	if(ssRet == 1)
	{
		return ssRet;
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
uint8_t EC20_GetNetworkMode(void)
{
	gprsBuf_clr();
	EC20_SendAT("AT+QNWINFO");	/* 发送 AT 命令 */
	osDelay(100);
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 200);	/* 超时 200ms */	
  if(strstr((char*)gprsRecvBuf,"GSM")!=NULL)
	{
		return 1;
	}
	else if(strstr((char*)gprsRecvBuf,"WCDMA")!=NULL || strstr((char*)gprsRecvBuf,"TDSCDMA")!=NULL)
	{
		return 2;
	}
	else if(strstr((char*)gprsRecvBuf,"LTE")!=NULL)
	{
		return 3;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_SIMCardStatus
*	功能说明: 查询SIM卡状态，打开模块GPS功能
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
uint8_t EC20_SIMCardStatus(void)
{
	/*
		AT+CPIN?
		+CPIN: READY
		OK				
	*/
	int len;
	
	comClearRxFifo(RS2_COM);	/* 清零串口接收缓冲区 */	
	gprsBuf_clr();
	
	EC20_SendAT("AT+CPIN?");	/* 发送 AT 命令 */
	osDelay(1300);
	len = EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 20);	/* 超时 200ms */
	if (len == 0)
	{
		return 0;		
	}
	if(strstr((char*)gprsRecvBuf,"READY") !=NULL)   /* ME is not pending for any password */
	{
		return 1;
	}
	else
		return 2;//SIM卡异常
}

/*
*********************************************************************************************************
*	函 数 名: EC20_GetGPRSNetStatus
*	功能说明: 查询网络状态
*	形    参: 无
*	返 回 值: 网络状态
*********************************************************************************************************
*/
uint8_t EC20_GetGPRSNetStatus(void)
{
	/*
		AT+CGATT?
		+CGATT: 1
		
		OK				
	*/
	uint16_t len;
	uint8_t status = 0;
	
	gprsBuf_clr();
	
	
	EC20_SendAT("AT+CMEE=2");	/* 发送 AT 命令 */
	osDelay(100);
	len = EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 200);	/* 超时 200ms */	
	
	EC20_SendAT("AT+CGATT=1");	/* 发送 AT 命令 */
	osDelay(1100);
	len = EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 200);	/* 超时 200ms */
	
	EC20_SendAT("AT+CGATT?");	/* 发送 AT 命令 */
	osDelay(1100);
	len = EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 200);	/* 超时 200ms */
	if (len == 0)
	{
		return 0;		
	}
	
	else
	{
		char* pGetPos = strstr((char*)gprsRecvBuf,"+CGATT: ");
		if(pGetPos!=NULL)
		{
			sscanf(pGetPos,"+CGATT: %hhu",&status);
			return status;
		}
	}
	return 2;  /* 无SIM卡或异常 */
}

/*
*********************************************************************************************************
*	函 数 名: EC20_OpenGPS
*	功能说明: EC20开启GPS
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_OpenGPS(void)
{
	osDelay(100);
	EC20_SendAT("AT+QGPS=1");	/* 发送 AT 命令 */
	osDelay(500);
	EC20_SendAT("AT+QGPSCFG=\"gpsnmeatype\",3");	/* 发送 AT 命令 */
	osDelay(100);		
	EC20_SendAT("AT+QGPSCFG=\"outport\",\"uartdebug\"");	/* 发送 AT 命令 */
	osDelay(100);
	return 1;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_CloseGPS
*	功能说明: EC20关闭GPS
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_CloseGPS(void)
{
	osDelay(1000);
	EC20_SendAT("AT+QGPSEND");	/* 发送 AT 命令 */
	osDelay(100);
	return 1;
}

/*
*********************************************************************************************************
*	函 数 名: EC20_OpenSleepModel
*	功能说明: EC20开启休眠模式
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_OpenSleepModel(void)
{
	osDelay(1000);
	EC20_SendAT("AT+QSCLK=1");	/* 发送 AT 命令 */
	osDelay(300);
	return 1;
}


/*
*********************************************************************************************************
*	函 数 名: EC20_PppdCallGPRS
*	功能说明: GPRS拨号连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t EC20_PppdCallGPRS(void)
{
	uint8_t rtn = 0;
	char* pGetPos;
	gprsBuf_clr();
	
	/* 命令不回显 */
	EC20_SendAT("ATE0");
	osDelay(10);
	
	/* URC输出口 */
	EC20_SendAT("AT+QURCCFG=\"urcport\",\"uart1\"");
	osDelay(10);

	/* CSQ自动上上报 */
	//EC20_SendAT("AT+QINDCFG=\"csq\",1,0");
	osDelay(10);
	
	/* 设置APN */
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QICSGP=1,1,\"%s\",\"\",\"\",1","CMNET");
	EC20_SendAT(ipbuf);
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 1000);
	osDelay(100);
	
	/* GPRS拨号 */
	EC20_SendAT("AT+QIACT=1");
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 2000);
	osDelay(100);
	
	/* 获取本地地址 */
	EC20_SendAT("AT+QIACT?");
	osDelay(2000);
	memset(gprsRecvBuf, 0, sizeof(gprsRecvBuf));
	EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 100);
	pGetPos = strstr((char*)gprsRecvBuf,"\"");
	if(sscanf(pGetPos,"\"%*hhu.%*hhu.%*hhu.%hhu",&rtn)==1)
	{
		rtn = 1;
	}
	else
	{
		rtn = 0;
	}
	return rtn;
}


uint8_t EC20_ftpDownload(void)
{
	uint32_t ftp_FileLength = 0;
	uint32_t try_cnt = 0;
	uint8_t ucData = 0,ftpFileHead=0,isDbcFile = 0;
	char* pGetPos;
	int	ftpFileSize = 0,bufPosition = 0;

	osDelay(3000);
	EC20_SendAT("AT+QIACT=1");
	osDelay(3000);		
	
	EC20_SendAT("AT+QFTPCFG=\"contextid\",1");
	osDelay(3000);			
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QFTPCFG=\"account\",\"%s\",\"%s\"",gFtpParas.ftpUser,gFtpParas.ftpPwd);
	EC20_SendAT(ipbuf);
	osDelay(2000);
	
	EC20_SendAT("AT+QFTPCFG=\"filetype\",0");
	osDelay(2000);
	
	EC20_SendAT("AT+QFTPCFG=\"transmode\",1");
	osDelay(2000);
	
	EC20_SendAT("AT+QFTPCFG=\"rsptimeout\",90");
	osDelay(3000);
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QFTPOPEN=\"%s\",%d",gFtpParas.ftpUrl,gFtpParas.ftpPort);
	for(try_cnt=0;try_cnt<5;try_cnt++)
	{
		EC20_SendAT(ipbuf);
		if(comChkBufDelay(RS2_COM,"+QFTPOPEN: 0,0",10000,0)==1)
		{
			break;
		}
	}
	osDelay(1000);
	try_cnt = 0;
	//DBC文件
	if(strstr(gFtpParas.ftpName,".dbc") != NULL)
	{
		isDbcFile = 1;
	}
	else
	{
		strcpy(gFtpParas.ftpName,BINFILE_PATH);
	}
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QFTPCWD=\"/%s\"",gFtpParas.ftpDir);
	for(try_cnt=0;try_cnt<3;try_cnt++)
	{
		EC20_SendAT(ipbuf);
		if(comChkBufDelay(RS2_COM,"+QFTPCWD: 0,0",10000,0)==1)
		{
			break;
		}
	}
	osDelay(1000);
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QFTPNLST=\"/%s\"",gFtpParas.ftpDir);		
	for(try_cnt=0;try_cnt<3;try_cnt++)
	{
		EC20_SendAT(ipbuf);
		if(comChkBufDelay(RS2_COM,"+QFTPNLST: 0",10000,0)==1)
		{
			break;
		}
	}
	osDelay(1000);
	/*+QFTPSIZE: 0,110621*/
	memset(ipbuf,0,sizeof(ipbuf));
	sprintf(ipbuf,"AT+QFTPSIZE=\"%s\"",gFtpParas.ftpName);	
	for(try_cnt=0;try_cnt<3;try_cnt++)
	{
		EC20_SendAT(ipbuf);
		if(comChkBufDelay(RS2_COM,"+QFTPSIZE: 0",10000,0)==1)
		{
			EC20_ReadResponse((char*)gprsRecvBuf, sizeof(gprsRecvBuf), 200);	
			pGetPos=strstr((char*)gprsRecvBuf,"+QFTPSIZE: 0");
			pGetPos += 13;
			sscanf(pGetPos,"%d",&ftpFileSize);
			if(ftpFileSize > 0 && ftpFileSize < FTP_FILE_MAX_SIZE)
			{
				break;
			}
		}
		if(try_cnt >= 2)
		{
			BSP_Ioctl(PWR_GPRS_IO,OFF);
			osDelay(100);
			BSP_Ioctl(PWR_GPRS_IO,ON);			
			BoardReset();
		}
	}
	comClearRxFifo(RS2_COM);
	ftpFileHead = 2;
	while(1)
	{
		if(ftpFileHead==2)
		{//获取命令
			int size = ftpFileSize - ftp_FileLength;
			if(size>2048)
				size = 2048;
			osDelay(1);
			memset(ipbuf,0,sizeof(ipbuf));
			sprintf(ipbuf,"AT+QFTPGET=\"%s\",\"COM:\",%d,%d",gFtpParas.ftpName,ftp_FileLength,size);		
			EC20_SendAT(ipbuf);//获取FTP文件
			ftpFileHead = 0;
		}
		if(comGetChar(RS2_COM,&ucData))
		{
			try_cnt = 0;
			szMainBuf[bufPosition++]=ucData;
			if(ftpFileHead == 0)
			{//文件流开始标志
				if((pGetPos =strstr((char*)szMainBuf,"CONNECT\r\n"))!=NULL)	//FTP文件流开始
				{
					bufPosition = 0;
					ftpFileHead = 1;
					memset(szMainBuf,0,sizeof(szMainBuf));
				}
			}
			else if(ftpFileHead==1)
			{
				if((bufPosition == 2048 && ftp_FileLength + bufPosition < ftpFileSize)||ftp_FileLength + bufPosition == ftpFileSize)
				{//正在下载中
					/* 写入FLASH */
					if(isDbcFile == 0)
					{
						Flash_Write(FTP_FILE_SAVE_ADDR+ftp_FileLength,szMainBuf,0x800);
					}
					else
					{
						Flash_Write(DBC_FILE_FLAG_ADDR+ftp_FileLength,szMainBuf,0x800);
					}
					ftp_FileLength += bufPosition;
					bufPosition = 0;
					memset(szMainBuf,0,sizeof(szMainBuf));
					if(ftp_FileLength < ftpFileSize)
					{//文件没有下载完成
						ftpFileHead = 2;
					}
				}
				if(ftp_FileLength==ftpFileSize)//写flash实际大小等于文件实际大小
				{//下载完成，且收到结束标志
					//sscanf(pGetPos,"+QFTPGET: 0,%d",&recvFileSize);//模块接收文件大小
					if(isDbcFile == 0 && ftp_FileLength == ftpFileSize)//bin文件需要校验，写flash实际大小等于文件实际大小
					{
						if(updateFile_Crc(ftp_FileLength) == 1)								//累和校验
						{
							fileTagSave.newFile_Flag = 0xAA55;		//升级标志
							fileTagSave.fileLength = ftp_FileLength;										
							Flash_Write(FW_UPDATE_FLAG_ADDR,(uint8_t*)&fileTagSave,0x800);
						}
					}
					BoardReset();
				}
			}
		}
		else
		{
			++try_cnt;
			if(ftpFileHead==0)
			{
				osDelay(1);
				if(try_cnt > 60000)//60秒没有下载完成
					BoardReset();
			}
			else
			{
				if(try_cnt%20==0)
					osDelay(1);
				if(try_cnt > 200000)//10秒没有数据
					BoardReset();				
			}
		}
	}
}
