#include "bsp_gprs.h"
#include "bsp_io.h"
#include "bsp_uart_fifo.h"
#include "bsp_gprs_EC20.h"
#include "cmsis_os2.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "stdio.h"

static const osMutexAttr_t mutex = { NULL, osMutexRecursive|osMutexPrioInherit, NULL, 0 };//����ʹ�û����źţ���ֹ���߳�ͬʱ����
static osMutexId_t mutexId;

static uint8_t gprs_abort = 0;					                                			//GPRS�Ͽ���ʶ
static uint8_t isFirstLoad = 1;					                                			//GPRS������ʼ��
static uint8_t getIccidFlag = 0;																							//��ȡICCID��ʶ
uint8_t gprsRecvBuf[2048]; 																										//�������ݻ�����
char ipbuf[100];												                                			//ģ�鷢�ͻ�����
static char iccid[20];								                                				//ICCID
                    			
TCP_CLIENT_PARA tcpClientPara[MAX_LINK];																			//TCP�ͻ��˲���

static void simInit(void);																										//оƬ��ʼ��
static uint8_t simGetCCID2buf(char* rbuf, uint8_t rLen);										//��ȡICCID���ֵ�ICCID������
static uint8_t simGetIMEI(char* rbuf, uint8_t rLen);
static uint8_t simGetIMSI(char* rbuf, uint8_t rLen);
static uint8_t simGetNetworkMode(void);
static uint16_t simReadData(void);																						//��ȡ���������
static uint8_t chkSimModel(void);

uint8_t gFtp_Mode = 0;
uint8_t gprs_State = 0;
FTP_PARS gFtpParas;

/* GPRS�Ͽ���־ */
void gprsRedial(void)
{
	uint8_t i;
	for(i=0;i<MAX_LINK;i++)
	{
		tcpClientPara[i].linkState = 0;
	}
	simIPClose();
	gprs_abort = 1;   /* GPRS�Ͽ� */
	comClearRxFifo(RS2_COM);
}

/* GPRS��·�Ͽ����� */
void reConnect(uint8_t link)
{
	if(link < MAX_LINK)
	{
		simLinkClose(link);
		tcpClientPara[link].linkState = 0;   /* GPRS�Ͽ� */
	}
}

uint8_t getNAckTm(uint8_t link)
{
	return tcpClientPara[link].nAckTm;
}


void ftpUpdateApp(void)
{
	gFtp_Mode = 1;   /* FTP����ģʽ */
}

/*
�����·״̬
����ֵ 0��ȫ����·δ����  1��ȫ����·�������� 2��������·����
*/
static uint8_t checktcpClientState()
{
	uint8_t i;
	uint8_t state1 = 0,state2 = 0 ,ret = 0;
	for(i=0;i<MAX_LINK;i++)
	{
		if(tcpClientPara[i].used == 1 && tcpClientPara[i].linkState == 1)
		{
			//��⵽��������·
			state1 = 1;
		}
		if(tcpClientPara[i].used == 1 && tcpClientPara[i].linkState == 0)
		{
			//��⵽��·������
			state2 = 1;
		}
	}
	if(state1==0)
	{
		//������·�쳣
		ret = 0;
		gprs_State = 1;
	}
	else if(state2 == 0)
	{
		//������·����
		ret = 1;
		gprs_State = 3;
	}
	else
	{
		//������·�쳣
		ret = 2;
		gprs_State = 2;
	}
	return ret;
}

static int16_t ConnectInterval = -1;			//�������ڼ���
static uint16_t ConnectCnt = 0;						//���Ӵ�������
static uint8_t st = 0;
static int8_t sttmp = -1;
static uint8_t pppdCallCnt = 0;   //���Ŵ���������10�Σ��������ն�
static uint8_t getSimFaultCnt = 0;//��ȡSIM��״̬ʧ�ܼ��㣬�������10��û�����ݣ���תGPRS����
static uint8_t getSimModeled = 0; //��ȡ��SIMģ���ʶ

void gprsPppdask(void)
{
		int8_t ret = 0, i = 0;	
		if(1 == isFirstLoad )
		{
			/* ���������ź��� */
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
	
		if(st == 0)      /* 1.��ʼ����ǰ׼�������SIM��״̬�����ģ����GPS�������GPS */
		{
			gTerminalState.networkMode = 0;
			if(getSimModeled == 1)
			{
				//ģ����������������ָ��
				simOpenSleepModel();						//EC20����˯��ģʽ

				//ģ����������ʼ���
				ret = simGetSIMCardStatus();//���SIM��,��ʱ1��
				if(2 == ret)//��SIM���쳣
				{
					gTerminalState.simCardState = 0;
				}
				else if(1 == ret)
				{
					gTerminalState.simCardState = 1;
					getSimFaultCnt = 0;
					//��⵽SIM��
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
				//ģ����
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
		
		else if(st==1) /*�������״̬*/
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
		
		else if(st == 2)  /* 2.PPPD�������ӷ����� */
		{
			if(1 == simPppdCallGPRS())
			{//�κųɹ�
				gTerminalState.csq = simGetSignalQuality();
				osDelay(100);
				gTerminalState.networkMode = simGetNetworkMode();
				pppdCallCnt = 0;
				st = 3;
				gprs_State = 1;
				ConnectCnt = 0;
				for(i=0;i<MAX_LINK;i++)
				{
					//Ĭ������ʱ��5��
					tcpClientPara[i].linkState = 0;
					tcpClientPara[i].connectCnt = 25;
					tcpClientPara[i].connectHZ = 25;
				}
			  comClearRxFifo(RS2_COM);	/* ���㴮�ڽ��ջ����� */
			}
			else
			{
				if(++pppdCallCnt > 50)
				{//2�벦��û�гɹ�
					st = 0;
					pppdCallCnt = 0;
					simReset();
				}
			}
		}
		else if(st == 3) /* 4.�ж�GPRS����״̬����ʱ׼���ز� */
		{
			if(++ConnectInterval >= 450)			//90�룬��ѯһ��
			{
				ConnectInterval = 0;
				//��ѯ����״̬
				simReadData();						 //�ȶ�ȡ�������ٷ�������
				simQueryConnectionStatus();//���������ѯ����
				simReadData();						 //��ȡ���ݺ�����״̬
			}
			if(checktcpClientState()!=1)
			{	//����·������
				for(i=0;i<MAX_LINK;i++)
				{
					if(tcpClientPara[i].linkState == 0 && tcpClientPara[i].used==1)//��·��ʹ�ã��Ҳ�����
					{
						if(++tcpClientPara[i].connectCnt >= tcpClientPara[i].connectHZ)//��·����ʱ�䵽
						{
							if(tcpClientPara[i].connectCnt == 300)//1����δ����
							{
								simLinkClose(i);
							}
							tcpClientPara[i].connectCnt = 0;
							if(tcpClientPara[i].connectHZ <= 450)//�´�����ʱ���ӳ�5�룬�90��
							{
								tcpClientPara[i].connectHZ += 25;
							}
							simTcpConnect(i,tcpClientPara[i].address,tcpClientPara[i].port);
							simReadData();
							if(ConnectInterval < 435)//450��ʱ���ѯ��Ԥ��15����3��
								ConnectInterval = 435;
						}
					}
				}
			}
			simReadData();
			if(checktcpClientState()==0)
			{
				//ȫ��δ����,��������6��
				ConnectCnt++;//�ز���ʱ
				if(ConnectCnt >= 90)
				{//����3����������
					gprs_abort = 1;
					ConnectCnt = 0;
				}
			}
			else
			{
				//��һ����·���ӳɹ�
				ConnectCnt = 0;
			}
			if(gprs_abort == 1 && gFtp_Mode == 0)
			{
				/* GPRS�Ͽ��󣬸�λģ�� ���½��벦������ */
				gprs_abort = 0;
				simReset();
				st = 0;
			}
		}
		if(gFtp_Mode == 1)//��������
		{
			//�ر�����
			USART_Cmd(USART3,DISABLE);	//�ر�GPS����
			BSP_Ioctl(CAN_STB_ID,OFF);	//CAN˯��
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
*	�� �� ��: simReset
*	����˵��: ģ����
*	��    ��: ��
*	�� �� ֵ: ģ����ɹ�
*********************************************************************************************************
*/
static uint8_t chkSimModel(void)
{
	uint8_t ret = 0;
	memset(gprsRecvBuf,0,sizeof(gprsRecvBuf));
	comSendBuf(RS2_COM,(uint8_t*)"ATI\r\n",5);	/* ���� AT ���� */	
	osDelay(200);
	if(comGetBufDelay(RS2_COM,sizeof(gprsRecvBuf),(uint8_t*)gprsRecvBuf,200)>0)	/* ��ʱ 200ms */
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
*	�� �� ��: bsp_InitGprs
*	����˵��: SIMģ���ʼ��
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: simReset
*	����˵��: ��SIMģ�鸴λ
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: simGetSIMCardStatus
*	����˵��: ��ѯSIM��״̬
*	��    ��: ��
*	�� �� ֵ: SIM��״̬
*********************************************************************************************************
*/
uint8_t simGetSIMCardStatus(void)
{
	return EC20_SIMCardStatus();
}

/*
*********************************************************************************************************
*	�� �� ��: simQueryLinkStatus
*	����˵��: ��ѯGPRS ��·�������
*	��    ��: ��
*	�� �� ֵ: ����״̬
*********************************************************************************************************
*/
uint8_t simQueryLinkStatus(uint8_t link)
{
	if(tcpClientPara[link].linkState == 1)//�����������Ͽ�����ʱ��3��
	{
		tcpClientPara[link].connectCnt = 10;
		tcpClientPara[link].connectHZ = 25;
	}
	//������������
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
//	//���Զ�ʱ����
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
*	�� �� ��: simQueryConnectionStatus
*	����˵��: ��ѯGPRS TCP/IP�������
*	��    ��: ��
*	�� �� ֵ: ����״̬
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
*	�� �� ��: simGetSignalQuality
*	����˵��: ��ѯ�ź�ǿ��
*	��    ��: ��
*	�� �� ֵ: �ź�ǿ��
*********************************************************************************************************
*/
uint8_t simGetSignalQuality (void)
{
	return EC20_GetSignalQuality();
}
/*
*********************************************************************************************************
*	�� �� ��: simIPClose
*	����˵��: �ر�TCP����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void simIPClose(void)
{
	EC20_IPClose();
}

/*
*********************************************************************************************************
*	�� �� ��: simLinkClose
*	����˵��: �ر�TCP��·
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void simLinkClose(uint8_t link)
{
	EC20_linkClose(link);
}

/*
*********************************************************************************************************
*	�� �� ��: simTcpConnect
*	����˵��: ����TCP����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: simPppdCallGPRS
*	����˵��: GPRS��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t simPppdCallGPRS(void)
{
	return EC20_PppdCallGPRS();
}

/*
*********************************************************************************************************
*	�� �� ��: simOpenSleepModel
*	����˵��: ����˯��ģʽ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t simOpenSleepModel(void)
{
	return EC20_OpenSleepModel();
}

/*
*********************************************************************************************************
*	�� �� ��: simFtpUpdate
*	����˵��: ftp����ʱ���ݽ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t simFtpUpdate(void)
{
	return EC20_ftpDownload();	
}

/*
*********************************************************************************************************
*	�� �� ��: simSendData
*	����˵��: GPRS��������
*	��    ��: ��
*	�� �� ֵ: 2��������� 0�����ͳɹ� 1������ʧ��
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
	{//�������,����
		reConnect(linkIndex);
	}
	return ret;
}

/*
*********************************************************************************************************
*	�� �� ��: simReadData
*	����˵��: GPRS��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint16_t simReadData(void)
{
	return EC20_ReadData();
}

/*
*********************************************************************************************************
*	�� �� ��: simReadData
*	����˵��: GPRS��������
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: simGetCCID
*	����˵��: ��ȡCCID
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: simGetCCID2buf
*	����˵��: ��ȡCCID��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t simGetCCID2buf(char* rbuf, uint8_t rLen)
{
	return EC20_GetCCID(rbuf,rLen);
}

/*
*********************************************************************************************************
*	�� �� ��: simGetCCID2buf
*	����˵��: ��ȡCCID��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t simGetIMEI(char* rbuf, uint8_t rLen)
{
	return EC20_GetIMEI(rbuf,rLen);
}

/*
*********************************************************************************************************
*	�� �� ��: simGetCCID2buf
*	����˵��: ��ȡCCID��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t simGetIMSI(char* rbuf, uint8_t rLen)
{
	return EC20_GetIMSI(rbuf,rLen);
}

/*
*********************************************************************************************************
*	�� �� ��: simGetGPRSNetStatus
*	����˵��: GPRS��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t simGetGPRSNetStatus(void)
{
	return EC20_GetGPRSNetStatus();
}

