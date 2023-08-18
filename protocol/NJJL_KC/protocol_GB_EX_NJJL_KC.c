/*
文 件：protocol_GB_EX_NJJL_KC.c
功 能：南京金通信协议 - 解析上送自定义扩展数据
日 期: 2021/12/30
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/

#include "NJJL_KC/protocol_GB_EX_NJJL_KC.h"

#define MAX_YZT_LINK 1
uint8_t sendUseVIN_TermID = 1;		//1：默认使用终端ID上送 	0：使用VIN上送
uint8_t sendSelfData = 1;					//1：默认上送自定义数据 	0：不上送自定义数据

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;										//是否使用
	char *vin;											//车架号
	uint8_t sendOverTimeCnt;				//发送次数
	//内部时间戳
	uint32_t lockStaStamp;					//锁状态时间戳
	//其他参数
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffLen;								//发送数据缓冲区长度
}GBSTA;


static GBSTA gbSta[MAX_YZT_LINK] = {0};
USER_DATA gUserDara;												/* 远程锁车参数 */
static uint32_t reConnectStamp = 0;					//重播计时

static uint8_t lockResponCout = 0;					//锁车响应次数

/*
功能：存储用户数据
		存储锁车指令相关数据
*/
void saveUserData(void)
{
		gUserDara.store_flag = 0xAA;
		User_ID_Save(&gUserDara,sizeof(USER_DATA));				
}


//添加数据报头及校验,返回值，
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
功能：打包车辆控制状态反馈
*/
static uint16_t packCtrlStaFeedBack(uint8_t link,char* vin,uint8_t* buff,uint16_t maxLen)
{
	buff[24] = (uint8_t)(g_system_dt.year - 2000);
	buff[25] = (uint8_t)(g_system_dt.month);
	buff[26] = (uint8_t)(g_system_dt.day);
	buff[27] = (uint8_t)(g_system_dt.hour);
	buff[28] = (uint8_t)(g_system_dt.minute);
	buff[29] = (uint8_t)(g_system_dt.second);
	buff[30] = (uint8_t)(gUserDara.carDoorCtrCount >> 8);         	//控制流水号
	buff[31] = (uint8_t)(gUserDara.carDoorCtrCount >> 0);
	buff[32] = gUserDara.carDoorCtrRspCode;
	return MakeCmd(CMD_LOCKCARRRSP,0xFE,vin,buff,9);
}

/*
功能：解析锁车指令
*/
static uint8_t UnpackLockCMD(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gUserDara.carDoorCtrCount = rbuf[30] << 8 | rbuf[31];			//流水号
	gUserDara.carDoorCtrCode = rbuf[32];											//锁车控制指令
	gUserDara.carDoorCtrRspCode = 0;													//锁车响应代码
	gUserDara.isReturnLockState = 0xAA;													
	
	sendLockCMDSign = 5;																			//发送锁车指令开关
	
	//保存COMMON_PARA
	saveUserData();
	
	DataAreaLen = MakeCmd(CMD_LOCKCAR,replyFlag,vin,ResponCtrlBuf,DataAreaLen);
	Fun_Gprs_Tcp_send(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}


/*
功能：扩展协议初始化
描述：返回配置状态参数
*/
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	
	/* 读取用户参数 */
	if(gUserDara.store_flag != 0xAA)
	{
		User_ID_Read(&gUserDara,sizeof(USER_DATA));		
		gUserDara.store_flag = 0xAA;		
	}
	
	for(i = 0;i < MAX_YZT_LINK;i++)
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
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].buff = buff;
	gbSta[objLinkIdx].buffLen = buffLen;
	//内部
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].lockStaStamp = 0;
	
	return &gbSta[objLinkIdx];
}

/*
功能：扩展协议运行   
描述：ctrl:0 网络离线 1:网络在线 2:睡眠 
			sta:0 国标未登入 1:国标已登入 
			返回值:0 扩展协议未登入 1:扩展协议已登入
*/
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta)
{
	GBSTA *pSta = obj;
	uint16_t dataLen = 0;
	if(pSta == NULL)//云智通链路未配置，返回运行成功
	{
		return 1;
	}
	else if(ctrl == 0 || ctrl == 2)//掉线或睡眠
	{
		return 0;
	}
	if(pSta->sendOverTimeCnt >= 3 &&  osKernelGetTickCount() - reConnectStamp >= 30000)
	{
		reConnectStamp = osKernelGetTickCount();
		pSta->sendOverTimeCnt = 0;
		Fun_Gprs_Tcp_disconnect(pSta->bLink);					//连续3次超3秒响应超时,重播
	}
	//上报锁车状态
	if(osKernelGetTickCount() - pSta->lockStaStamp >= 3000 && gUserDara.isReturnLockState == 0xA5)//控制状态回复
	{
		pSta->lockStaStamp = osKernelGetTickCount();
		lockResponCout++;
		if(gUserDara.carDoorCtrRspCode == 1 || lockResponCout > 10)			//收到锁车状态反馈，或者等待30秒
		{
			if((dataLen = packCtrlStaFeedBack(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
			{
				Fun_Gprs_Tcp_send(pSta->bLink,pSta->buff,dataLen);
			
				gUserDara.isReturnLockState = 0xAA;
				lockResponCout = 0;
				saveUserData();
			}
		}
	}
	return 1;
}


/*
功能：扩展协议接收解包
*/
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen)
{
	uint8_t i,ret = 1;
	GBSTA* pSta = NULL;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bLink == link && gbSta[i].bUse == 1)
		{
			pSta = &gbSta[i];
			break;
		}
	}
	if(pSta == NULL)//未配置此链路
		return 0;
	switch(szRecvBuf[2])
	{
		case CMD_LOCKCAR:		//接收锁车指令
		{
			pSta->lockStaStamp = 0;
			UnpackLockCMD(link,pSta->vin,szRecvBuf,rLen);
		}
		break;
		default: ret = 0;break;
	}
	return ret;
}

/*
功能：打包B0数据
*/
uint16_t PackB0Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	bPackBuf[0] = 0xB0;
	SelfDataB0*  pB0Data = (SelfDataB0*)&pRealData->externData[selfDataB0Pos];		
	
	//填充自定义数据
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 24);											//累计消耗电量		
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 16);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 8);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsOutTotalEnergy >> 0);
	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 24);											//累计充电能量		
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 16);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 8);	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsChgTotalEnergy >> 0);
	
	bPackBuf[PackLen++] = pB0Data->sBatsBreakOnceEnergy;																			//单次制动回馈能量
	bPackBuf[PackLen++] = pB0Data->sSOH;																											//SOH
	
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsPower >> 8);
	bPackBuf[PackLen++] = (uint8_t)(pB0Data->sBatsPower >> 0);																//电池功率																						//预留5			
	
	//填充长度
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
功能：打包B2数据
*/
uint16_t PackB2Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	
	bPackBuf[0] = 0xB2;
	SelfDataB2*  pB2Data = (SelfDataB2*)&pRealData->externData[selfDataB2Pos];		
	//填充自定义数据
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE1;											//sBMSDTC_CODE1
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE2;											//sBMSDTC_CODE2
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE3;											//sBMSDTC_CODE3
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE4;											//sBMSDTC_CODE4
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE5;											//sBMSDTC_CODE5
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE6;											//sBMSDTC_CODE6
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE7;											//sBMSDTC_CODE7
	bPackBuf[PackLen++] = pB2Data->sBMSDTC_CODE8;											//sBMSDTC_CODE8

	bPackBuf[PackLen++] = pB2Data->sBMSCode1;													//sBMSCode1
	bPackBuf[PackLen++] = pB2Data->sBMSCode2;													//sBMSCode2

	bPackBuf[PackLen++] = pB2Data->sDTC_Code;													//sDTC_Code		

	//填充长度
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;
}

/*
功能：打包B3数据
*/
uint16_t PackB3Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	bPackBuf[0] = 0xB3;
	SelfDataB3*  pB3Data = (SelfDataB3*)&pRealData->externData[selfDataB3Pos];		
	//填充自定义数据	
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_1_8;						//1-8		从控在线状态
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_9_16;					//9-16	从控在线状态
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_17_24;					//17-24	从控在线状态
	bPackBuf[PackLen++] = pB3Data->sSlaveControlOnLine_25_32;					//25_32	从控在线状态
	
	bPackBuf[PackLen++] = pB3Data->sBatsCount;												//电池箱体数
	bPackBuf[PackLen++] = pB3Data->sBatSlaveControlCount;							//从控数

	//填充长度
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;
}

/*
功能：打包B4数据
*/
uint16_t PackB4Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	uint8_t usTems = 0;
	
	bPackBuf[0] = 0xB4;
	SelfDataB4*  pB4Data = (SelfDataB4*)&pRealData->externData[selfDataB4Pos];		
	//填充自定义数据
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sTCU_FaultCode >> 8);				//TCU故障代码
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sTCU_FaultCode >> 0);				

	bPackBuf[PackLen++] = pB4Data->sTCU_OtherFaultCode;										//TCU其他故障代码
	bPackBuf[PackLen++] = pB4Data->sTCU_FaultLevel;												//TCU故障等级
	bPackBuf[PackLen++] = pB4Data->sMoter_FaultLevel;											//驱动电机故障等级

	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMoter_DTCCode >> 8);				//驱动电机DTC_CODE
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMoter_DTCCode >> 0);				

	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMotor_FaultCount >> 8);			//驱动电机故障总数
	bPackBuf[PackLen++] = (uint8_t)(pB4Data->sMotor_FaultCount >> 0);				

	usTems = ((pB4Data->sMCU_TempAlarm & 0x03)<<0);												//MCU温度报警
	usTems |= ((pB4Data->sMotor_TempAlarm & 0x03)<<2);										//驱动电机温度报警
	bPackBuf[PackLen++] = usTems;

	//填充长度
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
功能：打包B5数据
*/
uint16_t PackB5Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	bPackBuf[0] = 0xB5;
	SelfDataB5*  pB5Data = (SelfDataB5*)&pRealData->externData[selfDataB5Pos];		
	//填充自定义数据
	bPackBuf[PackLen++] = pB5Data->sCarFaultLevel;											//整车故障等级
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum1;												//整车故障码1
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum2;												//整车故障码2
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum3;												//整车故障码3
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum4;												//整车故障码4
	bPackBuf[PackLen++] = pB5Data->sCarFaultNum5;												//整车故障码5	

	//填充长度
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
功能：打包BF数据
*/
uint16_t PackBFData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 3;
	uint8_t usTems = 0;

	bPackBuf[0] = 0xBF;
	SelfDataBF*  pBFData = (SelfDataBF*)&pRealData->externData[selfDataBFPos];		
	//填充自定义数据
	usTems = 0;
	usTems |= (pBFData->sCE_LockST<<0);								//换电锁状态
	usTems |= (pBFData->sCE_DropLockST<<4);						//换电锁锁紧状态
	bPackBuf[PackLen++] = usTems;

	//填充长度
	bPackBuf[1] = (PackLen - 3) >> 8;
	bPackBuf[2] = (PackLen - 3) >> 0;
	return PackLen;	
}

/*
功能：打包98数据
*/
uint16_t Pack98Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0;
	bPackBuf[0] = 0x98;																			//信息类型：自定义数据0x80
	PackLen = 3;
	bPackBuf[PackLen++] = gUserDara.remoteLockState;	//远程锁车状态				
	bPackBuf[PackLen++] = gUserDara.heartLockState;		//心跳锁车状态		
	bPackBuf[1] = (uint8_t)((PackLen-3)>>8);
	bPackBuf[2] = (uint8_t)((PackLen-3)>>0);
	return PackLen;	
}

/*
功能：自定义数据上送
描述：扩展实时数据，实时数据增加自定义数据
*/
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;
	if(sendSelfData == 1)									//1：默认上送自定义数据 	0：不上送自定义数据
	{
		index += PackB0Data(pRealData,&bPackBuf[index]);
		index += PackB2Data(pRealData,&bPackBuf[index]);
		index += PackB3Data(pRealData,&bPackBuf[index]);
		index += PackB4Data(pRealData,&bPackBuf[index]);
		index += PackB5Data(pRealData,&bPackBuf[index]);
		index += PackBFData(pRealData,&bPackBuf[index]);
		index += Pack98Data(pRealData,&bPackBuf[index]);
	}
	return index;
}

/*
功能：使用终端编号，或者设备号代替VIN连接上送平台
*/
uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
	if(sendUseVIN_TermID == 1)							//1：默认使用终端ID上送 	0：使用VIN上送
	{
		//if(!(cmd == 1 && rsp == 1))
		{
			memset(vin,'0',17);
			memcpy(&vin[0],gFrimPara.terminalId,12);
			return 1;
		}	
	}

	return 0;
}
