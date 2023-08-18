/*
文 件：protocol_GB_EX_NJJL_DK.c
功 能：南京金龙通信协议 - 解析上送自定义扩展数据
日 期: 2021/12/9
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/

#include "NJJL_DK/protocol_GB_EX_NJJL_DK.h"

/*南京金龙大客自定义数据*/
SelfData80* pSelfData80;					
SelfData81* pSelfData81;
SelfData82* pSelfData82;
SelfData83* pSelfData83;
SelfData84* pSelfData84;
SelfData85* pSelfData85;
SelfData86* pSelfData86;
SelfData87* pSelfData87;
SelfData88* pSelfData88;
/* 远程锁车参数 */
REMOTELOCK_PARA gRemoteLockPara;	
uint8_t sendLockCMDSign = 0;									//发送锁车指令开关

static uint16_t p80offset;
static uint16_t p81offset;
static uint16_t p82offset;
static uint16_t p83offset;
static uint16_t p84offset;
static uint16_t p85offset;
static uint16_t p86offset;
static uint16_t p87offset;
static uint16_t p88offset;

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
	buff[30] = (uint8_t)(gRemoteLockPara.carDoorCtrCount >> 8);         	//控制流水号
	buff[31] = (uint8_t)(gRemoteLockPara.carDoorCtrCount >> 0);
	buff[32] = gRemoteLockPara.carDoorCtrRspCode;
	return MakeCmd(CMD_LOCKCARRRSP,0xFE,vin,buff,9);
}

static uint8_t UnpackLockCMD(uint8_t link,char* vin,const uint8_t *rbuf, uint16_t rLen)
{
	uint8_t ResponCtrlBuf[40];
	uint8_t replyFlag = 0x01;
	uint16_t DataAreaLen = (uint16_t)rbuf[22]*256 + rbuf[23];	
	memcpy(ResponCtrlBuf,rbuf,DataAreaLen + 25);
	
	gRemoteLockPara.carDoorCtrCount = rbuf[30] << 8 | rbuf[31];				//流水号
	gRemoteLockPara.carDoorCtrCode = rbuf[32];												//锁车控制指令
	gRemoteLockPara.carDoorCtrRspCode = 0;														//锁车响应代码
	gRemoteLockPara.isReturnLockState = 0xAA;													
	
	sendLockCMDSign = 5;																							//发送锁车指令开关
	
	//保存COMMON_PARA
	System_Pare_Save();
	
	DataAreaLen = MakeCmd(CMD_LOCKCAR,replyFlag,vin,ResponCtrlBuf,DataAreaLen);
	//simSendData(link,ResponCtrlBuf,DataAreaLen);
	return 0;
}

#define MAX_YZT_LINK 1

//掉电保存链路参数
typedef struct _CFG
{
	uint32_t canMsgSeq;							//can消息流水号
	uint32_t canLogSeq;							//can日志流水号
}CFG;

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

//扩展协议初始化，返回配置状态参数
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
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
	//自定义数据分配内存，设置偏移量
	p80offset = 0;
	pSelfData80 = (SelfData80*)&gRealData.externData[p80offset];
	
	p81offset = p80offset + sizeof(SelfData80);
	pSelfData81 = (SelfData81*)&gRealData.externData[p81offset];
	
	p82offset = p81offset + sizeof(SelfData81);
	pSelfData82 = (SelfData82*)&gRealData.externData[p82offset];
	
	p83offset = p82offset + sizeof(SelfData82);
	pSelfData83 = (SelfData83*)&gRealData.externData[p83offset];
	
	p84offset = p83offset + sizeof(SelfData83);
	pSelfData84 = (SelfData84*)&gRealData.externData[p84offset];
	
	p85offset = p84offset + sizeof(SelfData84);
	pSelfData85 = (SelfData85*)&gRealData.externData[p85offset];

	p86offset = p85offset + sizeof(SelfData85);
	pSelfData86 = (SelfData86*)&gRealData.externData[p86offset];
	
	p87offset = p86offset + sizeof(SelfData86);
	pSelfData87 = (SelfData87*)&gRealData.externData[p87offset];
	
	p88offset = p87offset + sizeof(SelfData87);
	pSelfData88 = (SelfData88*)&gRealData.externData[p88offset];
	return &gbSta[objLinkIdx];
}

//扩展协议运行   ctrl:0 网络离线 1:网络在线 2:睡眠 sta:0 国标未登入 1:国标已登入 返回值:0 扩展协议未登入 1:扩展协议已登入
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
	if(pSta->sendOverTimeCnt >= 3 &&  osKernelGetTickCount() - pSta->lockStaStamp >= 3000)
	{
		pSta->sendOverTimeCnt = 0;
		//连续3次超3秒响应超时,重播
		//reConnect(pSta->bLink);
	}
	if(osKernelGetTickCount() - pSta->lockStaStamp >= 30000 && gRemoteLockPara.isReturnLockState == 0xA5)//控制状态回复
	{
		pSta->lockStaStamp = osKernelGetTickCount();
		//if((dataLen = packCtrlStaFeedBack(pSta->bLink,pSta->vin,pSta->buff,pSta->buffLen)) > 0)
		//  simSendData(pSta->bLink,pSta->buff,dataLen);
	}
	return 1;
}


//扩展协议接收解包
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

uint16_t Pack80Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x80;
	SelfData80*  p80Data = (SelfData80*)&pRealData->externData[p80offset];		
	
	//填充自定义数据
	bPackBuf[index++] = p80Data->sCarState;															//整车状态				
	bPackBuf[index++] = p80Data->sRunModel;															//车辆运行模式
	bPackBuf[index++] =	(uint8_t)(p80Data->sRunMileage >> 8);
	bPackBuf[index++] = (uint8_t)(p80Data->sRunMileage >> 0);						//续航里程	
	bPackBuf[index++] = p80Data->sPowerAvgConsumption;									//百公里平均电耗
	bPackBuf[index++] = p80Data->sVCUTorsion >> 8;
	bPackBuf[index++] = p80Data->sVCUTorsion >> 0;											//VCU扭矩请求	
	bPackBuf[index++] =	(uint8_t)(p80Data->sPhaseCurr >> 8);
	bPackBuf[index++] = (uint8_t)(p80Data->sPhaseCurr >> 0);						//相电流	
	bPackBuf[index++] = p80Data->sHighDevState; 												//高压附件使能状态
	bPackBuf[index++] = p80Data->sVCUTouchControlCmd;										//VCU接触器控制指令	
	bPackBuf[index++] = p80Data->sVCUTouchCloseState;										//VCU接触器闭合状态
	bPackBuf[index++] = p80Data->sVCUFault;															//VCU故障
	bPackBuf[index++] = p80Data->sMCUFault;															//MCU故障
	bPackBuf[index++] = p80Data->sLifeSign;															//Life信号
	bPackBuf[index++] = p80Data->sVCUVersionInfo;												//VCU版本信息
	bPackBuf[index++] =	(uint8_t)(p80Data->sCarAcceleratedSpeed >> 8);
	bPackBuf[index++] = (uint8_t)(p80Data->sCarAcceleratedSpeed >> 0);	//整车加速度
	bPackBuf[index++] = p80Data->sCarState1;														//车辆状态1
	bPackBuf[index++] = p80Data->sCarFaultState1;												//车辆故障状态1
	
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																						//预留3
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																						//预留4
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																						//预留5			
	
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack81Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	uint8_t Tmp = 0;
	
	bPackBuf[0] = 0x81;
	SelfData81*  p81Data = (SelfData81*)&pRealData->externData[p81offset];		
	//填充自定义数据
		Tmp = 0;		
		if(p81Data->sBMSFaultLevel <= 3)
		{
			Tmp |= (p81Data->sBMSFaultLevel << 6);   													//BMS故障等级
		}
		if(p81Data->sBMSFaultShutHighPower <= 3)
		{
			Tmp |= (p81Data->sBMSFaultShutHighPower << 4);										//BMS异常状态下请求切断高压
		}		
		Tmp = Tmp | (p81Data->sBalanceState > 0 ? 0x08 : 0x00);          		//均衡状态
		Tmp = Tmp | (p81Data->sHeatState > 0 ? 0x04 : 0x00);         				//加热状态
		Tmp = Tmp | (p81Data->sBatCoolState > 0 ? 0x02 : 0x00);       	 		//电池冷却状态
		Tmp = Tmp | (p81Data->sCHGLinkState > 0 ? 0x01 : 0x00);           	//充电连接状态
		bPackBuf[index++] = Tmp;
		
		Tmp = 0;		
		Tmp = Tmp | (p81Data->sSingleTemOffLinState > 0 ? 0x20 : 0x00); 		//温度采集掉线状态
		Tmp = Tmp | (p81Data->sSingleBatOffLinState > 0 ? 0x10 : 0x00); 		//单体电压采集掉线状态
		Tmp = Tmp | (p81Data->sBatProtectOut > 0 ? 0x08 : 0x00);          	//电池放电保护（用于持续小于10A放电2小时切断总负）
		Tmp = Tmp | (p81Data->sFireLimitFaultAlarm > 0 ? 0x04 : 0x00);      //火灾极限故障报警
		Tmp = Tmp | (p81Data->sBranchPreAlarm > 0 ? 0x02 : 0x00);       	 	//支路压差报警
		Tmp = Tmp | (p81Data->sToucherLinkFault > 0 ? 0x01 : 0x00);         //接触器粘连故障
		bPackBuf[index++] = Tmp;
				
		Tmp = 0;		
		Tmp = Tmp | (p81Data->sBMSControlOffLinAlarm > 0 ? 0x20 : 0x00); 		//BMS从控掉线报警（针对某箱数据丢失）
		Tmp = Tmp | (p81Data->sCHGDevInfoAlarm > 0 ? 0x10 : 0x00); 					//与充电机通信报警
		Tmp = Tmp | (p81Data->sPreCHGAlarm > 0 ? 0x08 : 0x00);          		//预充电报警
		Tmp = Tmp | (p81Data->sBalanceAlarmState > 0 ? 0x04 : 0x00);      	//均衡报警状态
		Tmp = Tmp | (p81Data->sHeatFaultAlarmState > 0 ? 0x02 : 0x00);     	//加热故障报警状态
		Tmp = Tmp | (p81Data->sBatCoolSystemFault > 0 ? 0x01 : 0x00);       //电池冷却系统故障
		bPackBuf[index++] = Tmp;	
		
		Tmp = 0;		
		Tmp = Tmp | (p81Data->sBatSystemOtherFault > 0 ? 0x80 : 0x00); 			//电池系统其他故障
		Tmp = Tmp | (p81Data->sBMSCommunicationFault > 0 ? 0x40 : 0x00); 		//BMS通讯故障（针对can硬件故障）		
		if(p81Data->sBatLowTemAlarm <= 3)
		{
			Tmp |= (p81Data->sBatLowTemAlarm <<4);														//电池低温报警
		}
		if(p81Data->sSOCDiffenceAlarm <= 3)
		{
			Tmp |= (p81Data->sSOCDiffenceAlarm << 2);													//SOC差异报警
		}	
		if(p81Data->sCHGCurrentAlarm <= 3)
		{
			Tmp |= (p81Data->sCHGCurrentAlarm << 0);													//充电电流报警
		}		
		bPackBuf[index++] = Tmp;		
		
		Tmp = 0;		
		if(p81Data->sOutCurrentAlarm <= 3)
		{
			Tmp |= (p81Data->sOutCurrentAlarm << 6);													//放电电流报警
		}
		if(p81Data->sCHGGunHighTemAlarm <= 3)
		{
			Tmp |= (p81Data->sCHGGunHighTemAlarm << 2);												//充电枪高温报警
		}	
		if(p81Data->sPoleColumnHighTemAlarm <= 3)
		{
			Tmp |= (p81Data->sPoleColumnHighTemAlarm << 0);										//极柱高温报警
		}	
		bPackBuf[index++] = Tmp;	
				
		bPackBuf[index++] = 0xFE;																						//预留9
		
		Tmp = 0;		
		if(p81Data->sInsulationAlarmState <= 3)
		{
			Tmp |= (p81Data->sInsulationAlarmState << 2);											//绝缘监测报警状态
		}	
		if(p81Data->sSOCHighAlarm <= 3)
		{
			Tmp |=(p81Data->sSOCHighAlarm << 0);															//SOC高报警
		}	
		bPackBuf[index++] = Tmp;	
		
		Tmp = 0;		
		if(p81Data->sSOCLowAlarm1 <= 3)
		{
			Tmp |= (p81Data->sSOCLowAlarm1 << 6);															//SOC低报警1
		}
		if(p81Data->sTemDiffAlarm1 <= 3)
		{
			Tmp |= (p81Data->sTemDiffAlarm1 << 4);														//温度差异报警1
		}
		if(p81Data->sSingleDifVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sSingleDifVlotAlarm<< 2);												//单体电压差异报警
		}	
		if(p81Data->sSingleLowVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sSingleLowVlotAlarm << 0);												//单体欠压报警
		}	
		bPackBuf[index++] = Tmp;

		Tmp = 0;		
		if(p81Data->sBatsLowVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sBatsLowVlotAlarm << 6);													//电池组欠压报警
		}
		if(p81Data->sBatsOverVlotAlarm <= 3)
		{		
			Tmp |= (p81Data->sBatsOverVlotAlarm << 4);												//电池组过压报警
		}
		if(p81Data->sSingleOverVlotAlarm <= 3)
		{
			Tmp |= (p81Data->sSingleOverVlotAlarm << 2);											//单体过压报警
		}	
		if(p81Data->sBatHighTemAlarm1 <= 3)
		{
			Tmp |= (p81Data->sBatHighTemAlarm1 << 0);													//电池高温报警1
		}	
		bPackBuf[index++] = Tmp;
		
		bPackBuf[index++] = p81Data->sBatFaultNum;													//动力电池故障码
		bPackBuf[index++] = p81Data->sBatsCount >> 8;
		bPackBuf[index++] = p81Data->sBatsCount >> 0;												//电池组总串数
		bPackBuf[index++] = p81Data->sBatsTemsCount;												//电池组温度点数
		bPackBuf[index++] = p81Data->sMaxLongInCurrent;											//最大可用持续充电电流（5min）
		bPackBuf[index++] = p81Data->sMaxShortInCurrent;										//最大可用短时充电电流（30s）
		bPackBuf[index++] = p81Data->sMaxLongOutCurrent;										//最大可用持续放电电流（5min）
		bPackBuf[index++] = p81Data->sMaxShortOutCurrent;										//最大可用短时放电电流（30s）
		bPackBuf[index++] = p81Data->sBMSTouchControlCMD;										//BMS接触器控制命令
		bPackBuf[index++] = p81Data->sBMSTouchControlCloseState;						//BMS接触器闭合状态
		bPackBuf[index++] = p81Data->sCHGCounts >> 8;	
		bPackBuf[index++] = p81Data->sCHGCounts >> 0;												//充电次数
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 24);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 16);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 8);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpOutPower >> 0);			//电池组累计输出能量
		
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 24);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 16);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 8);
		bPackBuf[index++] = (uint8_t)(p81Data->sBMSAddUpChgPower >> 0);			//电池组累计充电（不含制动回馈）能量
		bPackBuf[index++] = p81Data->sInsulationCheckAllVolt >> 8;	
		bPackBuf[index++] = p81Data->sInsulationCheckAllVolt >> 0;					//绝缘检测电池总压
		bPackBuf[index++] = p81Data->sCHGPlusesTem1;												//充电抢1正温度
		bPackBuf[index++] = p81Data->sCHGMinusTem1;													//充电枪1负温度
		bPackBuf[index++] = p81Data->sCHGPlusesTem2;												//充电抢2正温度
		bPackBuf[index++] = p81Data->sCHGMinusTem2;													//充电枪2负温度
		bPackBuf[index++] = p81Data->sBatsProductDate_Month;								//电池组生产日期（月）
		bPackBuf[index++] = p81Data->sBatsProductDate_Year;									//电池组生产日期（年）【】
		bPackBuf[index++] = p81Data->sBatsProducer;													//动力电池生产厂家
		bPackBuf[index++] = p81Data->sBMSLifeSignal;												//BMS life信号
		bPackBuf[index++] = p81Data->sBMSSoftwareVersion;										//BMS程序版本
		
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留2
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留3
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留4
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留5		

	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;
}

uint16_t Pack82Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x82;
	SelfData82*  p82Data = (SelfData82*)&pRealData->externData[p82offset];		
	//填充自定义数据
	bPackBuf[index++] = p82Data->sFourInOne_State;												//4合1状态		
	bPackBuf[index++] = p82Data->sFourInOne_FaultNum;											//4合1故障码
	bPackBuf[index++] = p82Data->sFourInOne_BMSToucherState;							//4合1 - BMS接触器状态反馈
	bPackBuf[index++] = p82Data->sFourInOne_VCUToucherState;							//4合1 - VCU接触器状态反馈
	bPackBuf[index++] = p82Data->sFourInOne_BMSToucherFaultState;					//4合1 - BMS接触器故障状态
	bPackBuf[index++] = p82Data->sFourInOne_VCUToucherFaultState;					//4合1 - VCU接触器故障状态	
	bPackBuf[index++] = (uint8_t)( p82Data->sHighOilPump_OutVolt >> 8);
	bPackBuf[index++] = (uint8_t)( p82Data->sHighOilPump_OutVolt >> 0);		//高压油泵输出电压		
	bPackBuf[index++] = p82Data->sHighOilPump_DCACOutCur;									//高压油泵DC/AC输出电流
	bPackBuf[index++] = p82Data->sHighOilPump_MoterTem;										//高压油泵电机温度
	bPackBuf[index++] = p82Data->sHighOilPump_DCACStateAndFault;					//高压油泵DC/AC状态及故障
	bPackBuf[index++] = p82Data->sHighOilPump_ConverterFaultNum;					//高压油泵变频器故障码
	bPackBuf[index++] = p82Data->sHighOilPump_motorSpeed;									//高压油泵转速
	bPackBuf[index++] = p82Data->sHighOilPump_DCACLifeSignal;							//高压油泵DC/AC life信号
	bPackBuf[index++] = p82Data->sDCDC_RealTimeOutCur;										//DC/DC实时输出电流
	bPackBuf[index++] = p82Data->sDCDCTem;																//DC/DC本体温度
	bPackBuf[index++] = p82Data->sDCDCWorkState;													//DCDC工作状态
	bPackBuf[index++] = p82Data->sDCDCLifeSignal;													//DCDC Life信号
	bPackBuf[index++] = (uint8_t)( p82Data->sAirPump_DCACOutVolt >> 8);
	bPackBuf[index++] = (uint8_t)( p82Data->sAirPump_DCACOutVolt >> 0);		//气泵DC/AC输出电压
	bPackBuf[index++] = p82Data->sAirPump_DCACOutCur;											//气泵DC/AC输出电流
	bPackBuf[index++] = p82Data->sAirPump_DCACStateAndFault;							//气泵DC/AC状态及故障		
	bPackBuf[index++] = p82Data->sAirPump_Tem;														//气泵温度
	bPackBuf[index++] = p82Data->sAirPump_ConverterFaultNum;							//气泵变频器故障码
	bPackBuf[index++] = p82Data->sAirPump_motorSpeed;											//气泵转速
	bPackBuf[index++] = p82Data->sAirPump_DCACLifeSignal;									//气泵DC/AC Life信号
	bPackBuf[index++] = (uint8_t)( p82Data->sLowOilPump_OutVolt >> 8);
	bPackBuf[index++] = (uint8_t)( p82Data->sLowOilPump_OutVolt >> 0);		//低压油泵输出电压
	bPackBuf[index++] = p82Data->sLowOilPump_DCACOutCur;									//低压油泵DC/AC输出电流
	
	bPackBuf[index++] = p82Data->sLowOilPump_DCACStateAndFault;						//低压油泵DC/AC状态及故障
	bPackBuf[index++] = p82Data->sLowOilPump_ConverterFaultNum;						//低压油泵变频器故障码
	bPackBuf[index++] = p82Data->sLowOilPump_motorSpeed;									//低压油泵转速
	bPackBuf[index++] = p82Data->sLowOilPump_DCACLifeSignal;							//低压油泵DC/AC life信号
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//预留2
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//预留3
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//预留4
	bPackBuf[index++] = 0xFF;
	bPackBuf[index++] = 0xFF;																							//预留5		

	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;
}

uint16_t Pack83Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x83;
	SelfData83*  p83Data = (SelfData83*)&pRealData->externData[p83offset];		
	//填充自定义数据	
		bPackBuf[index++] = p83Data->sLowBatVolt;														//低压电池电压		
		bPackBuf[index++] =  p83Data->sFrontBrakeAirPressure;								//前制动储气筒气压
		bPackBuf[index++] =  p83Data->sRearBrakeAirPressure;								//后制动储气筒气压
		
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>24);						//总里程	
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>16);
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>8);
		bPackBuf[index++] = (uint8_t)(p83Data->sAllMileage>>0);										
		
		bPackBuf[index++] =  p83Data->sCarState1;														//车辆状态1
		bPackBuf[index++] =  p83Data->sCarState2;														//车辆状态2
		bPackBuf[index++] =  p83Data->sCarState3;														//车辆状态3		
		bPackBuf[index++] =  p83Data->sInstrumentAlarmState1;								//仪表报警状态1
		bPackBuf[index++] =  p83Data->sInstrumentAlarmState2;								//仪表报警状态2
		bPackBuf[index++] = p83Data->sInstrumentSoftwareVersion;						//仪表程序版本

		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留19
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留20
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留21
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																						//预留22			
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;
}

uint16_t Pack84Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x84;
	SelfData84*  p84Data = (SelfData84*)&pRealData->externData[p84offset];		
	//填充自定义数据
		bPackBuf[index++] = p84Data->sBMSCoolWorkMode;									//BMS冷却请求工作模式
		bPackBuf[index++] = p84Data->sBMSsetoutWaterTem;								//BMS水冷机组出水口（电池入水口）设定温度
		bPackBuf[index++] = p84Data->sBatsHighestTem;										//电池组最高温度
		bPackBuf[index++] = p84Data->sBatsLowestTem;										//电池组最低温度
		bPackBuf[index++] = p84Data->sBatsQueLifeValue;									//电池组请求life值
		bPackBuf[index++] = p84Data->sHotContrlMode;										//热管理系统工作模式汇总
		bPackBuf[index++] = p84Data->sInWaterTem;												//进水温度
		bPackBuf[index++] = p84Data->sOutWaterTem;											//出水温度
		bPackBuf[index++] = p84Data->sCompressorPower;									//压缩机负载比率
		bPackBuf[index++] = p84Data->sHotContrlFaultNum;								//热管理系统故障码
		bPackBuf[index++] = p84Data->sHotContrlLifeValue;								//热管理工作life值

		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//预留23
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//预留24
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//预留25
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																				//预留26		
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}
uint16_t Pack85Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x85;
	SelfData85*  p85Data = (SelfData85*)&pRealData->externData[p85offset];		
	//填充自定义数据
		bPackBuf[index++] = p85Data->sAirConditionerOpenCMD;						//空调开关机命令/状态
		bPackBuf[index++] = p85Data->sAirConditionerSetTem;							//空调设定温度
		bPackBuf[index++] = p85Data->sAirConditionerRunStall;						//空调风机运行档位
		bPackBuf[index++] = p85Data->sInCarTem;													//车内环境温度		
		bPackBuf[index++] = p85Data->sAirConditionerRunMode;						//空调整机运行模式
		bPackBuf[index++] = p85Data->sOutCarTem;												//车外环境温度
		bPackBuf[index++] = p85Data->sAirToucherContrlAndState;					//空调接触器控制以及状态
		bPackBuf[index++] = (uint8_t)(p85Data->sAirSystemVolt >> 8);													
		bPackBuf[index++] = (uint8_t)(p85Data->sAirSystemVolt >> 0);								//空调系统母线电压
		bPackBuf[index++] = p85Data->sAirSystem_PartsRunState;											//空调系统一部件运行状态
		bPackBuf[index++] = p85Data->sAirSystem_SystemRunState;										//空调系统一系统运行状态
		bPackBuf[index++] = p85Data->sAirRunTargetHz;															//压缩机目标频率
		bPackBuf[index++] = p85Data->sAirRunHz;																		//压缩机运行频率
		bPackBuf[index++] = p85Data->sAirFaultNum;																	//空调故障码
		bPackBuf[index++] = p85Data->sAirLife;																			//空调life	
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack86Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x86;
	SelfData86*  p86Data = (SelfData86*)&pRealData->externData[p86offset];		
	//填充自定义数据
		bPackBuf[index++] = p86Data->sTirePosition;																//轮胎位置
		bPackBuf[index++] = p86Data->sTirePressure;																//轮胎压力
		bPackBuf[index++] = (uint8_t)(p86Data->sTireTem >> 8);																	
		bPackBuf[index++] = (uint8_t)(p86Data->sTireTem >> 0);										//轮胎温度		
		bPackBuf[index++] = p86Data->sTireState;																	//状态（轮胎状态）		
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																									//预留27
		bPackBuf[index++] = p86Data->sPressureValveCheck;													//压力阀检测
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack87Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x87;
	SelfData87*  p87Data = (SelfData87*)&pRealData->externData[p87offset];		
	//填充自定义数据
		bPackBuf[index++] = p87Data->sFlameArrester_SystemState;										//灭火器系统状态
		bPackBuf[index++] = p87Data->sBatsNum;																			//电池组号
		bPackBuf[index++] = p87Data->sSensorBoxState;															//箱体内传感器工作状态
		bPackBuf[index++] = p87Data->sSensorBoxFault;															//箱体内故障状态等级
		bPackBuf[index++] = p87Data->sSensorBoxTem;																//箱体内温度
		bPackBuf[index++] = p87Data->sSensorBoxStartState;													//箱体内灭火器启动状态
		bPackBuf[index++] = 0xFF;																											//预留28
		bPackBuf[index++] = p87Data->sSensorBoxLife;																//LIFE
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack88Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3;
	bPackBuf[0] = 0x88;
	SelfData88*  p88Data = (SelfData88*)&pRealData->externData[p88offset];		
	//填充自定义数据
		bPackBuf[index++] = p88Data->sAheadCarWarning;															//前车碰撞警告
		bPackBuf[index++] = p88Data->sAheadCarDistance >> 8;												
		bPackBuf[index++] = p88Data->sAheadCarDistance >> 0;												//前车距离
		bPackBuf[index++] = p88Data->sAheadCar_RelativeSpeed >> 8;									
		bPackBuf[index++] = p88Data->sAheadCar_RelativeSpeed >> 0;									//相对速度
		bPackBuf[index++] = p88Data->sLaneWarning;																	//车道偏离警告
		bPackBuf[index++] = p88Data->sLaneDirection;																//车道偏离方向
		bPackBuf[index++] = p88Data->sActivateCollisionWarning;										//启动碰撞预警功能
		bPackBuf[index++] = p88Data->sActivateEmergencyBraking;										//启动紧急制动功能
		bPackBuf[index++] = p88Data->sABESystemState;															//AEB系统状态
			
		bPackBuf[index++] = 0xFF;
		bPackBuf[index++] = 0xFF;																											//预留29	
	//填充长度
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	return index;	
}

uint16_t Pack98Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0;
	bPackBuf[0] = 0x98;																			//信息类型：自定义数据0x80
	PackLen = 3;
	bPackBuf[PackLen++] = gRemoteLockPara.remoteLockState;	//远程锁车状态				
	bPackBuf[PackLen++] = gRemoteLockPara.heartLockState;		//心跳锁车状态		
	bPackBuf[1] = (uint8_t)((PackLen-3)>>8);
	bPackBuf[2] = (uint8_t)((PackLen-3)>>0);
	return PackLen;	
}

//扩展实时数据，实时数据增加自定义数据
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;;
	index += Pack80Data(pRealData,&bPackBuf[index]);
	index += Pack81Data(pRealData,&bPackBuf[index]);
	index += Pack82Data(pRealData,&bPackBuf[index]);
	index += Pack83Data(pRealData,&bPackBuf[index]);
	index += Pack84Data(pRealData,&bPackBuf[index]);
	index += Pack85Data(pRealData,&bPackBuf[index]);
	index += Pack86Data(pRealData,&bPackBuf[index]);
	index += Pack87Data(pRealData,&bPackBuf[index]);
	index += Pack88Data(pRealData,&bPackBuf[index]);
	index += Pack98Data(pRealData,&bPackBuf[index]);
	return index;
}

uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff)
{
	if(!(cmd == 1 && rsp == 1))
	{
		memset(vin,'0',17);
		memcpy(&vin[5],gFrimPara.terminalId,12);
		return 1;
	}
	else if(resvBuff != NULL)
	{
		if(resvBuff[0] == 'L'  && memcmp(gSysPara.vinCode,resvBuff,17) != 0)
		{
			memcpy(gSysPara.vinCode,resvBuff,17);
			System_Pare_Save();
		}
		memcpy(vin,resvBuff,17);
	}
	return 0;
}
