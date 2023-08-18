
#include "diagnostic.h"
static uint32_t getUdsTime = 0;


//电压低检测
DTCTestResult checkVolBreak(void)
{
	if(osKernelGetTickCount() < 15000 ){
		return IN_TESTING;
	}
	else if(gTerminalState.pwrVolt < 6.5f){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//GPRS天线开路检测，模块不支持，以SIM卡状态代替
DTCTestResult checkGprsAntOpen(void)
{
	static uint32_t faultTime = 0;
	if(osKernelGetTickCount() < 15000 ){
		return IN_TESTING;
	}
	else if(!gTerminalState.simCardState){
		if(faultTime == 0)
		{
			faultTime = osKernelGetTickCount();
		}
		if(osKernelGetTickCount() - faultTime >= 30000)
		{
			return FAILEDD;
		}
		else
		{
			return IN_TESTING;
		}
	}
	else{
		faultTime = 0;
		return PASSED;
	}
}

//GSP天线短路检测
DTCTestResult checkGpsAntShort(void)
{
	if(osKernelGetTickCount() < 15000){
		return IN_TESTING;
	}
	else if(gTerminalState.gpsState & 0x02){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//GPS天线开路检测
DTCTestResult checkGpsAntOpen(void)
{
	if(osKernelGetTickCount() < 15000){
		return IN_TESTING;
	}
	else if(gTerminalState.gpsState & 0x04){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//GPRS通信故障,版本不能识别
DTCTestResult checkGprsFault(void)
{
	if(osKernelGetTickCount() < 60000){
		return IN_TESTING;
	}
	else if(gTerminalState.gprsVer[0] == 0){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}


//UDSCAN发送
static uint8_t SendFrame(uint32_t ID, uint8_t *array, uint8_t length, uint8_t priority, uint8_t rtr, uint8_t ide)
{
	CAN_msg msg_buf = {0x346,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,STANDARD_TYPE,DATA_FRAME};
	msg_buf.id = ID;
	msg_buf.len = length > 8 ? 8 : length;
	memcpy(msg_buf.data,array,msg_buf.len);
	msg_buf.format = ide;
	msg_buf.type = rtr;
	return CAN_send(1,&msg_buf,10);
}
static uint8_t SnapshotPowerVol;
static uint8_t SnapshotPowerCsq;

//UDS初始化
void TerimalUSD_Init(TP_T* pTp,uint8_t ch)
{
	FLASH_Unlock();
	Diagnostic_Init(pTp,0x18000101, 0x18000202 , 0x18000103, 0 , 1024 , SendFrame,50,2000);
	
	//********************************** service 10*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x10,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO);
	InitSetSessionControlParam(TRUE , TRUE , TRUE , FALSE , FALSE , TRUE); 
	//********************************** service 14*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x14,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO);
	InitAddDTCGroup(0x00100000); //动力
	InitAddDTCGroup(0x00400000); //底盘
	InitAddDTCGroup(0x00800000); //车身
	InitAddDTCGroup(0x00C00000); //通信	
	InitAddDTCGroup(0x00FFFFFF); //全部
	//********************************** service 19*****************************************//
 	InitSetSessionSupportAndSecurityAccess(TRUE,0x19,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO);
	InitSetDTCAvailiableMask(0x09);
	
	//DTC代码,故障检测函数,,DTC确认次数值,DTC等级
	InitAddDTC(0xC00010,checkVolBreak,10,1 ,LEVEL_C);				//ECU外部供电断开
	InitAddDTC(0xC00011,checkGprsAntOpen,10, 1 ,LEVEL_C);		//检测到4G天线开路
	InitAddDTC(0xC00012,checkGpsAntShort,10, 1 ,LEVEL_C);		//检测到GPS天线短路
	InitAddDTC(0xC00013,checkGpsAntOpen,10, 1 ,LEVEL_C);		//检测到GPS天线开路
	InitAddDTC(0xC00014,checkGprsFault,10, 1 ,LEVEL_C);			//MCU检测到4G模块无法通讯
	
	//快照记录号，快照ID，数据指针，数据长度
	InitAddDTCSnapShot(0x01 , 0xDF01 , &SnapshotPowerCsq ,1);			//CSQ
	InitAddDTCSnapShot(0x01 , 0xDF02 , &SnapshotPowerVol ,1);			//供电电压
	
	InitSetAgingCounterRecordNumber(3);
	InitSetAgedCounterRecordNumber(2);
	InitSetOccurrenceCounterRecordNumber(1);
	InitSetPendingCounterRecordNumber(0);
	Diagnostic_LoadAllData();
	Diagnostic_SetNLParam(pTp,70, 150, 150, 70, 20, 70, 0, 20, 0);
}

/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
void udsRecvData(TP_T* pTp,uint8_t ch,CAN_msg *msg)
{
	SnapshotPowerCsq = gTerminalState.csq;
	SnapshotPowerVol = gTerminalState.pwrVolt * 10;

	uint32_t ticks = osKernelGetTickCount();
	switch(msg->id)
	{
		case 0x18000103:getUdsTime = ticks;break;
		case 0x18000101:getUdsTime = ticks;break;
		default:break;
	}
	if(msg->id == 0x18000103 || msg->id == 0x18000101)
	{
		Diagnostic_RxFrame(pTp,msg->id, msg->data , msg->format , msg->len , msg->type);	
	}
}











