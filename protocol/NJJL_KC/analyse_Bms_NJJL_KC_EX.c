/*
文 件：analyse_Bms_NJJL_KC_EX.c
功 能：南京金龙卡车通信协议 - 解析自定数据
日 期: 2021/12/30
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
#include "NJJL_KC/protocol_GB_EX_NJJL_KC.h"
#include <stdlib.h>

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

SelfDataB0 *gSelfDataB0;							/*南京金龙卡车自定义数据*/
SelfDataB2 *gSelfDataB2;
SelfDataB3 *gSelfDataB3;
SelfDataB4 *gSelfDataB4;
SelfDataB5 *gSelfDataB5;
SelfDataBF *gSelfDataBF;

uint16_t selfDataB0Pos;
uint16_t selfDataB2Pos;
uint16_t selfDataB3Pos;
uint16_t selfDataB4Pos;
uint16_t selfDataB5Pos;
uint16_t selfDataBFPos;

uint8_t sendLockCMDSign = 0;					//锁车指令发送标志

uint16_t upFirstSendLockTime = 0;			//每次程序更新发送心跳锁指令
uint8_t upDataHeartLock = 0;					//每次程序更新发送心跳锁指令标志
uint32_t upDataFirstTime = 0;					//每次程序更新 发送计时


CAN_msg	msg_buf;																			//CAN 报文
uint32_t termID = 0;																	//十进制-终端编号

uint8_t isRequestVINCounts = 10;											//是否成功请求VIN  0：未成功应答   1：成功应答
uint8_t requestVINSing = 0;														//请求VIN标志

uint8_t heartNum = 0;																	//心跳状态
uint8_t linkNum = 0;																	//保持连接

uint8_t randNums = 0;																	//随机数

void unpackSelfInit(void);												//企标数据初始化
void unpackSelfcan(uint8_t ch,CAN_msg *msg);			//企标数据解析
static void VCU_ResponseFunction(uint32_t canID,const uint8_t *canData);			//接收解析定制功能反馈CAN
static void SelfFunction(unsigned char canCh);																//用户自定义功能入口

	
/*
功能：企标数据初始化
*/
void unpackSelfInit(void)												//企标数据初始化
{
	//初始化 自定义数据结构体
	selfDataB0Pos = 0;
	gSelfDataB0 = (SelfDataB0*)&gRealData.externData[selfDataB0Pos];
	
	selfDataB2Pos = selfDataB0Pos + sizeof(SelfDataB0);
	gSelfDataB2 = (SelfDataB2*)&gRealData.externData[selfDataB2Pos];

	selfDataB3Pos = selfDataB2Pos + sizeof(SelfDataB2);
	gSelfDataB3 = (SelfDataB3*)&gRealData.externData[selfDataB3Pos];
	
	selfDataB4Pos = selfDataB3Pos + sizeof(SelfDataB3);
	gSelfDataB4 = (SelfDataB4*)&gRealData.externData[selfDataB4Pos];
	
	selfDataB5Pos = selfDataB4Pos + sizeof(SelfDataB4);
	gSelfDataB5 = (SelfDataB5*)&gRealData.externData[selfDataB5Pos];
	
	selfDataBFPos = selfDataB5Pos + sizeof(SelfDataB5);
	gSelfDataBF = (SelfDataBF*)&gRealData.externData[selfDataBFPos];
	
	//终端编号十进制转换
	sscanf(&gFrimPara.terminalId[6],"%d", &termID);
	isRequestVINCounts = 10;													//开机启动 VIN请求次数

}

/*
功能：企标数据CAN解析
*/
void unpackSelfcan(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val;
	if(gSelfDataB0 == NULL || gSelfDataB2 == NULL || gSelfDataB3 == NULL)
	{
		return;
	}
	switch(msg->id)
	{
		case 0x18FFA2F3:
			{
				//自定义 B0数据 - SOH
				gSelfDataB0->sSOH = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);			
			}
			break;	
		case 0x18FFC1F0:
			{				
				//自定义 B4数据 - 驱动电机故障等级
				gSelfDataB4->sMoter_FaultLevel = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义 B4数据 - 驱动电机DTC_CODE
				gSelfDataB4->sMoter_DTCCode = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义 B4数据 - 驱动电机故障总数 
				gSelfDataB4->sMotor_FaultCount = calcCanValue(24,16,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义 B4数据 - MCU温度报警
				gSelfDataB4->sMCU_TempAlarm = calcCanValue(40,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义 B4数据 - 驱动电机温度报警
				gSelfDataB4->sMotor_TempAlarm = calcCanValue(42,2,msg->data,HIGHTLOW_MODE,CAN_MODE);							

			}
			break;
		case 0x18FECAF3:
			{				
				//自定义 B2数据 - sBMSDTC_CODE1
				gSelfDataB2->sBMSDTC_CODE1 = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);					
				//自定义 B2数据 - sBMSDTC_CODE2
				gSelfDataB2->sBMSDTC_CODE2 = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSDTC_CODE3
				gSelfDataB2->sBMSDTC_CODE3 = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSDTC_CODE4
				gSelfDataB2->sBMSDTC_CODE4 = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSDTC_CODE5
				gSelfDataB2->sBMSDTC_CODE5 = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSDTC_CODE6
				gSelfDataB2->sBMSDTC_CODE6 = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSDTC_CODE7
				gSelfDataB2->sBMSDTC_CODE7 = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSDTC_CODE8
				gSelfDataB2->sBMSDTC_CODE8 = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;			
		case 0x18FFAFF3:
			{
				//自定义 B0数据 - 电池组累计输出能量
				gSelfDataB0->sBatsOutTotalEnergy = calcCanValue(0,20,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义 B0数据 - 电池组累计充电能量
				gSelfDataB0->sBatsChgTotalEnergy = calcCanValue(20,20,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义 B0数据 - 电池组累计制动回馈能量
				gSelfDataB0->sBatsBreakOnceEnergy = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FFA1F3:
			{
				//自定义 B0数据 - 电池功率
				gSelfDataB0->sBatsPower = calcCanValue(16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);				//改			
			}
			break;
		case 0x18FF03F3:
			{
				//自定义 B2数据 - sBMSCode1
				gSelfDataB2->sBMSCode1 = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sBMSCode2
				gSelfDataB2->sBMSCode2 = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B2数据 - sDTC_Code
				gSelfDataB2->sDTC_Code = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;
		case 0x18FFA3F3:
			{
				//自定义 B3数据 - 1-8		从控在线状态
				gSelfDataB3->sSlaveControlOnLine_1_8 = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B3数据 - 9-16		从控在线状态
				gSelfDataB3->sSlaveControlOnLine_9_16 = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B3数据 - 17-24		从控在线状态
				gSelfDataB3->sSlaveControlOnLine_17_24 = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B3数据 - 25_32		从控在线状态
				gSelfDataB3->sSlaveControlOnLine_25_32 = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				
				//自定义 B3数据 - 电池箱体数
				gSelfDataB3->sBatsCount = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义 B3数据 - 电池组从控数
				gSelfDataB3->sBatSlaveControlCount = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18FF31F0:
			{
				//自定义 B4数据 - TCU故障代码
				gSelfDataB4->sTCU_FaultCode = calcCanValue(0,16,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义 B4数据 - TCU其他故障代码
				gSelfDataB4->sTCU_OtherFaultCode = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义 B4数据 - TCU故障等级
				gSelfDataB4->sTCU_FaultLevel = calcCanValue(16,3,msg->data,HIGHTLOW_MODE,CAN_MODE);							
			}
			break;
		case 0x08F000A0:
			{
				//自定义 B5数据 - 整车故障等级
				gSelfDataB5->sCarFaultLevel = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);										
			}
			break;
		case 0x08F030A0:
			{
				//自定义 B5数据 - 整车故障码1
				gSelfDataB5->sCarFaultNum1 = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);										
				//自定义 B5数据 - 整车故障码2
				gSelfDataB5->sCarFaultNum2 = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);										
				//自定义 B5数据 - 整车故障码3
				gSelfDataB5->sCarFaultNum3 = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);										
				//自定义 B5数据 - 整车故障码4
				gSelfDataB5->sCarFaultNum4 = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);										
				//自定义 B5数据 - 整车故障码5
				gSelfDataB5->sCarFaultNum5 = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);										
			
			}
			break;
		case 0x0C3C01A0:
			{
				//自定义 BF 数据 换电状态			0 未换电  1 换电中  2 换电完成
				n8Val = calcCanValue(40,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val <= 2)
					gSelfDataBF->sCE_LockST = n8Val;
			
				//自定义 BF 数据 换电锁锁紧状态			0 全部锁紧到位  1 部分锁紧  2 全部未锁紧
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val <= 2)
					gSelfDataBF->sCE_DropLockST = n8Val;				
			}
			break;									
	}
	
	VCU_ResponseFunction(msg->id,msg->data);				//接收反馈定制功能CAN
	SelfFunction(ch);																//用户自定义功能入口
}

/*
功能：接收解析 定制功能反馈信息
*/
static char vinTempBuf[18] = {0};			//VIN接收缓冲区
static uint8_t receVinSign = 0;				//VIN接收完成标志
static void VCU_ResponseFunction(uint32_t canID,const uint8_t *canData)
{
	uint8_t n8Val = 0;

	switch(canID){
		case 0x0C6501A0:												//VCU应答终端请求VIN							
			break;	
		case 0x0CFF01A0:												//VCU反馈VIN				
			{		
					n8Val = calcCanValue(0,8,canData,HIGHTLOW_MODE,CAN_MODE);
					if(0x01 == n8Val)
					{
						memcpy(&vinTempBuf[0],&canData[1],7);
						receVinSign |= 1<<0;
					}
					else if(0x02 == n8Val)
					{
						memcpy(&vinTempBuf[7],&canData[1],7);
						receVinSign |= 1<<1;
					}
					else if(0x03 == n8Val)
					{
						memcpy(&vinTempBuf[14],&canData[1],3);	
						receVinSign |= 1<<2;
					}
			}
			break;
		case 0x1862F0A0:												//VCU反馈锁车状态				
			{
				n8Val= calcCanValue(0,8,canData,HIGHTLOW_MODE,CAN_MODE);		
				if(n8Val < 3)
					gUserDara.remoteLockState = n8Val;
				n8Val = calcCanValue(8,8,canData,HIGHTLOW_MODE,CAN_MODE);							
				if(n8Val < 2)
					gUserDara.heartLockState = n8Val;
			}
			break;
			
			
		case 0x1865F0A0:												//VCU应答远程锁车指令			
			{				
				n8Val = calcCanValue(0,8,canData,HIGHTLOW_MODE,CAN_MODE);										
				if(n8Val==1 || n8Val == 2 || n8Val == 4)
				{
					gUserDara.carDoorCtrRspCode = 1;					
				}
				else
				{
					gUserDara.carDoorCtrRspCode = 0;
				}
				if(sendLockCMDSign>0)
					gUserDara.isReturnLockState = 0xA5;																
			}
			break;
		case 0x1867F0A0:												//VCU应答心跳锁车指令		
			{
				n8Val= calcCanValue(0,8,canData,HIGHTLOW_MODE,CAN_MODE);											
				if(n8Val==1 || n8Val == 4)
				{
					gUserDara.carDoorCtrRspCode = 1;
				}
				else
				{
					gUserDara.carDoorCtrRspCode = 0;
				}
				if(sendLockCMDSign>0)
					gUserDara.isReturnLockState = 0xA5;																			
			}
			break;
	}
	if(receVinSign == 7)										//VIN 更新			
	{
		if(strcmp(gSysPara.vinCode,(char*)vinTempBuf) != 0 && vinTempBuf[0] == 'L')
		{
			memcpy(gSysPara.vinCode,vinTempBuf,17);
			System_Pare_Save();									
		}
		receVinSign = 0;	
	}
}


/*
功能：发送CAN报文到CAN总线
描述：
*/
static uint32_t sframeIntV = 0;
static void send_To_CAN(unsigned char canCh,unsigned int canid,unsigned char *canbuf){
	if(osKernelGetTickCount() - sframeIntV > 10)
	{
		sframeIntV = osKernelGetTickCount();
		msg_buf.id = canid;
		msg_buf.len = 0x08;
		msg_buf.format =  EXTENDED_FORMAT;
		msg_buf.type = DATA_FRAME;
		
		memset(&msg_buf.data[0],0xFF,8);	
		memcpy(&msg_buf.data[0],&canbuf[0],8);
		
		CAN_send(canCh,&msg_buf,10);
	}
}

/*
功能：BCC校验
*/
static uint8_t BCChecks(const uint8_t *buff, uint16_t StartIdx, int16_t len)
{
	uint8_t rtn = 0;
	uint16_t i = 0;
	if (len > 0)
	{
		rtn = buff[StartIdx];
		for (i = 0; i < len - 1; i++)
		{
			rtn = rtn ^ buff[i + StartIdx + 1];
		}
	}
	return rtn;
}

/*
功能：发送心跳状态或保持连接到CAN总线
描述：周期500ms
*/
static void send_LockHeartOrSaveLin(uint8_t cmd,unsigned int canCh ,uint8_t numbs)
{
	uint8_t sendBuf[8] = {0};
	uint8_t tempNums;
	unsigned int canids = 0;
	
	if(cmd == 0xF0)							//心跳锁车信息
	{
		canids = 0x1860A0F0;
		heartNum++;
		if(heartNum > 0xFF){
			heartNum = 0;
		}	
		tempNums = heartNum;
	}
	else if(cmd == 0xEE)
	{
		canids = 0x1861A0F0;				//保持连接信息
		linkNum++;
		if(linkNum > 0xFF){
			linkNum = 0;
		}
		tempNums = linkNum;	
	}
	
	sendBuf[0] = cmd;													
	sendBuf[1] = tempNums;										
	sendBuf[2] = numbs;												
	
	sendBuf[3] = (uint8_t)(termID>>24);
	sendBuf[4] = (uint8_t)(termID>>16);
	sendBuf[5] = (uint8_t)(termID>>8);
	sendBuf[6] = (uint8_t)(termID>>0);
	
	sendBuf[7] = BCChecks(sendBuf,0,7);
	send_To_CAN(canCh,canids,sendBuf);
}

/*
功能：发送锁车指令到CAN总线
描述：周期500ms 每次接收指令后向CAN总线连续发送5次指令码
*/
static void sendLockCMD(unsigned int canCh,uint8_t numbs)
{
	if(sendLockCMDSign > 0)
	{	
		uint8_t sendBuf[8] = {0};
		int canid = 0;
		switch(gUserDara.carDoorCtrCode)
		{
			case 0: canid = 0x1864A0F0; sendBuf[0] = 4; break;				//远程 - 解锁
			case 1: canid = 0x1866A0F0; sendBuf[0] = 4; break;				//心跳 - 解锁
			case 2: canid = 0x1866A0F0; sendBuf[0] = 1; break;				//心跳 - 心跳锁
			case 3: canid = 0x1864A0F0; sendBuf[0] = 1; break;				//远程 - 一级锁
			case 4: canid = 0x1864A0F0; sendBuf[0] = 2; break;				//远程 - 二级锁
		}												
		sendBuf[2] = numbs;	
		
		sendBuf[7] = BCChecks(sendBuf,0,7);
		send_To_CAN(canCh,canid,sendBuf);		
		sendLockCMDSign--;	
	}
}
/*
功能：发送VIN到CAN总线
周期：500ms一帧，发送完成VIN需3帧
*/
static void sendVIN_To_CAN(void)
{
	static uint8_t sendVINSeril = 0;
	uint8_t sendBuf[8] = {0xFF};
	uint8_t tempVal = 0;

	switch(sendVINSeril)
	{
		case 0:
			sendVINSeril++;
			memcpy(sendBuf,&gSysPara.vinCode[0],8);
			send_To_CAN(1,0x18F301F8,sendBuf);
			break;
		case 1:
			sendVINSeril++;
			memcpy(sendBuf,&gSysPara.vinCode[8],8);
			send_To_CAN(1,0x18F302F8,sendBuf);
			break;
		case 2:
			sendBuf[0] = gSysPara.vinCode[16];
		
			tempVal |= gRealData.locationState<<0;			//定位状态
			tempVal |= (g_tGPS.NS == 'N' ? 0:1)<<1;			//南、北维
			tempVal |= (g_tGPS.EW == 'E' ? 0:1)<<2;			//东、西经
			sendBuf[7] = tempVal;
			send_To_CAN(1,0x18F303F8,sendBuf);
			sendVINSeril = 0;
			break;
	}	
}
/*
功能：发送自检状态到CAN总线
描述：周期200ms
*/
static void sendTerm_SelfStateToCAN(void)
{
	static uint16_t lifeCount = 0;
	uint8_t sendBuf[8] = {0xFF};
	uint8_t temp = 0;
	
	memset(sendBuf,0xFF,8);
	sendBuf[0] = 1;											//远程终端运行状态	 0自检中 1自检成功 2自检失败
	
	temp |= (fun_can_Get_State(BSP_CAN) > 0 ? 0:1) << 0; 					//终端CAN通信掉线故障
	temp |= ((Fun_Gprs_GetSta() <= FUN_GPRS_GET_SIM)? 0:1) << 1;	//GPRS故障
	temp |= ((g_tGPS.antSta == 1) ? 0:1) << 2;										//GPS 故障
	temp |= ((bsp_storage_state() > 0) ? 0:1) << 3;								//SD卡故障
	sendBuf[1] = temp;
	
	sendBuf[2] = 0x01;											//终端注意此处默认为01禁 止发其他值
																					//3-5预留
	sendBuf[6] = 0x0A;											//版本1	
	if(lifeCount > 255)											//life
		lifeCount = 0;	
	sendBuf[7] = lifeCount++;
	
	send_To_CAN(1,0x18F304F8,sendBuf);
}

/*
功能：发送定位信息到CAN总线
描述：周期500ms
*/
static void sendLocationInfoToCAN(void)
{
	uint8_t sendBuf[8] = {0xFF};
	uint32_t dwVal = 0;
	uint8_t Idx = 0;
	//经度
	dwVal = (uint32_t)(gRealData.longd * 1000000);    
	sendBuf[Idx++] = (uint8_t)((dwVal & 0xFF000000)>>24);
	sendBuf[Idx++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
	sendBuf[Idx++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
	sendBuf[Idx++] = (uint8_t)dwVal;
	//纬度
	dwVal = (uint32_t)(gRealData.latd * 1000000);  
	sendBuf[Idx++] = (uint8_t)((dwVal & 0xFF000000)>>24);
	sendBuf[Idx++] = (uint8_t)((dwVal & 0x00FF0000)>>16);
	sendBuf[Idx++] = (uint8_t)((dwVal & 0x0000FF00)>>8);
	sendBuf[Idx++] = (uint8_t)dwVal;
	
	send_To_CAN(1,0x18F305F8,sendBuf);
}

/*
功能：发送终端时间到CAN总线
描述：周期200ms
*/
static void sendTerm_TimeStateToCAN(void)
{
	uint8_t sendBuf[8] = {0xFF};
	uint8_t index = 0;
	
	sendBuf[index++] = g_system_dt.second;					//秒
	sendBuf[index++] = g_system_dt.minute;					//分
	sendBuf[index++] = g_system_dt.hour;						//时
	
	sendBuf[index++] = g_system_dt.day;							//日
	sendBuf[index++] = g_system_dt.month;						//月
	sendBuf[index++] = (uint8_t)(g_system_dt.year - 1985);	//年（偏移+1985，2000-1985 = 15）
	
	send_To_CAN(1,0x18FEE6F8,sendBuf);
}
/*
功能：请求VIN 
描述：开机请求10次，周期500ms
*/
static void request_VIN(unsigned int canCh,uint8_t numbs)
{
	uint8_t sendBuf[8] = {0};

	if(requestVINSing == 0)											//A1 开始请求VIN 
	{
		sendBuf[0] = 0xA1;												//命令字
		send_To_CAN(canCh,0x0C64A001,sendBuf);	
		requestVINSing = 2;
	}
	else																																
	{
		if(isRequestVINCounts > 1)									//A3	正式请求VIN		
		{		
			sendBuf[0] = 0xA3;				
			sendBuf[1] = 0xA1;				
			sendBuf[2] = 0x01;				
			sendBuf[3] = numbs;		
		
			send_To_CAN(canCh,0x0C64A001,sendBuf);	
			isRequestVINCounts--;
		}
		else														//AF  结束VIN请求
		{
			sendBuf[0] = 0xAF;					
			send_To_CAN(canCh,0x0C64A001,sendBuf);	
			isRequestVINCounts--;
		}
	}	
}


/*
功能：用户自定义功能入口
*/
static void SelfFunction(unsigned char canCh)
{
	static uint32_t sendInterVal = 0, sendLockCmdTime = 0,sendHeartBeatTime = 0,sendConnectTime = 0;
	static uint32_t sendVinTime = 0,sendStaTime = 0,sendLocation = 0,sendDevTimTime = 0;
	static uint32_t reqVinTime = 0;
	

	if(upFirstSendLockTime >= 60 && upFirstSendLockTime <= 240 && gUserDara.upDataHeartLock == 1)		//每次升级更新程序，就发5次心跳锁车
	{
		gUserDara.upDataHeartLock = 0;
		saveUserData();
		gUserDara.carDoorCtrCode = 2;								//心跳锁指令
		sendLockCMDSign = 5;
		upFirstSendLockTime = 250;
	}
	//计时
	if(osKernelGetTickCount() - upDataFirstTime >= 500)
	{
		upDataFirstTime = osKernelGetTickCount();	
		upFirstSendLockTime++;										//升级后第一次发送锁车指令的计时
		if(upFirstSendLockTime > 240)
			upFirstSendLockTime = 254;
	}

	if(canCh == 1 && osKernelGetTickCount() - sendInterVal > 10)
	{
		sendInterVal = osKernelGetTickCount();
		if(osKernelGetTickCount() -  sendLockCmdTime >= 500)
		{
			sendLockCmdTime = osKernelGetTickCount();
			sendLockCMD(1,randNums);				//发送锁车指令	500ms发送一次	
		}
		else if(osKernelGetTickCount() -  sendHeartBeatTime >= 500)
		{
			sendHeartBeatTime = osKernelGetTickCount();
			srand(osKernelGetTickCount());
			randNums = rand() % 256;				//随机密钥
			send_LockHeartOrSaveLin(0xF0,1,randNums);//发送锁车心跳信息 0x1860A0F0	500ms 执行一次			
		}
		else if(osKernelGetTickCount() -  sendConnectTime >= 500)
		{
			sendConnectTime = osKernelGetTickCount();
			srand(osKernelGetTickCount());	
			randNums = rand() % 256;				//随机密钥
			send_LockHeartOrSaveLin(0xEE,1,randNums);//发送保持连接信息	0x1861A0F0		0xFF 不带锁车功能，0xEE 带锁车功能	
		}
		
		else if(osKernelGetTickCount() - sendVinTime >= 500)
		{
			sendVinTime = osKernelGetTickCount();
			sendVIN_To_CAN();									//发送VIN到CAN总线  301,302,303
		}
		else if(osKernelGetTickCount() - sendLocation >= 500)
		{
			sendLocation = osKernelGetTickCount();
			sendLocationInfoToCAN();					//发送定位到CAN总线	0x18F305F8
		}
		else if(osKernelGetTickCount() -  reqVinTime >= 500 && isRequestVINCounts > 1)
		{
			reqVinTime = osKernelGetTickCount();
			request_VIN(1,randNums);					//请求车辆VIN  0x0c64a001 500s 每次重启 共请求10次 												
		}
		else if(osKernelGetTickCount() - sendStaTime >= 200)
		{
			sendStaTime = osKernelGetTickCount();
			sendTerm_SelfStateToCAN();				//发送状态到CAN总线	0x18F304F8
		}
		else if(osKernelGetTickCount() - sendDevTimTime >= 200)
		{
			sendDevTimTime = osKernelGetTickCount();
			sendTerm_TimeStateToCAN();				//发送时间到CAN总线	0x18FEE6F8
		}
	}
}








