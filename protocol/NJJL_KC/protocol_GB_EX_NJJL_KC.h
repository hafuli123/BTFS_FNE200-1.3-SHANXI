/*
文 件：protocol_GB_EX_NJJL_KC.c
功 能：南京金通卡车信协议 - 解析上送自定义扩展数据
日 期: 2021/12/30
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/
#ifndef __NJLL_KC_DEF_H
#define __NJLL_KC_DEF_H

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"
#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_rtc.h"
#include "bsp_ec20.h"
#include "algo_verify.h"
#include "protocol_GB.h"

#include "fun_mon.h"
#include "bsp_gps.h"
#include "bsp_storage.h"
#include "bsp_io.h"
#include "bsp_sys.h"
#include "bsp_power.h"
#include "protocol_GB32960.h"
#include "bsp_storage.h"
#include "rl_fs.h"
#include "bsp_uart_fifo.h"


enum NJJL_GB_CODE
{
	CMD_LOCKCAR = 0x83,								//锁车指令	
	CMD_CARDOORRSP = 0xC0,						//锁车应答
	CMD_UPAPP = 0xC4,									//升级程序应答
	CMD_LOCKCARRRSP = 0xC5,						//锁车应答(锁车结果上报)
};

typedef struct{
	uint8_t store_flag;									//存储标志

	uint8_t carDoorCtrCode;							//车辆控制代码  1:锁车指令 2:解锁指令 其他:无操作
	uint8_t isReturnLockState;					//0xA5:发送应答 0xAA：无效指令
	uint16_t carDoorCtrCount;						//远程锁车流水号 
	uint8_t upDataHeartLock;						//程序更新后默认启动一次 心跳锁指令

	uint16_t carDoorCtrRspCode;					//远程锁车应答 0x00:刚接收到平台指令 0xFF:等到
	uint8_t remoteLockState;						//远程锁车状态
	uint8_t heartLockState;							//心跳锁状态
}USER_DATA;


//自定义B0数据 - 卡车电池状态
typedef struct{
	uint32_t sBatsOutTotalEnergy;				//累计消耗电量
	uint32_t sBatsChgTotalEnergy;				//累计充电能量
	uint8_t sBatsBreakOnceEnergy;				//单次制动回馈能量
	uint8_t sSOH;												//SOH
	uint16_t sBatsPower;								//电池功率
	
}SelfDataB0,*pSelfDataB0;

//自定义B2数据 - 卡车电池报警
typedef struct{
	uint8_t sBMSDTC_CODE1;							//sBMSDTC_CODE1
	uint8_t sBMSDTC_CODE2;							//sBMSDTC_CODE2
	uint8_t sBMSDTC_CODE3;							//sBMSDTC_CODE3
	uint8_t sBMSDTC_CODE4;							//sBMSDTC_CODE4
	uint8_t sBMSDTC_CODE5;							//sBMSDTC_CODE5
	uint8_t sBMSDTC_CODE6;							//sBMSDTC_CODE6
	uint8_t sBMSDTC_CODE7;							//sBMSDTC_CODE7
	uint8_t sBMSDTC_CODE8;							//sBMSDTC_CODE8
	
	uint8_t sBMSCode1;									//BSM补充CODE1
	uint8_t sBMSCode2;									//BSM补充CODE2
	
	uint8_t sDTC_Code;									//DTC_CODE
	
}SelfDataB2,*pSelfDataB2;

//自定义B3数据 - 卡车电池箱体从控数据
typedef struct{
	uint8_t sSlaveControlOnLine_1_8;		//1-8	从控在线状态
	uint8_t sSlaveControlOnLine_9_16;		//9-16	从控在线状态
	uint8_t sSlaveControlOnLine_17_24;	//17-24	从控在线状态
	uint8_t sSlaveControlOnLine_25_32;	//25_32	从控在线状态
	
	uint8_t sBatsCount;									//电池箱体数
	uint8_t sBatSlaveControlCount;			//从控数

}SelfDataB3,*pSelfDataB3;

//自定义B4数据 - 卡车驱动报警数据
typedef struct{
	uint16_t sTCU_FaultCode;						//TCU故障代码
	uint8_t sTCU_OtherFaultCode;				//TCU其他故障代码
	uint8_t sTCU_FaultLevel;						//TCU故障等级
	uint8_t sMoter_FaultLevel;					//驱动电机故障等级
	uint16_t sMoter_DTCCode;						//驱动电机DTC_CODE
	uint16_t sMotor_FaultCount;					//驱动电机故障总数 
	
	uint8_t sMCU_TempAlarm;							//MCU温度报警
	uint8_t sMotor_TempAlarm;						//驱动电机温度报警

}SelfDataB4,*pSelfDataB4;

//自定义B5数据 - 卡车整车报警信息
typedef struct{
	uint8_t sCarFaultLevel;							//整车故障等级
	uint8_t sCarFaultNum1;							//整车故障码1
	uint8_t sCarFaultNum2;							//整车故障码2
	uint8_t sCarFaultNum3;							//整车故障码3
	uint8_t sCarFaultNum4;							//整车故障码4
	uint8_t sCarFaultNum5;							//整车故障码5
	
}SelfDataB5,*pSelfDataB5;

//自定义BF数据 - 换电锁
typedef struct{
	uint8_t sCE_LockST;									//换电锁状态
	uint8_t sCE_DropLockST;							//换电锁锁紧状态
	
}SelfDataBF,*pSelfDataBF;


extern SelfDataB0 *gSelfDataB0;							/*南京金龙卡车自定义数据*/
extern SelfDataB2 *gSelfDataB2;
extern SelfDataB3 *gSelfDataB3;
extern SelfDataB4 *gSelfDataB4;
extern SelfDataB5 *gSelfDataB5;
extern SelfDataBF *gSelfDataBF;

extern uint16_t selfDataB0Pos;
extern uint16_t selfDataB2Pos;
extern uint16_t selfDataB3Pos;
extern uint16_t selfDataB4Pos;
extern uint16_t selfDataB5Pos;
extern uint16_t selfDataBFPos;

extern uint8_t sendLockCMDSign;					//锁车指令发送标志

extern uint16_t upFirstSendLockTime;		//每次程序更新发送心跳锁指令
extern uint32_t upDataFirstTime;				//每次程序更新 发送计时

extern USER_DATA gUserDara;							//用户参数

extern void unpackSelfInit(void);												//企标数据初始化
extern void unpackSelfcan(uint8_t ch,CAN_msg *msg);			//企标数据解析
extern void saveUserData(void);													//保存用户数据

#endif

