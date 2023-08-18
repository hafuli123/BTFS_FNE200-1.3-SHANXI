//云智通定义文件
#ifndef __DST_DEF_H
#define __DST_DEF_H

//#include "newant_bms.h"
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


//用户数据存储
typedef struct{
	uint8_t store_flag;         //保存标志	
	uint32_t wk_Interval;				//唤醒间隔
	uint8_t wk_SendInterval;		//唤醒后上送数据时间间隔 (单位：10s)
	uint16_t wk_Year;						//唤醒日期
	uint8_t wk_Month;	
	uint8_t wk_Day	;
	uint8_t wk_Hour;
	uint8_t wk_Min;
	uint8_t wk_Sec;		
	
	uint8_t runCMD;							//锁车指令许可位
	uint8_t lockCMD;						//锁车控制:0x55,一级；0x56,二级；0x57,三级；0x58,四级；0x59,五级；0x60,重启一级；0xAA,不限速
	uint8_t changeCMD;					//程序版本切换控制：0x55，锁车版本；0xAA，非锁车版本
	uint8_t	bindingCMD;					//绑定控制：0x55，执行绑定；0x00，默认
	uint8_t doorCMD;						//车门控制：开门:0x55, 关门:0xAA,  默认:0x00;
	uint8_t maintCMD;						//保养提示
	uint8_t payCMD;							//缴费提示
	uint8_t yearCheckCMD;				//年审提示
	uint32_t outLineMile;				//无网络时累计里程
	uint32_t outSiteMile;				//无定位累计里程
	
	float autoChgVolt;					//需补电电压			
	float autoChgOverVolt;			//停止补电电压			
	float underVolt;						//欠压值								
	
	char httpURL[160];								//URL
	char httpUserName[20];						//用户名
	char httpPassWord[20];						//密码
	
	uint32_t outLineTime;				//网络异常计时
	uint8_t isSiteExcute;
}USER_DATA;


/* 地上铁控制指令 */
enum DST_GB_CODE
{
	CMD_LOCKCMD = 0x84,						//锁车控制指令(限速)
	CMD_SOFTVER_CHANGE = 0x85,		//程序版本切换控制：0x55，锁车版本；0xAA，非锁车版本
	CMD_BINDINGCTR = 0x86,				//绑定控制：0x55，执行绑定；0x00，默认
	CMD_DOORCTR	= 0x87,						//车门控制：开门:0x55, 关门:0xAA, 
	CMD_MAINTTIPS = 0x88,					//保养提示
	CMD_PAYTIPS = 0x89,						//缴费提示
	CMD_CHECKTIPS = 0x8A,					//年审提示
	
	CMD_SENDVINCMD = 0x0B,				//上送首次获取VIN
	CMD_CHGCMD = 0x0C,						//上送充电电量
};

/* 地上铁自定义 0A数据 */
typedef struct _selfData0A{
	uint8_t sTboxST				:3;					//Tbox工作状态
	uint8_t sTermBindingST:1;					//终端绑定状态
	uint8_t sQorS_CHGSign	:2;					//车辆快慢充标志
	uint8_t sAirWorkST		:2;					//空调工作状态

	uint8_t sVCUVerST			:1;					//车辆控制器版本状态
	uint8_t sPTCST				:1;					//PTC 工作状态
	uint8_t sPumpFault		:1;					//水泵故障
	uint8_t sVacuumPumpST	:1;					//真空泵状态
	uint8_t sPTCRelayST		:1;					//PTC继电器实际状态
	uint8_t sBatRelayST		:1;					//动力电池加热继电器状态
	uint8_t sQ_CHG_Fault	:1;					//快充继电器粘连故障
	uint8_t sBreakST			:1;					//手刹状态

	uint8_t sRelayST			:2;					//继电器状态
	
	uint8_t sLockCarST;								//锁车状态
	uint8_t sLockCarSign;							//锁车标志
	uint8_t sFaultCT;									//故障数
	uint16_t sCarType;								//车型识别	
	uint8_t sTboxNetCSQ;							//TBOX 网络信号
	uint8_t sAirCurr;									//空调反馈电流
	uint8_t sVacuumValue;							//真空度
	uint8_t sTboxSoftVer[5];					//Tbox固件版本号
	uint8_t sVCUVerNum[8];						//VCU版本号
}SelfData0A;

extern USER_DATA gUserDara;									//用户数据存储
extern const char* getCarType(void);
extern SelfData0A* pSelfData0A;					
extern void unpackDSTcan(uint8_t ch,CAN_msg *msg);			/* 地上铁CAN参数解析 */
extern uint8_t autoChgState;														//主动 补电状态位 			1：终端主动发起补电申请		 0：不发起补电申请
extern uint8_t sAllowAutoChg;														//被动 补电指令许可位  1：VCU反向控制终端禁止补电 0：允许自动补电
extern RTC_INFOR getVINTime;														/* 首次获取VIN的时间 */
extern double glongd;									            	 		//经度
extern double glatd;									              		//纬度
extern uint8_t fisrtGetVIN;															/* 是否时首次获取VIN 1是，0否 */
extern uint8_t getCanLogSwitch;													/* 获取CAN日志开关 默认值 0 指令是1，执行成功变成2 */


extern void saveUserData(void);													/* 保存用户数据 */
extern void startSendCANData(uint8_t ch,CAN_msg *msg);	/* 发送CAN数据接口 */
extern uint8_t isEnableSendCANData(void);								/* 查询CAN数据是否允许发送 */	
extern void sendIntervalData(unsigned char canCh);			//发送周期数据



#endif

