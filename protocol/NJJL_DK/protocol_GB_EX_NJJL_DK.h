//云智通定义文件
#ifndef __YZT_DEF_H
#define __YZT_DEF_H

#include "fun_can.h"
#include "bsp_rtc.h"
#include "stdint.h"
#include "cmsis_os2.h"

enum YZT_GB_CODE
{
	CMD_LOCKCAR = 0x83,				 //锁车指令	
	CMD_CARDOORRSP = 0xC0,		 //锁车应答
	CMD_UPAPP = 0xC4,					 //升级程序应答
	CMD_LOCKCARRRSP = 0xC5,		 //锁车应答(锁车结果上报)
};

//自定义 80数据
typedef struct _selfData80{
	uint8_t sCarState;																		//整车状态
	uint8_t sRunModel;																		//车辆运行模式
	uint16_t sRunMileage;																		//续航里程
	uint8_t sPowerAvgConsumption;													//百公里平均电耗
	uint16_t sVCUTorsion;																	//VCU扭矩请求
	
	uint16_t sPhaseCurr;																	//相电流
	
	uint8_t sHighDevState;																//高压附件使能状态
	uint8_t sVCUTouchControlCmd;													//VCU接触器控制指令
	uint8_t sVCUTouchCloseState;													//VCU接触器闭合状态
	uint8_t sVCUFault;																		//VCU故障
	uint8_t sMCUFault;																		//MCU故障
	uint8_t sLifeSign;																		//Life信号
	uint8_t sVCUVersionInfo;															//VCU版本信息
	uint16_t sCarAcceleratedSpeed;													//整车加速度
	uint8_t sCarState1;																	//车辆状态1
	uint8_t sCarFaultState1;														//车辆故障状态1；
	
}SelfData80;

//自定义 81数据
typedef struct _selfData81{
	uint8_t sCHGLinkState;																//充电连接状态
	uint8_t sBatCoolState;																//电池冷却状态
	uint8_t sHeatState;																		//加热状态
	uint8_t sBalanceState;																//均衡状态
	uint8_t sBMSFaultShutHighPower;												//BMS异常状态下请求切断高压
	uint8_t sBMSFaultLevel;																//BMS故障等级(动力电池故障状态)
	
	uint8_t sToucherLinkFault;														//接触器粘连故障
	uint8_t sBranchPreAlarm;															//支路压差报警
	uint8_t sFireLimitFaultAlarm;													//火灾极限故障报警
	uint8_t sBatProtectOut;																//电池放电保护（用于持续小于10A放电2小时切断总负）
	uint8_t sSingleBatOffLinState;												//单体电压采集掉线状态
	uint8_t sSingleTemOffLinState;												//温度采集掉线状态
	
	uint8_t sBatCoolSystemFault; 													//电池冷却系统故障
	uint8_t sHeatFaultAlarmState;													//加热故障报警状态
	uint8_t sBalanceAlarmState;														//均衡报警状态
	uint8_t sPreCHGAlarm;																	//预充电报警
	uint8_t sCHGDevInfoAlarm;															//与充电机通信报警
	uint8_t sBMSControlOffLinAlarm;												//BMS从控掉线报警（针对某箱数据丢失）
	
	uint8_t sCHGCurrentAlarm;															//充电电流报警
	uint8_t sSOCDiffenceAlarm;														//SOC差异报警
	uint8_t sBatLowTemAlarm;															//电池低温报警
	uint8_t sBMSCommunicationFault;												//BMS通讯故障（针对can硬件故障）
	uint8_t sBatSystemOtherFault;													//电池系统其他故障
	
	uint8_t sPoleColumnHighTemAlarm;											//极柱高温报警
	uint8_t sCHGGunHighTemAlarm;													//充电枪高温报警
	uint8_t sOutCurrentAlarm;															//放电电流报警

	uint8_t sSOCHighAlarm;																//SOC高报警
	uint8_t sInsulationAlarmState;												//绝缘监测报警状态
	
	uint8_t sSingleLowVlotAlarm;													//单体欠压报警
	uint8_t sSingleDifVlotAlarm;													//单体电压差异报警
	uint8_t sTemDiffAlarm1;																//温度差异报警1
	uint8_t sSOCLowAlarm1;																//SOC低报警1
	
	uint8_t sBatHighTemAlarm1;														//电池高温报警1
	uint8_t sSingleOverVlotAlarm;													//单体过压报警
	uint8_t sBatsOverVlotAlarm;														//电池组过压报警
	uint8_t sBatsLowVlotAlarm;														//电池组欠压报警
	
	uint8_t sBatFaultNum;																	//动力电池故障码
	uint16_t sBatsCount;																	//电池组总串数
	uint8_t sBatsTemsCount;																//电池组温度点数
	
	uint8_t sMaxLongInCurrent;														//最大可用持续充电电流（5min）
	uint8_t sMaxShortInCurrent;														//最大可用短时充电电流（30s）
	uint8_t sMaxLongOutCurrent;														//最大可用持续放电电流（5min）
	uint8_t sMaxShortOutCurrent;													//最大可用短时放电电流（30s）
	
	uint8_t sBMSTouchControlCMD;													//BMS接触器控制命令
	uint8_t sBMSTouchControlCloseState;										//BMS接触器闭合状态
	
	uint16_t sCHGCounts;																	//充电次数
	uint32_t sBMSAddUpOutPower;															//电池组累计输出能量
	uint32_t sBMSAddUpChgPower;															//电池组累计充电（不含制动回馈）能量
	
	uint16_t sInsulationCheckAllVolt;											//绝缘检测电池总压
	
	uint8_t sCHGPlusesTem1;																//充电抢1正温度
	uint8_t sCHGMinusTem1;																//充电枪1负温度
	uint8_t sCHGPlusesTem2;																//充电抢2正温度
	uint8_t sCHGMinusTem2;																//充电枪2负温度
	
	uint8_t sBatsProductDate_Month;												//电池组生产日期（月）
	uint8_t sBatsProductDate_Year;												//电池组生产日期（年）
	uint8_t sBatsProducer;																//动力电池生产厂家
	uint8_t sBMSLifeSignal;																//BMS life信号
	uint32_t sBMSSoftwareVersion;														//BMS程序版本
}SelfData81;

//自定义 82数据
typedef struct _selfData82{
	uint8_t sFourInOne_State;															//4合1状态
	uint8_t sFourInOne_FaultNum;													//4合1故障码
	uint8_t sFourInOne_BMSToucherState;										//4合1 - BMS接触器状态反馈
	uint8_t sFourInOne_VCUToucherState;										//4合1 - VCU接触器状态反馈
	uint8_t sFourInOne_BMSToucherFaultState;							//4合1 - BMS接触器故障状态
	uint8_t sFourInOne_VCUToucherFaultState;							//4合1 - VCU接触器故障状态
	
	uint16_t sHighOilPump_OutVolt;														//高压油泵输出电压							
	uint8_t sHighOilPump_DCACOutCur;												//高压油泵DC/AC输出电流
	uint8_t sHighOilPump_MoterTem;												//高压油泵电机温度
	uint8_t sHighOilPump_DCACStateAndFault;								//高压油泵DC/AC状态及故障
	uint8_t sHighOilPump_ConverterFaultNum;								//高压油泵变频器故障码
	uint8_t sHighOilPump_motorSpeed;											//高压油泵转速
	uint8_t sHighOilPump_DCACLifeSignal;									//高压油泵DC/AC life信号
	
	uint8_t sDCDC_RealTimeOutCur;														//DC/DC实时输出电流
	uint8_t sDCDCTem;																			//DC/DC本体温度
	uint8_t sDCDCWorkState;																//DCDC工作状态
	uint8_t sDCDCLifeSignal;															//DCDC Life信号
	
	uint16_t sAirPump_DCACOutVolt;												//气泵DC/AC输出电压
	uint8_t sAirPump_DCACOutCur;												//气泵DC/AC输出电流
	uint8_t sAirPump_DCACStateAndFault;										//气泵DC/AC状态及故障
	uint8_t sAirPump_Tem;																	//气泵温度
	uint8_t sAirPump_ConverterFaultNum;										//气泵变频器故障码
	uint8_t sAirPump_motorSpeed;													//气泵转速
	uint8_t sAirPump_DCACLifeSignal;											//气泵DC/AC Life信号
	
	uint16_t sLowOilPump_OutVolt;													//低压油泵输出电压							
	uint8_t sLowOilPump_DCACOutCur;												//低压油泵DC/AC输出电流
	uint8_t sLowOilPump_DCACStateAndFault;								//低压油泵DC/AC状态及故障
	uint8_t sLowOilPump_ConverterFaultNum;								//低压油泵变频器故障码
	uint8_t sLowOilPump_motorSpeed;												//低压油泵转速
	uint8_t sLowOilPump_DCACLifeSignal;										//低压油泵DC/AC life信号
	
}SelfData82;

//自定义 83数据
typedef struct _selfData83{
	uint8_t sLowBatVolt;																	//低压电池电压
	uint8_t sFrontBrakeAirPressure;												//前制动储气筒气压
	uint8_t sRearBrakeAirPressure;												//后制动储气筒气压
	
	uint32_t sAllMileage;																		//总里程
	
	uint8_t sCarState1;																		//车辆状态1
	uint8_t sCarState2;																		//车辆状态2
	uint8_t sCarState3;																		//车辆状态3
	
	uint8_t sInstrumentAlarmState1;												//仪表报警状态1
	uint8_t sInstrumentAlarmState2;												//仪表报警状态2
	uint8_t sInstrumentSoftwareVersion;										//仪表程序版本
}SelfData83;

//自定义 84数据
typedef struct _selfData84{
	uint8_t sBMSCoolWorkMode;															//BMS冷却请求工作模式
	uint8_t sBMSsetoutWaterTem;														//BMS水冷机组出水口（电池入水口）设定温度
	uint8_t sBatsHighestTem;															//电池组最高温度
	uint8_t sBatsLowestTem;																//电池组最低温度
	uint8_t sBatsQueLifeValue;														//电池组请求life值
	uint8_t sHotContrlMode;																//热管理系统工作模式汇总
	uint8_t sInWaterTem;																	//进水温度
	uint8_t sOutWaterTem;																	//出水温度
	uint8_t sCompressorPower;															//压缩机负载比率
	uint8_t sHotContrlFaultNum;														//热管理系统故障码
	uint8_t sHotContrlLifeValue;													//热管理工作life值
}SelfData84;

//自定义 85数据
typedef struct _selfData85{
	uint8_t sAirConditionerOpenCMD;												//空调开关机命令/状态
	uint8_t sAirConditionerSetTem;												//空调设定温度
	uint8_t sAirConditionerRunStall;											//空调风机运行档位
	uint8_t sInCarTem;																		//车内环境温度
	uint8_t sAirConditionerRunMode;												//空调整机运行模式
	uint8_t sOutCarTem;																		//车外环境温度
	uint8_t sAirToucherContrlAndState;										//空调接触器控制以及状态
	uint16_t sAirSystemVolt;															//空调系统母线电压
	uint8_t sAirSystem_PartsRunState;											//空调系统一部件运行状态
	uint8_t sAirSystem_SystemRunState;										//空调系统一系统运行状态
	uint8_t sAirRunTargetHz;															//压缩机目标频率
	uint8_t sAirRunHz;																		//压缩机运行频率
	uint8_t sAirFaultNum;																	//空调故障码
	uint8_t sAirLife;																			//空调life
}SelfData85;

//自定义 86数据
typedef struct _selfData86{
	uint8_t sTirePosition;															//轮胎位置
	uint8_t sTirePressure;																//轮胎压力
	uint16_t sTireTem;																			//轮胎温度
	uint8_t sTireState;																	//状态（轮胎状态）
	uint8_t sPressureValveCheck;												//压力阀检测
}SelfData86;

//自定义 87数据
typedef struct _selfData87{
	uint8_t sFlameArrester_SystemState;										//灭火器系统状态
	uint8_t sBatsNum;																			//电池组号
	uint8_t sSensorBoxState;															//箱体内传感器工作状态
	uint8_t sSensorBoxFault;															//箱体内故障状态等级
	uint8_t sSensorBoxTem;																//箱体内温度
	uint8_t sSensorBoxStartState;													//箱体内灭火器启动状态
	uint8_t sSensorBoxLife;																//LIFE
}SelfData87;

//自定义 88数据
typedef struct _selfData88{
	uint8_t sAheadCarWarning;															//前车碰撞警告
	uint16_t sAheadCarDistance;														//前车距离
	uint16_t sAheadCar_RelativeSpeed;											//相对速度
	uint8_t sLaneWarning;																	//车道偏离警告
	uint8_t sLaneDirection;																//车道偏离方向
	uint8_t sActivateCollisionWarning;										//启动碰撞预警功能
	uint8_t sActivateEmergencyBraking;										//启动紧急制动功能
	uint8_t	sABESystemState;															//AEB系统状态
}SelfData88;

typedef struct{
	/*********************远程锁车相关****************/
	uint8_t carDoorCtrCode;						//车辆控制代码  1:锁车指令 2:解锁指令 其他:无操作
	uint8_t isReturnLockState;				//0xA5:发送应答 0xAA：无效指令
	uint16_t carDoorCtrCount;					//远程锁车流水号 
	uint8_t isHeartLock;							//心跳锁是否开启		0 关闭；1 开启
	/*远程锁车功能参数*/
	uint16_t carDoorCtrRspCode;				//远程锁车应答 0x00:刚接收到平台指令 0xFF:等到
	uint8_t remoteLockState;					//远程锁车状态
	uint8_t heartLockState;						//心跳锁状态
	uint8_t randomKey;								//随机密钥		
}REMOTELOCK_PARA;


extern SelfData80* pSelfData80;					
extern SelfData81* pSelfData81;
extern SelfData82* pSelfData82;
extern SelfData83* pSelfData83;
extern SelfData84* pSelfData84;
extern SelfData85* pSelfData85;
extern SelfData86* pSelfData86;
extern SelfData87* pSelfData87;
extern SelfData88* pSelfData88;
extern REMOTELOCK_PARA gRemoteLockPara;		/* 远程锁车参数 */
extern uint8_t sendLockCMDSign;

extern void unpackYZTcan(uint8_t ch,CAN_msg *msg);

#endif

