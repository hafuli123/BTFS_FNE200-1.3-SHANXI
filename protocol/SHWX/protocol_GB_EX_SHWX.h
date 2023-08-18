#ifndef __SHWX_GBEX_H
#define __SHWX_GBEX_H

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

//自定义 80数据
typedef struct _selfData80{

	uint8_t vcucheckfault;//VCU自检故障
    uint8_t Prechargingfault;//预充故障
    uint8_t Gearfault;       //档位故障
    uint8_t Brakepedalfault; //制动踏板故障
    uint8_t Acceleratorpedalfault; //加速踏板故障
    uint8_t Bmsoffline;             //BMS节点总点掉线
    uint8_t Instrumentoffline;      //仪表节点总点掉线
    uint8_t Maincontactorfault;     //主接触器故障
    uint8_t Batterylowtemalarm;     //电池低温报警
    uint8_t Batteryrechargefault;   //电池回充电流超限报警
    uint8_t Batterydischargecurrentfault;//电池放电电流超限报警
    uint8_t Heatingcontactorfault;   //加热正接触器故障
    uint8_t Mainnegativecontactorfault; //主负接触器故障
    uint8_t Dcchargingcontactorfault; //直流充电接触器
    uint8_t Mainpositivecontactor;      //主正接触器故障
    uint8_t BMSinternalCANcommunicationfault; //BMS内部CAN通讯故障
    uint8_t BMS24Vpowersupplyfault;         //BMS的24V供电故障
    uint8_t batteryinterlockfault;          //电池互锁故障
    uint8_t Chargingoverheatedfault;        //充电插座过温
    uint8_t Motoroverspeedfault;           //电机超速报警
    uint8_t Leakagealarm;                      //漏电报警
    uint8_t Wadingalarm;                   //涉水报警
}SelfData80;

//自定义 81数据 整车数据
typedef struct _selfData81{
	uint8_t AirPressure1;	//气压 1
	uint8_t AirPressure2;	//气压 2
	float BatteryVoltage;	//蓄电池电压
	uint8_t FrontDoorStatus:2;	//前门状态
	uint8_t MiddleDoorStatus:2;	//中门状态
	uint8_t RearDoorStatus:2;	//后门状态
	uint8_t RearHatchStatus:2;	//后舱门状态
	uint8_t HandBrakeSignal:2;	//手刹信号
	uint8_t FootBrakeSignal:2;	//脚刹信号
	uint8_t KeyLocation:3;	//钥匙位置
	uint8_t LeftTurnSignal:2;	//左转向灯
	uint8_t RightTurnSignal:2;	//右转向灯
	uint8_t PositionLight:2;	//位置灯
	uint8_t NearLightLamp:2;	//近光灯
	uint8_t HighBeamLamp:2;	//远光灯
	uint8_t FrontFogLight:2;	//前雾灯
	uint8_t RearFogLamp:2;	//后雾灯
	uint8_t Wiper:2;	//雨刮
	
}SelfData81;

//自定义 82数据 整车控制器数据
typedef struct _selfData82{
	uint8_t PrechargeControlSignal:2;	//预充控制信号
	uint8_t MainContactorPositiveControlSignal:2;	//主接触器正控制信号
	uint8_t MainContactorFeedbackSignal:2;	//主接触器反馈信号
	uint8_t MainContactorNegativeControlSignal:2;	//主接触器负控制信号
	uint8_t MainContactorNegativeFeedbackSignal:2;	//主接触器负反馈信号
	uint8_t EmergencyPoweroffRequest:2;	//紧急下电请求
	uint8_t FaultCode;	//故障码
	uint8_t WarningCode;	//警告码
	uint8_t DriveSystemFaultCode;	//驱动系统故障码
	
}SelfData82;

//自定义 83数据 动力电池数据
typedef struct _selfData83{
    uint16_t RangeDriving;	//续航里程
	float SOH;	//SOH
	float batteryMaximumAllowableDischargeCurrent;	//电池最大允许放电电流
	float batteryMaximumAllowableChargeCurrent;	//电池最大允许充电电流
	float BatteryVoltageInsideMainRelay;	//电池电压（主继电器内侧）
	float BatteryVoltageOutsideMainRelay;	//电池电压（主继电器外侧）
	float AccumulatedChargeCapacity;	//累积充电电量
	float AccumulatedDischargeCapacity;	// 累积放电电量
	float MotorFeedbackQuantity;	//电机回馈电量
	
    uint16_t batteryPackPositiveInsulation;	//电池包正极绝缘值
	uint16_t batteryPackNegativeInsulation;	//电池包负极绝缘值
	uint8_t EmergencyPoweroffRequest:2;	//紧急下电请求
	uint8_t BatteryEquilibriumState:2;	// 电池均衡状态
	
}SelfData83;

typedef struct selfData83
{
    float SOH;	//SOH
}data83;

//自定义 84数据 DCDC 数据
typedef struct _selfData84{
	float DCDC_OutputCurrent;	//DC/DC 输出电流
	float DCDC_OutputVoltage;	//DC/DC 输出电压
	float DCDC_InputVoltage;	//DC/DC 输入电压
	uint8_t DCDC_HeatSinkTemperature;	//DC/DC 散热器温度
	uint8_t DCDC_EnableSignal:1;	//DC/DC 使能信号
	
}SelfData84;

//自定义 85数据 气泵 DC/AC 数据
typedef struct _selfData85{
	float AirPumpDCAC_U_PhaseOutputCurrent;	//气泵 DC/AC U 相输出电流
	float AirPumpDCAC_V_PhaseOutputCurrent;	//气泵 DC/AC V 相输出电流
	float AirPumpDCAC_W_PhaseOutputCurrent;	//气泵DC/AC W 相输出电流
	uint8_t AirPump_DCAC_HeatSinkTemperature;	//气泵 DC/AC 散热器温度
	uint8_t AirPump_DCAC_EnableSignal:1;	//气泵 DC/AC 使能信号
	
}SelfData85;

//自定义 86数据 油泵 DC/AC 数据
typedef struct _selfData86{
	float OilPumpDCAC_U_PhaseOutputCurrent;	//油泵 DC/AC U 相输出电流
	float OilPumpDCAC_V_PhaseOutputCurrent;	//油泵 DC/AC V 相输出电流
	float OilPumpDCAC_W_PhaseOutputCurrent;	//油泵DC/AC W 相输出电流
	uint8_t OilPump_DCAC_HeatSinkTemperature;	//油泵 DC/AC 散热器温度
	uint8_t OilPump_DCAC_EnableSignal:1;	//油泵 DC/AC 使能信号
	
}SelfData86;

//自定义 87数据 空调数据
typedef struct _selfData87{
	uint8_t AirConditionerLowSpeed:1;	//空调低速
	uint8_t AirConditioningMediumSpeed:1;	//空调中速
	uint8_t AirConditionerHighSpeed:1;	//空调高速
	uint8_t AirConditioningHeating:1;	//空调加热
	uint8_t AirConditioningRefrigerationDefrosting1:1;	//空调制冷 1 化霜
	uint8_t AirConditioningFreshAir:1;	//空调新风
	uint8_t AirConditioningSterilization:1;	//空调杀菌
	uint8_t AirConditioningRefrigerationDefrosting2:1;	//空调制冷 2 化霜
	uint8_t AirConditioningRefrigeration2:1;	// 空调制冷 2
	uint8_t AirConditioningRefrigeration1:1;	//空调制冷 1
	float InsideCarTemperature;	//车内温度
	float OutsideCarTemperature;	//车外温度
	
}SelfData87;

//自定义 88数据 集中润滑数据
typedef struct _selfData88{
	uint8_t LubricationPressureState:2;	//润滑压力状态
	uint8_t LubricatingOilLevelState:2;	//润滑油位状态
	uint8_t LubricatingMotorState:2;	//润滑电机状态
	uint8_t LubricationSystemStatus:2;	// 润滑系统状态
	
}SelfData88;

//自定义 89数据 发动机数据
typedef struct _selfData89{
	uint8_t EngineWaterTemperature;	//发动机水温
	uint8_t EngineOilPressure;	//发动机机油压力
	float RemainingOilAmount;	//剩余油量
	uint8_t UreaLevel;	//尿素液位
	
}SelfData89;

//自定义 8A数据 胎压监测数据
typedef struct _selfData8A{
	uint8_t LeftFrontTirePressure;	//左前轮胎压力
	float LeftFrontTireTemperature;	//左前轮胎温度
	uint8_t RightFrontTirePressure;	//右前轮胎压力
	float RightFrontTireTemperature;	// 右前轮胎温度
	uint8_t LeftRear1TirePressure;	//左后 1 轮胎压力
	float LeftRear1TireTemperature;	//左后 1 轮胎温度
	uint8_t LeftRear2TirePressure;	//左后 2 轮胎压力
	float LeftRear2TireTemperature;	//左后 2 轮胎温度
	uint8_t RightRear1TirePressure;	//  右后 1 轮胎压力
	float RightRear1TireTemperature;	//右后 1 轮胎温度
	uint8_t RightRear2TirePressure;	// 右后 2 轮胎压力
	float RightRear2TireTemperature;	//右后 2 轮胎温度
	
}SelfData8A;

//自定义 8B数据 电池冷却系统 TMS 数据
typedef struct _selfData8B{
	uint8_t TMS_OperatingStatus:2;	//TMS 工作状态
	uint8_t TMS_HighVoltageRelayStatus:2;	//TMS 高压继电器状态
	uint8_t WaterOutletTemperature;	//出水温度
	uint8_t WaterInletTemperature;	// 回水温度
	uint8_t TMS_FaultCode;	//TMS 故障码
	
}SelfData8B;

//自定义 8C数据 电耗数据
typedef struct _selfData8C{
	float DCDC_InstantaneousPower;	//DC/DC 瞬时功率
	float SteeringOilPumpInstantaneousPower;	//转向油泵瞬时功率
	float AirPumpInstantaneousPower;	// 气泵瞬时功率
	float DriveSystemInstantaneousPower;	// 驱动系统瞬时功率
	float DriveSystemRemainingPower;	//驱动系统剩余功率
	float DCDC_Power_Consumption;	//DC/DC 电耗
	float PowerConsumptionSteeringOilPump;	//转向油泵电耗
	float AirPumpPowerConsumption;	// 气泵电耗
	float DriveSystemPowerConsumption;	// 驱动系统电耗
	float RangeDriving;	// 续驶里程
	
}SelfData8C;

//自定义 8D数据 电除霜数据
typedef struct _selfData8D{
	uint8_t DefrosterHighPressureContactorStatus:2;	//除霜器内高压接触器状态
	uint8_t FanStatus:2;	//风机状态
	uint8_t DefrostEnable:2;		//电除霜使能
	uint8_t HeatingBodyTemperature;	//发热体温度
	uint8_t DefrosterEnclosureTemperature;	// 除霜器外壳温度
	uint8_t DefrosterOutletTemperature;	//除霜器出风温度
	float HeatingBodyCurrent;	//发热体电流
	float DefrosterWorkingPower;	//除霜器工作功率
	
}SelfData8D;

//自定义 8E数据 电子差速器数据
typedef struct _selfData8E{
	uint8_t EDC_TurnSign;	//EDC 转向标志
	uint8_t EDC_WorkingMode;	//EDC 工作模式
	
}SelfData8E;

//自定义 8F数据 其他补充数据
typedef struct _selfData8F{
	uint16_t ChargeCumulativeNumber;	//充电累计次数
	uint16_t OilPumpDCAC_LineVoltage;	//油泵 DCAC 线电压
	uint16_t AirPumpDCAC_LineVoltage;	//气泵 DCAC 线电压
	uint8_t AirConditioningInstantaneousPower;	//空调瞬时功率
	uint8_t LeftFrontExplosionproofTireDeviceStatus:1;	//左前防爆胎装置状态
	uint8_t RightFrontExplosionproofTireDeviceStatus:1;	//右前防爆胎装置状态
	uint8_t LeftRear1ExplosionproofTireDeviceStatus:1;	//左后 1 防爆胎装置状态
	uint8_t LeftRear2ExplosionproofTireDeviceStatus:1;	//左后 2 防爆胎装置状态
	uint8_t RightRear1ExplosionproofTireDeviceStatus:1;	//右后 1 防爆胎装置状态
	uint8_t RightRear2ExplosionproofTireDeviceStatus:1;	//右后 2 防爆胎装置状态
	float DrontAxleLeftBrakeShoeResidualAmount;	//前轴左制动蹄片剩余量
	float DrontAxleRightBrakeShoeResidualAmount;	//前轴右制动蹄片剩余量
	float RearAxle1LeftBrakeShoeRemainingAmount;	//后轴 1 左制动蹄片剩余量
	float RearAxle1RightBrakeShoeRemainingAmount;	//后轴 1 右制动蹄片剩余量
	uint8_t TotalBatteryPacks;	//电池包总数
	uint8_t AirConditioningSettingTemperature;	//空调设定温度
	
}SelfData8F;

//自定义 90数据 动力电池灭火系统数据
typedef struct _selfData90{
	uint8_t DetectorNum;	//探测器编号（电池箱号）
	uint8_t FireLevel;	//火警级别
	uint8_t SystemStatus;	//系统状态
	uint8_t SubvalveControlCommandStatus;	//子阀控制命令状态
	
}SelfData90;

//自定义 91数据 ADAS 数据
typedef struct _selfData91{
	uint8_t LaneDepartureWarningStatus:2;	//车道偏离预警状态
	uint8_t ForwardCollisionWarningStatus:2;	//前撞预警状态
	uint8_t AEB_WorkingStatusAlert:2;	//AEB 工作状态警示
	uint8_t VehicleWarningSign:2;	//车辆预警标志
	float FrontVehicleRelativeSpeed;	//与前车相对速度
	float FrontVehicleRelativeDistance;	//与前车距离
	uint8_t AEBS_FaultStatus:2;	//AEBS 故障状态
	uint8_t PrimaryControllerFailure:2;	//主控制器故障
	uint8_t CommunicationFailure:2;	//通讯故障（与车辆通讯）
	uint8_t PedestrianWarningSigns:2;	//行人预警标志
	float TimeIntervalValue;	//时距数值
	uint8_t LaneDepartureSystemWorkingStatus:3;	//车道偏离系统工作状态
	uint8_t LaneKeepingSystemWorkingStatus:3;	//车道保持系统工作状态
	uint8_t ImageSensorFault:2;	//图像传感器故障
	uint8_t AdaptiveCruiseSystemOperatingState:3;	//自适应巡航系统工作状态
	uint8_t ForwardCollisionWarningSystemOperatingStatus:3;	//前向碰撞预警系统工作状态
	uint8_t ImageSensorCommunicationFaulty:2;	//图像传感器通讯故障
	uint8_t CollisionMitigationSystemWorkingStatus:3;	//碰撞缓速系统工作状态
	uint8_t ThrottleAntimisstepSystemWorkingStatus:3;	//油门防误踩系统工作状态
	uint8_t AuxiliaryControllerFailure:2;	//辅助控制器故障
	uint8_t ImageSensorWorkingStatus:3;	//图像传感器工作状态
	uint8_t Millimeter_waveRadarOperatingStatus:3;	//毫米波雷达工作状态
	uint8_t AuxiliaryControllerCommunicationFaulty:2;	//辅助控制器通讯故障
	uint8_t Millimeter_waveRadarFaulty:2;	//毫米波雷达故障
	uint8_t Millimeter_waveRadarCommunicationFaulty:2;	//毫米波雷达通讯故障
	uint16_t CSVU_self_CheckStatus;	//CSVU 自检状态
	
}SelfData91;

extern SelfData80* pSelfData80;					
extern SelfData81* pSelfData81;
extern SelfData82* pSelfData82;
extern SelfData83 pSelfData83;
extern SelfData84 pSelfData84;
extern SelfData85 pSelfData85;
extern SelfData86 pSelfData86;
extern SelfData87 pSelfData87;
extern SelfData88 pSelfData88;
extern SelfData89 pSelfData89;					
extern SelfData8A pSelfData8A;
extern SelfData8B pSelfData8B;
extern SelfData8C pSelfData8C;
extern SelfData8D pSelfData8D;
extern SelfData8E pSelfData8E;
extern SelfData8F pSelfData8F;
extern SelfData90 pSelfData90;
extern SelfData91 pSelfData91;


uint16_t extSHDBReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen);

uint16_t extHyFuelCellReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen);


#endif
