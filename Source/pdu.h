#ifndef __PDU_H__ 
#define __PDU_H__

#include "stdint.h"
#include "versionManager.h"

#define countof(a) (sizeof(a) / sizeof(*(a)))

#define MAX_URL_LEN 50            //域名最大长度

#define MAX_FAULT_NUM 20   				//故障代码最大总数
#define MAX_EXTERN_DATALEN 500    //扩展数据最大长度

/* 终端参数 */
typedef struct{
	uint8_t store_flag;               //保存标志
	char terminalId[13];              //终端编号
	char scyId[17];										//安全芯片ID
	uint8_t chipKey[65];							//安全芯片公钥
	uint8_t setUpKey[65];							//备案用公钥
	uint8_t realKey[65];							//实时数据用公钥
	uint8_t setUpFlag;								//激活标志 0xAA，已激活 非0xAA 终端未激活
	uint8_t qbActicFlag;							//企标激活标志
	char qbId[21];										//企标设备编号
	uint16_t gpsCorpcode;							//GPS厂商代码
	uint8_t subDevCode;								//设备代码
	uint8_t qbAuthSta;								//锁车授权状态
	uint8_t factoryDate[4];						//出厂年月日BCD码
}TERMINAL_PARA;

/* 应用参数定义 */
typedef struct{
	uint8_t store_flag;               //保存标志
	//版本信息
	char hardWareVer[6];              //硬件版本
	char firmWareVer[6];              //固件版本
	//车辆基础信息
	char vinCode[18];                 //车架号VIN 
	char plateNumber[12];             //车牌号	
	//终端基础参数
	uint8_t can1_used;          			//can1 开关
	uint32_t can1_baudrate;           //can1 波特率
	uint8_t can2_used;                //can2 开关
	uint32_t can2_baudrate;           //can2 波特率
	uint8_t can3_used;                //can3 开关
	uint32_t can3_baudrate;           //can3 波特率
	char apn[32];                     //APN
	uint8_t iccid[21];								//设置ICCID
	uint8_t smsNo[16];								//短信中心号码
	//平台应用参数
	char domain[5][MAX_URL_LEN];  		//备案管理平台域名
	uint32_t port[5];             		//备案管理平台端口号
	uint8_t linkSwitch;								//链路控制，一个为标志一个链路
	uint8_t isDebug;           				//串口打印开关	
	uint8_t carType;									//车辆类型 1:新能源 2:氢燃料 3:混动 4:国四 5:国五 6:国六
	//平台周期控制，初始平台周期控制，后续各链路可单独保存
	uint16_t localSaveInterval;       //本地存储时间周期(ms)
	uint16_t realDataInterval;        //实时信息上报周期(s)
	uint16_t warnRealDataInterval;    //报警时信息上报周期(ms)
	uint16_t heartInterval;           //心跳周期(s)
	uint16_t terminalRespTimeOut;			//终端响应超时时间
	uint16_t platformRespTimeOut;			//平台响应超时时间
	uint16_t nextLoginInterval;				//下一轮登入时间间隔	
	//扩展参数
	uint8_t externPar1;               //扩展参数1
	uint8_t externPar2;               //扩展参数2
}APP_PARA;


//驱动电机总成信息 停车充电过程无效传输该数据
typedef struct _motorData{
	uint8_t motorIdx;                           //驱动电机序号
	uint8_t motorState;                         //驱动电机状态
	int16_t motorCtrTemp;					              //驱动电机控制器温度
	uint16_t motorSpeed;					              //驱动电机转速
	uint8_t motorLoad;													//驱动电机负载百分比	
	float motorTorsion;                         //驱动电机转矩
	int16_t motorTemp;						              //驱动电机温度
	float motorVol;								              //电机控制器输入电压
	float motorCur;								              //电机控制器直流母线电流
}MotorData, *pMotorData;

//可充电储能子系统信息
typedef struct _subSysData{
	char rechargeSysCode[50];  //可充电储能系统编码
	//电压信息
	uint8_t subSysIdx;                          //可充电储能子系统号
	float subSysVol;                            //可充电储能装置电压
	float subSysCur;                            //可充电储能装置电流
	uint16_t singleVolCnt;                      //子系统单体电池总数
	uint16_t singleVolStartIdx;                 //电池起始序号(总电池列表, 从0开始)
	//温度信息
	uint16_t singleTemCnt;                      //子系统温度探针个数
	uint16_t singleTemStartIdx;                 //温度起始序号(总温度列表，从0开始)
}SubSysData, *pSubSysData;

#pragma anon_unions
//诊断状态
typedef union{
	struct{
		uint8_t catalyst:1;														//催化转化器
		uint8_t heatedCatalyst:1;											//加热催化转化器
		uint8_t evaporativeSys:1;											//蒸发系统
		uint8_t secondaryAirSys:1;										//二次空气系统
		uint8_t acSysRefrigerant:1;										//AC系统制冷剂
		uint8_t exhaustGasSensor:1;										//排气传感器
		uint8_t exhaustGasSensorHeater:1;							//排气传感器加热器
		uint8_t egrAndVvtSys:1;												//EGR和Vvt系统
		uint8_t coldStartAidSys:1;										//冷启动辅助系统
		uint8_t boostPressureCtrlSys:1;								//增压压力控制系统
		uint8_t dpf:1;																//DPF监控
		uint8_t scrOrNOxAdsorber:1;										//选择性催化还原系统（SCR）或NOx吸附器
		uint8_t nmhcConvertingCatalyst:1;							//NMHC氧化催化器
		uint8_t misfire:1;														//失火
		uint8_t fuelSys:1;														//燃油系统
		uint8_t comprehensiveComponent:1;							//综合成分
	};
	uint16_t value;
}DiagMonitorState;

//国标协议实时数据
typedef struct _realData{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	//OBD数据
	uint8_t obdDiagProt;                        //OBD诊断协议 0-IOS15765,1-IOS27145,2-SAEJ1939,0xFE-无效
	uint8_t milState;                        		//MIL状态 0-未点亮,1-点亮,3未充电,4充电完成,0xFE异常,0xFF无效
	DiagMonitorState diagSpState;               //诊断支持状态
	DiagMonitorState diagRdyState;							//诊断就绪状态
	char vin[18];																//车架号，这是根据诊断协议读取的
	char softCalId[19];													//软件标定识别号
	char cvn[19];																//标定验证码
	char iuprVal[37];														//IUPR值"一般分母计数器-2字节
/*
点火循环计数器-2字节
NMHC分子计数器-2字节
NMHC分母计数器-2字节
NOx/SCR分子计数器-2字节
NOx/SCR分母计数器-2字节
NOx吸附器分子计数器-2字节
NOx吸附器分母计数器-2字节
PM分子计数器-2字节
PM分母计数器-2字节
排气传感器分子计数器-2字节
排气传感器分母计数器-2字节
EGR and/or VVT分子计数器-2字节
EGR and/or VVT分母计数器-2字节
Boost Pressure分子计数器-2字节
Boost Pressure分母计数器-2字节"	
*/
	
	uint8_t faultCodeCnt;												//故障码总数
	uint32_t faultCode[MAX_FAULT_NUM];       		//可充电储能装置故障代码列表
	
	//数据流信息
//	float speed;									              //车速
	float barometric;														//大气压力
	int16_t engineTorque;												//发动机输出扭矩/实际扭矩
	int16_t frictionTorque;											//摩擦扭矩
	float engineSpeed;													//发动机转速
	float engineFuelFlow;												//发动机燃料流量
	float scrUpperNOxSensor;										//SCR上游NOx传感器输出值
	float scrLowerNOxSensor;										//SCR下游NOx传感器输出值
	float reagentSurplus;												//反应剂余量
	float intakeFlowrate;												//进气量
	float scrInletTemp;													//SCR入口温度
	float scrOutletTemp;												//SCR出口温度
	float dpfPressDiff;													//dpf压差
	int16_t engineCoolantTemp;									//发动机冷却液温度
	float tankLevel;														//油箱液位
//	float totalMileage;						            //累计里程
	
	uint8_t technology;													//技术路线:0:DPF或SCR技术、1:TWC技术无NOx、2:TWC有NOx
	//三元催化技术数据流
	float twcLowerNOxSensor;										//TWC下游NOx传感器输出值
	float twcUpperOxySensor;										//TWC上游Oxy传感器输出值
	float twcLowerOxySensor;										//TWC下游Oxy传感器输出值
	float twcTemp;															//TWC（上游、下游、模拟）温度
	
	//补充数据流
	uint8_t engineTorqueMode;									  //发动机扭矩模式 0000 广播
	float acceleratorVal;				                //加速踏板行程值 F003 广播 确认
	float EngTotalFuelUsed;											//累计油耗 FEE9 请求（部分广播） 确认
	int16_t ureaTankTemp;												//尿素箱温度 FE56 广播 确认
	float actAreaInjectVal;											//实际尿素喷射量 广播 暂未实车报文验证
	uint32_t  totalUreaUsed;										//累计尿素消耗  广播 暂未实车报文验证
	float dpfExhaustTemp;												//DPF排气温度 确认
	//金旅版数据流
	float engFuelRate;													//瞬时油耗 可疑是发动机燃油经济性 FEF2 广播 确认
	float engTotalHoursOfOperation;							//发动机总运行时长, FEE5 请求（部分广播） 确认
	uint16_t	engReferenceTorque;								//发动机最大参考扭矩 FEE3 广播 data[19] data[20] 确认	
	
	//数据流信息
	float speed;									              //车速
	float totalMileage;						              //累计里程
	//车辆位置数据
	uint8_t locationState;				              //定位状态
	double longd;									              //经度
	double latd;									              //纬度
	//整车数据
	uint8_t carState;                           //车辆状态 1启动状态,2熄火,3其他状态,0xFE异常,0xFF无效
	uint8_t chargeState;                        //充电状态 1停车充电,2行驶充电,3未充电,4充电完成,0xFE异常,0xFF无效
	uint8_t operationState;                     //运行模式 1纯电,2混动,3燃油,0xFE异常,0xFF无效
	float total_volt;       			              //总电压
	float total_current;    			              //总电流
	uint8_t soc;									              //soc
	uint8_t dc2dcState;                         //DC-DC状态 1工作,2断开,0xFE异常,0xFF无效
	float brakingVal;						                //制动踏板行程值
	uint8_t stall;                              //档位 Bit0-Bit3:0-空档, 1-1挡, 2-2挡 ... 6-6档, 0xOD-倒档, 0x0E-自动D档, 0x0F-停车P挡; Bit4:1有制动力,0无制动力; Bit5:1有驱动力,0无驱动力
	uint16_t mohm;       					              //绝缘电阻
	//驱动电机数据
	uint8_t motorCnt;                           //驱动电机个数
	MotorData motorData[2];         						//驱动电机总成信息
	//燃料电池数据
	float fuelBatVol;                           //燃料电池电压
	float fuelBatCur;                           //燃料电池电流
	float batFuelConsumption;                   //电池燃料消耗率
	uint16_t fuelBatTemCnt;                     //燃料电池温度探针总数
	int16_t fuelBatTem[120];    								//燃料电池探针温度值
	int16_t maxHydrSysTem;                      //氢系统中最高温度
	float maxHydrSysTemIdx;                   	//氢系统中最高温度探针代号
	uint16_t maxHydrThickness;                  //氢气最高浓度
	uint8_t maxHydrThicknessIdx;                //氢气最高浓度传感器代号
	float maxHydrPressure;                      //氢气最高压力
	uint8_t maxHydrPressureIdx;                 //氢气最高压力传感器代号
	uint8_t dc2dcState_highVol;                 //高压DC/DC状态
	//发动机数据
	uint8_t engineState;                        //发动机状态
	uint16_t crankshaftSpeed;                   //曲轴转速
	float fuelConsumption;                      //燃料消耗率
	//极值数据
	uint8_t maxVolPack_index; 		              //最高电压电池子系统号
	uint16_t maxVol_index; 				              //最高电压电池单体代号
	float max_singleVol;					              //电池单体电压最高值
	uint8_t minVolPack_index;    	              //最低电压电池子系统号 
	uint16_t minVol_index; 				              //最低电压电池单体代号
	float min_singleVol;     			              //电池单体电压最低值
	uint8_t maxTemperPack_index; 	              //最高温度子系统号
	uint16_t maxTemper_index; 			            //最高温度探针序号
	int16_t max_singleTemper;			              //最高温度值
	uint8_t minTemperPack_index;                //最低温度子系统号
	uint16_t minTemper_index; 			            //最低温度探针序号
	int16_t min_singleTemper;                   //最低单温度
	//报警数据
	uint8_t alarmLevel;                         //报警等级
	uint8_t tempDiffAlert;				              //温度差异报警
	uint8_t batHighTempAlert;		                //电池高温报警
	uint8_t batHighVolAlert;	                  //车载储能装置类型过压报警
	uint8_t batLowVolAlert;	                    //车载储能装置类型欠压报警
	uint8_t socLowAlert;					              //SOC低报警
	uint8_t singleBatHighVolAlert;			        //单体电池过压报警
	uint8_t singleBattLowVolAlert;			        //单体电池欠压报警
	uint8_t socHighAlert;					              //SOC过高报警
	
	uint8_t socHopAlert;					              //SOC跳变报警
	uint8_t batNotMatchAlert;                   //可充电储能系统不匹配报警
	uint8_t singleBatPoorConsisAlert;           //电池单体一致性差报警
	uint8_t insulationFailtAlert;	              //绝缘故障
	uint8_t dc2dcTemAlert;                      //DC-DC温度报警
	uint8_t brakingAlert;                       //制动系统报警
	uint8_t dc2dcStateAlert;                    //DC-DC状态报警
	uint8_t motorCtrTemAlert;                   //驱动电机控制器温度报警
	
	uint8_t highPressInterlockStateAlert;       //高压互锁状态报警
	uint8_t motorTempAlert;                     //驱动电机温度报警
	uint8_t batOverCharge;                      //车载储能装置类型过充
	
	uint8_t batFaultCnt;                        //可充电储能装置故障总数N1  不大于MAX_BAT_FAULT_NUM
	uint32_t batFault[MAX_FAULT_NUM];       								//可充电储能装置故障代码列表
	uint8_t motorFaultCnt;                      //驱动电机故障总数N2  不大于MAX_MOTOR_FAULT_NUM
	uint32_t motorFault[MAX_FAULT_NUM];   									//驱动电机故障代码列表
	uint8_t engineFaultCnt;                     //发动机故障总数N3  不大于MAX_ENGINE_FAULT_NUM
	uint32_t engineFault[MAX_FAULT_NUM]; 									//发动机故障代码列表
	uint8_t otherFaultCnt;                      //其它故障总数N4  不大于MAX_OTHER_FAULT_NUM
	uint32_t otherFault[MAX_FAULT_NUM];   									//其它故障代码列表
	
	//可充电储能装置电压数据
	uint8_t subBatSysCnt;                      //可充电储能子系统个数 不大于MAX_BAT_SUBSYS_NUM
	SubSysData subSysData[2];                  //可充电储能子系统电压/温度信息表
	//可充电储能系统编码
	uint8_t rechargeSysCodeLen;                //可充电储能系统编码长度
	float single_vol[600];				             //单体电压总表
	int16_t single_temper[160];		             //探针温度值总表
	//自定义数据
	uint16_t externDataLen;                     //扩展有效数据长度
	uint8_t externData[MAX_EXTERN_DATALEN];     //用户自定义数据
}RealData,*pRealData;

typedef struct _terminalState{
	float batVolt;																//电池电压
	float pwrVolt;																//电源电压
	uint8_t powerState:1;													//外部电源状态
	uint8_t obdState:1;														//OBD状态
	uint8_t gbDebug;															//控制打印国标报文
	uint8_t gpsState:3;					//GPS状态 bit0:模块状态 bit1:天线短路 bit2:短信断路

	uint32_t speed;																//国六车速ID
	uint32_t barometric;													//国六大气压力ID
	uint32_t engineTorque;												//国六发动机转矩ID
	uint32_t engineSpeed;													//国六发动机转速ID
	uint32_t frictionTorque;											//国六摩擦扭矩ID
	uint32_t engineFuelFlow;											//国六燃料流量ID
	uint32_t reagentSurplus;											//国六反应剂余量ID
	uint32_t engineCoolantTemp;										//国六发动机冷却液温度ID
	uint32_t tankLevel;														//国六邮箱液位ID
	uint32_t totalMileage;												//国六累计里程ID
	uint32_t intakeFlowrate;											//国六进气量ID
	uint8_t getCarData;														//国六获取OBD状态
}TerminalState,*pTerminalState;

extern TERMINAL_PARA gFrimPara;         				/* 终端固件参数 */
extern APP_PARA gSysPara;               				/* 应用参数 */
extern RealData gRealData;      	      				/* 实时数据 */
extern TerminalState gTerminalState;						/* 终端状态数据*/
extern uint8_t szMainBuf[2048];

#endif
