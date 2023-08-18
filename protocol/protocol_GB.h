/*
文件：protocol_GB.h
功能：国标协议的一些解析
日期：2013-06-24
*/
#ifndef __PROTOCOL_GB_H__ 
#define __PROTOCOL_GB_H__

#include "pdu.h"

//CAN解析模式，不同的格式，不同的解析方式
typedef enum
{
    INTEL = 0x01,			//英特尔格式
    MOTOROLA = 0x02		//摩托罗拉格式
}CANANA_MODEL;

//字节高低位
typedef enum
{
    HIGHTLOW = 0x01,	//高位在前地位在后
    LOWHIGHT = 0x02		//低位在前高位在后
}BYTE_MODEL;

//偏移量系数计算模式
typedef enum
{
    FACTOR_OFFSET = 0x01,		//先乘以系数再加上偏移量
    OFFSET_FACTOR = 0x02		//先加上偏移量再乘以系数
}CALC_MODEL;

/*充放电状态*/
typedef enum{
	ABNORMAL = 0xFE,		/*异常*/
	INVALID = 0xFF			/*无效*/
}DataState;

/*运行模式*/
typedef enum{
	EV = 0x01,		/*纯电*/
	EV_FV = 0x02,			/*混动*/
	FV = 0x03,			/*燃油*/
}RunModle;

/*运行模式*/
typedef enum{
	NEUTRAL = 0x00,		/*空挡*/
	REVERSE = 0x0D,		/*倒挡*/
	DIRVE = 0x0E,			/*前进挡*/
	PART = 0x0F				/*P档*/
}Stall;

/*驱动电机状态*/
typedef enum{
	MOTOR_CONSUME = 0x01,				/*耗电*/
	MOTOR_GENERATION = 0x02,		/*发电*/
	MOTOR_OFF = 0x03,						/*关闭*/
	MOTOR_READY = 0x04,					/*准备状态*/
	MOTOR_ABNORMAL = 0xFE,			/*异常*/
	MOTOR_NVALID = 0xFF					/*无效*/
}MotorState;

typedef enum{
	CARSTA_START = 0x01,				/*启动*/
	CARSTA_STOP = 0x02,					/*熄火*/
	CARSTA_OTHER = 0x03,				/*其他*/
	CARSTA_ABNORMAL = 0xFE,			/*异常*/
	CARSTA_NVALID = 0xFF				/*无效*/
}CarState;

typedef enum{
	STOP_CHARGE = 0x01,				/*停车充电*/
	RUN_CHARGE = 0x02,				/*行车充电*/
	NO_CHARGE = 0x03,					/*未充电*/
	CHARGE_FINISH = 0x04,			/*充电完成*/
	CHARGE_ABNORMAL = 0xFE,		/*异常*/
	CHARGE_NVALID = 0xFF			/*无效*/
}ChargeState;

typedef enum{
	DCDC_WORK = 0x01,				/*DCDC工作*/
	DCDC_BREAK = 0x02,			/*DCDC断开*/
	DCDC_ABNORMAL = 0xFE,		/*异常*/
	DCDC_INVALID = 0xFF			/*无效*/
}DCDCState;

typedef enum{
	ENGINE_START = 0x01,				/*启动*/
	ENGINE_STOP = 0x02,					/*熄火*/
	ENGINE_ABNORMAL = 0xFE,			/*异常*/
	ENGINE_NVALID = 0xFF				/*无效*/
}ENGINESTATE;

/********国标上报信息数据项*******
				//车辆状态
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.carState = CARSTA_START;
				else if( == n8Val) 
					gRealData.carState = CARSTA_STOP;
				else if( == n8Val) 
					gRealData.carState = CARSTA_OTHER;
				else if( == n8Val) 
					gRealData.carState = CARSTA_ABNORMAL;
				else 
					gRealData.carState = CARSTA_NVALID;
				//充电状态
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if( == n8Val) 
					gRealData.chargeState = RUN_CHARGE;
				else if( == n8Val) 
					gRealData.chargeState = NO_CHARGE;
				else if( == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
				else if( == n8Val) 
					gRealData.chargeState = CHARGE_ABNORMAL;
				else 
					gRealData.chargeState = CHARGE_NVALID
				//运行模式
				gRealData.operationState = EV;
				//车速
				gRealData.speed = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//总里程
				gRealData.totalMileage = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//总电压
				gRealData.total_volt = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//总电流
				gRealData.total_current = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//设置DCDC状态
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if( == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				else if( == n8Val) 
					gRealData.dc2dcState = DCDC_ABNORMAL;
				else 
					gRealData.dc2dcState = DCDC_INVALID; 
				//档位
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				uint8_t bValTmp = (gRealData.stall & 0xF0);
				if( == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if( == n8Val) 
					gRealData.stall = (bValTmp | PART);
				else if( == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if( == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
				else {
					if(n8Val>=1 && n8Val<=6) 
						gRealData.stall = n8Val;
					else 
						gRealData.stall = INVALID;
				}
				//绝缘电阻
				gRealData.mohm = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//加速踏板
				gRealData.acceleratorVal = (uint8_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,);
				//驱动制动状态
				SET_ACCE_BREAK();


				//驱动电机状态
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_ABNORMAL;
				else 
					gRealData.motorData[0].motorState = MOTOR_NVALID; 
				//驱动电机总数
				gRealData.motorCnt = MOTORCOUNT;
				//驱动电机序号
				gRealData.motorData[0].motorIdx = 1;
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转速
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[0].motorTorsion = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);


				//发动机状态
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.engineState = ENGINE_START;
				else if( == n8Val) 
					gRealData.engineState = ENGINE_STOP;
				else if( == n8Val) 
					gRealData.engineState = ENGINE_ABNORMAL;
				else 
					gRealData.engineState = ENGINE_NVALID;
				//曲线转速
				gRealData.crankshaftSpeed = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//燃油消耗率
				gRealData.fuelConsumption = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号 
				gRealData.minVolPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针序号
				gRealData.maxTemper_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针序号
				gRealData.minTemper_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低单温度
				gRealData.min_singleTemper = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);

				//可充电储能子系统个数 不大于MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = BATT_PACK_COUNT;
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = 1;
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//子系统单体电池总数
				gRealData.subSysData[0].singleVolCnt = BATTERY_COUNT;
				//电池起始序号(总电池列表, 从0开始)
				gRealData.subSysData[0].singleVolStartIdx = 0;
				//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = TEMPER_PROBE_COUNT;
				//温度起始序号(总温度列表，从0开始)
				gRealData.subSysData[0].singleTemStartIdx = 0;

				//最高报警等级
				gRealData.alarmLevel = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//温度差异报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.tempDiffAlert = (n8Val == 0? 0:1);
				//电池高温报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batHighTempAlert = (n8Val == 0? 0:1);
				//车载储能装置类型过压报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batHighVolAlert = (n8Val == 0? 0:1);
				//车载储能装置类型欠压报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batLowVolAlert = (n8Val == 0? 0:1);
				//SOC低报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socLowAlert = (n8Val == 0? 0:1);
				//单体电池过压报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatHighVolAlert = (n8Val == 0? 0:1);
				//单体电池欠压报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBattLowVolAlert = (n8Val == 0? 0:1);
				//SOC过高报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHighAlert = (n8Val == 0? 0:1);
				//SOC跳变报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);
				//可充电储能系统不匹配报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);
				//电池单体一致性差报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatPoorConsisAlert = (n8Val == 0? 0:1);
				//绝缘报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.insulationFailtAlert = (n8Val == 0? 0:1);
				//DC-DC温度报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.dc2dcTemAlert = (n8Val == 0? 0:1);
				//制动系统报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.brakingAlert = (n8Val == 0? 0:1);
				//DC-DC状态报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.dc2dcStateAlert = (n8Val == 0? 0:1);
				//驱动电机控制器温度报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorCtrTemAlert = (n8Val == 0? 0:1);
				//高压互锁状态报警
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.highPressInterlockStateAlert = (n8Val == 0? 0:1);
				//驱动电机温度报警	
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorTempAlert = (n8Val == 0? 0:1);
				//车载储能装置类型过充	
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batOverCharge = (n8Val == 0? 0:1);

				//单体电压
				gRealData.single_vol[] = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体温度
				gRealData.single_temper[] = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体解析 举例
				if(msg->id >=TEMPER_1 && msg->id <=TEMPER_3)
				{
					analysisTemper(msg->id,msg->data);
				}
				else if(msg->id >=VOLGAGE_1 && msg->id <=VOLGAGE_13)
				{
					analysisVol(msg->id,msg->data);
				}
*********************************/

/******车辆状态******/
/*说明 
	val 解析CAN数据得到的值
	start	车辆启动	判别条件值
	stop	熄火	判别条件值
	other   其他状态	判别条件值
	abnormal 异常
****************/
#define SET_CAR_STATE(val,start,stop,other,abnormal) {\
	if(val == start) gRealData.carState = 0x01;\
	else if(val == stop) gRealData.carState = 0x02;\
	else if(val == other) gRealData.carState = 0x03;\
	else if(val == abnormal) gRealData.carState = 0x0FE;\
	else gRealData.carState = INVALID; }
	
#define SET_CAR_STATE1(val,start1,start2,stop,other,abnormal) {\
	if(val == start1 || val == start2) gRealData.carState = 0x01;\
	else if(val == stop) gRealData.carState = 0x02;\
	else if(val == other) gRealData.carState = 0x03;\
	else if(val == abnormal) gRealData.carState = 0x0FE;\
	else gRealData.carState = INVALID; }
	
	
/*******充放电状态*******/
/*说明 
	val 解析CAN数据得到的值
	stopChange	停车充电判 别条件值
	runChange		行驶充电判 别条件值
	noChange		未充电 判别条件值
	finish 			充电完成 判别条件值
	abnormal 异常
*************************/
#define SET_CHARGE_STATE(val,stopChange,runChange,noChange,finish,abnormal) {\
	if(val == stopChange) gRealData.chargeState = 0x01;\
	else if(val == runChange) gRealData.chargeState = 0x02;\
	else if(val == noChange) gRealData.chargeState = 0x03;\
	else if(val == finish) gRealData.chargeState = 0x04;\
	else if(val == abnormal) gRealData.chargeState = 0x0FE;\
	else gRealData.chargeState = INVALID; }

/*说明 
	val 解析CAN数据得到的值
	stopChange1、stopChange2	停车充电判 别条件值
	runChange	行驶充电判 别条件值
	noChange	未充电 判别条件值
	finish 	充电完成 判别条件值
	abnormal 异常
*************************/
#define SET_CHARGE_STATE1(val,stopChange1,stopChange2,runChange,noChange,finish,abnormal) {\
	if(val == stopChange1 || val == stopChange2) gRealData.chargeState = 0x01;\
	else if(val == runChange) gRealData.chargeState = 0x02;\
	else if(val == noChange) gRealData.chargeState = 0x03;\
	else if(val == finish) gRealData.chargeState = 0x04;\
	else if(val == abnormal) gRealData.chargeState = 0x0FE;\
	else gRealData.chargeState = INVALID; }

	/*说明 
	val 解析CAN数据得到的值
	stopChange1、stopChange2	停车充电判 别条件值
	runChange	行驶充电判 别条件值
	noChange1 noChange2	未充电 判别条件值
	finish 	充电完成 判别条件值
	abnormal 异常
*************************/
#define SET_CHARGE_STATE2(val,stopChange1,stopChange2,runChange,noChange1,noChange2,finish,abnormal) {\
	if(val == stopChange1 || val == stopChange2) gRealData.chargeState = 0x01;\
	else if(val == runChange) gRealData.chargeState = 0x02;\
	else if(val == noChange1 || val == noChange2) gRealData.chargeState = 0x03;\
	else if(val == finish) gRealData.chargeState = 0x04;\
	else if(val == abnormal) gRealData.chargeState = 0x0FE;\
	else gRealData.chargeState = INVALID; }

/*******档位数据*******/
/*说明：
	val 解析CAN数据得到的值
	n1,n2,n3	空挡判断依据
	r1,r2,r3	倒挡判断依据
	d1,d2,d3 	前进挡的判断依据
	没有判断依据值为-1
  如果只存在一个判断依据，则数据重复填写，比如说空挡判断依据数值为0，则n1,n2,n3都是0 
	如果档位是1-6档，则这些参数都赋值为-1*/
#define SET_STALL_3(val,n1,n2,n3,p1,p2,p3,r1,r2,r3,d1,d2,d3) {\
	unsigned char bValTmp = (gRealData.stall & 0xF0);\
	if(val == n1||val == n2||val == n3) gRealData.stall = (bValTmp | NEUTRAL);\
	else if(val == p1||val == p2||val == p3) gRealData.stall = (bValTmp | PART);\
	else if(val == r1||val == r2||val == r3) gRealData.stall = (bValTmp | REVERSE);\
	else if(val == d1||val == d2||val == d3) gRealData.stall = (bValTmp | DIRVE);\
	else {if(val>=1 && val<=6) gRealData.stall = val;else gRealData.stall = INVALID;}}
	
#define SET_STALL_2(val,n1,n2,p1,p2,r1,r2,d1,d2) {\
	unsigned char bValTmp = (gRealData.stall & 0xF0);\
	if(val == n1||val == n2) gRealData.stall = (bValTmp | NEUTRAL);\
	else if(val == p1||val == p1) gRealData.stall = (bValTmp | PART);\
	else if(val == r1||val == r2) gRealData.stall = (bValTmp | REVERSE);\
	else if(val == d1||val == d2) gRealData.stall = (bValTmp | DIRVE);\
	else {if(val>=1 && val<=6) gRealData.stall = val;else gRealData.stall = INVALID;}}
	
#define SET_STALL_1(val,n1,p1,r1,d1) {\
	unsigned char bValTmp = (gRealData.stall & 0xF0);\
	if(val == n1) gRealData.stall = (bValTmp | NEUTRAL);\
	else if(val == p1) gRealData.stall = (bValTmp | PART);\
	else if(val == r1) gRealData.stall = (bValTmp | REVERSE);\
	else if(val == d1) gRealData.stall = (bValTmp | DIRVE);\
	else {if(val>=1 && val<=6) gRealData.stall = val;else gRealData.stall = INVALID;}}
	
/*******设置制动踏板值*******/
/*说明：
	val 解析CAN数据得到的值
	bValid 无具体行程值时，制动有效状态的判定条件值，如果有行程值，则bValid = -1
***************************/
#define SET_BREAK_STATE(val,bValid) {\
	if(bValid == -1) {if(val>=0 && val<=101) gRealData.brakingVal = val; \
		else gRealData.brakingVal=INVALID;}\
	else {if(val == bValid) gRealData.brakingVal = 0x65; \
				else gRealData.brakingVal = 0;}}
	
/********设置驱动，制动有效无效**********/
#define SET_ACCE_BREAK() {\
	if(gRealData.brakingVal > 0 && 0xFF != gRealData.brakingVal) gRealData.stall |= 0x10;\
	else gRealData.stall &= 0x2F;\
	if(gRealData.acceleratorVal > 0 && 0xFF != gRealData.acceleratorVal) gRealData.stall |= 0x20;\
	else gRealData.stall &= 0x1F;}
	
/*******设置DCDC状态*******/
/*说明：
	val 解析CAN数据得到的值
	work1,work2值为工作	判别条件值
	break1,break2值为断开 判别条件值
	abnormal 异常
***************************/
#define SET_DCDC_STATE(val,work1,work2,break1,break2,abnormal) {\
	if(val == work1 || val == work2) gRealData.dc2dcState = 0x01;\
	else if(val == break1||val == break2) gRealData.dc2dcState = 0x02;\
	else if(val == abnormal) gRealData.dc2dcState = 0xFE;\
	else gRealData.dc2dcState = INVALID; }
	
/******设置驱动电机状态*****/
/*说明：
	val 解析CAN数据得到的值
	motoridx表示是第几个电机,从0开始
	consu1,consu2		耗电	判别条件值
	gene1 gene2			发电	判别条件值
	off1		关闭状态	判别条件值
	ready1	准备状态	判别条件值	
	abnormal 异常
****************************/
#define SET_MOTOR_STATE(val,motoridx,consu1,consu2,gene1,gene2,off1,ready1,abnormal) {\
	if(val == consu1 || val == consu2) gRealData.motorData[motoridx].motorState = 0x01;\
	else if(val == gene1 || val == gene2) gRealData.motorData[motoridx].motorState = 0x02;\
	else if(val == off1) gRealData.motorData[motoridx].motorState = 0x03;\
	else if(val == ready1) gRealData.motorData[motoridx].motorState = 0x04;\
	else if(val == abnormal) gRealData.motorData[motoridx].motorState = 0xFE;\
	else gRealData.motorData[motoridx].motorState = INVALID; }
	
#endif
	
