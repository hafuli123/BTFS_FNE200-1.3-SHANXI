/*
文件：analyse_Bms_HCXY_STD.c
功能：华晨鑫源
日期：2021/07/15
公司：佛山新源
作者：csj
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
//#include "analyse_Bms_UDS.h"
#include "protocol_GB_EX_SHWX.h"

static char* carType1 = "SHWX_STD_FV2.02";
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
extern data83 data_83;
extern float SOH;
/*
版本说明：
地上铁，不需要终端开启极值运算
TS 			测试版本
S 			指标准版本

*/

//SHDB_STA = 1;

static const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*是否使用CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		500000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/
static void analysisTemper(uint32_t canID,const uint8_t *data);					//解析单体温度
static void analysisVol(uint32_t canID,const uint8_t *data);					//解析单体电压
static void unpackSHWXcan(uint8_t ch,CAN_msg *msg);
const uint8_t CALC_EXTREMUN  = 0;											/*是否计算极值 0:关闭 1:开启*/
static char realVin[18] = {0};

extern uint8_t isChgVIN;
void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType1;
	gRealData.rechargeSysCodeLen = 0;
//	udsInit();
}

void sendSendCycleData(uint8_t ch);

/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析                    
*	返 回 值: 无
*****************************************************************/
    uint32_t spn;
    uint8_t fmi;
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint16_t n16Val = 0;
	uint8_t n8Val = 0,bValTmp = 0;
    uint8_t data = 0;
//	udsRecvData(ch,msg);
	uint32_t positiveMohm = 0;
	uint32_t negativeMohm = 0;
    
//    gRealData.carState = CARSTA_START;
	
	switch(msg->id)
	{
        #if 1
		case 0x18FA2FD0:
			{
					//车辆状态
					n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(1 == n8Val) 
						gRealData.carState = CARSTA_START;
					else if(2 == n8Val) 
						gRealData.carState = CARSTA_STOP;
					else if(3 == n8Val) 
						gRealData.carState = CARSTA_OTHER;
					else if(0xFE == n8Val) 
						gRealData.carState = CARSTA_ABNORMAL;
					else 
						gRealData.carState = CARSTA_NVALID;	
					//运行模式
					n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(1 == n8Val) 
						gRealData.operationState = EV;
					else if(2 == n8Val) 
						gRealData.operationState = EV_FV;
					else if(3 == n8Val) 
						gRealData.operationState = FV;
				//车速
				gRealData.speed = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//设置DCDC状态
				n8Val = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				else if(0xFE == n8Val) 
					gRealData.dc2dcState = DCDC_ABNORMAL;
				else 
					gRealData.dc2dcState = DCDC_INVALID; 
				//档位
				n8Val = calcCanValue(32,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(0x0F == n8Val) 
					gRealData.stall = (bValTmp | PART);
				else if(0x0D == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else  
					gRealData.stall = (bValTmp | DIRVE);
			}
			break;
		case 0x18FA0CF3:
			{
				//充电状态
				n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;
				else if(3 == n8Val || 0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;
				else if(4 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
				//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = calcCanValue(16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
//                gRealData.subSysData[0].singleTemCnt = 0x48;
			}
			break;
		case 0x18FAC917:
			{
				//总里程
				gRealData.totalMileage = calcRealValue(0,32,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F350F3:
			{
				//总电压
				gRealData.total_volt = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
			}
			break;
		case 0x18FA0BF3:
			{
				//总电流
				gRealData.total_current = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;				
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FA2ED0:
			{
				//加速踏板
				gRealData.acceleratorVal = (uint8_t)calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板
				n8Val = calcRealValue(8,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);
				//驱动制动状态
				SET_ACCE_BREAK();
				//驱动电机总数
				gRealData.motorCnt = calcCanValue(42,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
                gRealData.vehiclestatedata.PowerSystemState = calcRealValue(38,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//动力系统状态
                gRealData.vehiclestatedata.AccelerationSensorOutput = calcRealValue(59,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//加速度（碰撞）传感器输出    
			}
			break;
		case 0x18FAA0E2:
			{
				positiveMohm = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				negativeMohm = calcRealValue(32,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(positiveMohm > negativeMohm)
						gRealData.mohm = negativeMohm;
				else 
						gRealData.mohm = positiveMohm;
			}
			break;
		case 0x18FA32D0:
			{
				//驱动电机序号
				gRealData.motorData[0].motorIdx = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);;
				//驱动电机状态
				n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				else if(0xFE == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_ABNORMAL;
				else 
					gRealData.motorData[0].motorState = MOTOR_NVALID; 
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = (int16_t)calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转速
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(24,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[0].motorTorsion = calcRealValue(40,16,1,-10000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FA33D0:
			{
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(16,16,1,-3000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FA34D0:
			{
				//驱动电机序号
				gRealData.motorData[1].motorIdx = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机状态
				n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.motorData[1].motorState = MOTOR_CONSUME;
				else if(2 == n8Val) 
					gRealData.motorData[1].motorState = MOTOR_GENERATION;
				else if(3 == n8Val) 
					gRealData.motorData[1].motorState = MOTOR_OFF;
				else if(4 == n8Val) 
					gRealData.motorData[1].motorState = MOTOR_READY;
				else if(0xFE == n8Val) 
					gRealData.motorData[1].motorState = MOTOR_ABNORMAL;
				else 
					gRealData.motorData[1].motorState = MOTOR_NVALID; 
				//驱动电机控制器温度
				gRealData.motorData[1].motorCtrTemp = (int16_t)calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转速
				gRealData.motorData[1].motorSpeed = (int16_t)calcRealValue(24,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[1].motorTorsion = calcRealValue(40,16,1,-10000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[1].motorTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}			
			break;
		case 0x18FA35D0:
			{	
				//电机控制器输入电压
				gRealData.motorData[1].motorVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[1].motorCur = calcRealValue(16,16,1,-3000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F36BE4:
			{
				//燃料电池电压
				gRealData.fuelBatVol = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//燃料电池电流
				gRealData.fuelBatCur = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
                //燃料电池堆总电压
                gRealData.fuelcellsysdata[0].FuelCellStackTotalVoltage = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
                //燃料电池堆总电流
                gRealData.fuelcellsysdata[0].FuelCellStackTotalCurrent = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FA39D0:
			{
				//燃油消耗率
				gRealData.batFuelConsumption = calcRealValue(0,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//百公里燃料消耗量
				gRealData.fuelConsumption = calcRealValue(16,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//发动机状态
				n8Val = calcRealValue(32,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				if(n8Val == 0)
					gRealData.engineState = ENGINE_STOP;
				else if(n8Val == 1)
					gRealData.engineState = ENGINE_START;
				else if(n8Val == 2)
					gRealData.engineState = ENGINE_ABNORMAL;
				else if(n8Val == 3)
					gRealData.engineState = ENGINE_NVALID;
			}
			break;
		case 0x18FA46E4:
			{
				//燃料电池温度探针总数
				gRealData.fuelBatTemCnt = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//高压DC/DC状态
				n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 0)
					gRealData.dc2dcState_highVol = DCDC_BREAK;
				else if(n8Val == 1)
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else if(n8Val == 2)
					gRealData.dc2dcState_highVol = DCDC_ABNORMAL;
				else if(n8Val == 7)
					gRealData.dc2dcState_highVol = DCDC_INVALID;
			}
			break;
		case 0x18FA47E4:
			{
				//探针温度值
				for(int i=0;i<6;i++)
				{
					gRealData.fuelBatTem[i] = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				}
			}
			break;
		case 0x18F35B89:
			{
				//氢系统中最高温度
				gRealData.maxHydrSysTem = calcRealValue(0,16,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//氢系统中最高温度探针代号
				gRealData.maxHydrSysTemIdx = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
			}
			break;
		case 0x18F35A89:
			{
				//氢气最高浓度
				gRealData.maxHydrThickness = calcRealValue(40,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//氢气最高浓度传感器代号
				gRealData.maxHydrThicknessIdx = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//氢气最高压力
				gRealData.maxHydrPressure = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//氢气最高压力传感器代号
				gRealData.maxHydrPressureIdx = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
                //车载氢系统剩余压力                
                gRealData.onboardhydsysdata.ResidualPressure = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);     
			}
			break;
		case 0x18FA0DF3:
			{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号 
				gRealData.minVolPack_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(16,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FA0EF3:
			{
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针序号
				gRealData.maxTemper_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = (int16_t)calcRealValue(0,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针序号
				gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低单温度
				gRealData.min_singleTemper = (int16_t)calcRealValue(8,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
			}
			break;
            
        case 0x18FECAF3:
        {
            spn = calcRealValue(16,19,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            fmi = calcRealValue(35,5,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            
            if(spn == 520300)
            {
                if(fmi ==15 || fmi ==16)
                gRealData.tempDiffAlert = 1; //温度差异报警
            }
            
            if(spn == 520299)
            {
                if(fmi == 15)
                gRealData.batHighTempAlert = 2;//电池高温报警	
                else if(fmi == 16 || fmi == 0)
                gRealData.batHighTempAlert = 3;
            }
             
            if(spn == 520295)
            {
                if(fmi == 15 || fmi == 16 || fmi ==0)
                //车载储能装置类型过压报警					
                gRealData.batHighVolAlert = 1;    

                if(fmi == 17)
                //车载储能装置类型欠压报警					
                gRealData.batLowVolAlert = 2;
                else if(fmi == 18 || fmi == 1)					
                gRealData.batLowVolAlert = 3;  
            }
            
            if(spn == 520306)
            {
                if(fmi == 18 || fmi == 1)
                //SOC低报警
				gRealData.socLowAlert = 2;
            }
            
            if(spn == 520294)
            {
                if(fmi == 15 || fmi == 16 || fmi == 0)
                 //单体电池过压报警
				gRealData.singleBatHighVolAlert = 1; 
                
                if(fmi == 17)
				//单体电池欠压报警					 
				gRealData.singleBattLowVolAlert = 2;          
                else if(fmi == 18 || fmi ==1)
                gRealData.singleBattLowVolAlert = 3;
            }
            
            if(spn == 520306)
            {
                if(fmi == 13)
                //SOC过高报警						
				gRealData.socHighAlert = 1;
            }
            
            if(spn == 520308)
            {
                if(fmi == 9)
				//SOC跳变报警
				gRealData.socHopAlert = 1;     
            }
            
            if(spn == 520309)
            {
                if(fmi == 13)
                //可充电储能系统不匹配报警
				gRealData.batNotMatchAlert = 1;		 
            }
            
            if(spn == 520597)
           {
               if(fmi == 15 || fmi ==16)
				//电池单体一致性差报警				 
				gRealData.singleBatPoorConsisAlert = 1;	
           }
           
           if(spn == 520350)
           {
               if(fmi == 17)
                //绝缘报警							 
                gRealData.insulationFailtAlert = 2;
               else if(fmi == 18|| fmi == 1)
                gRealData.insulationFailtAlert = 3;
           }							
           
           if(spn == 520325)
           {
               if(fmi == 2)
                //高压互锁状态报警
					gRealData.highPressInterlockStateAlert = 2;		                  
           }
           
            if(spn == 520305)
           {
               if(fmi == 15)
           		//车载储能装置类型过充报警
				gRealData.batOverCharge = 0;					
               else if(fmi == 16)
                gRealData.batOverCharge = 2;					       
               else if(fmi == 0)
                gRealData.batOverCharge = 3;					                  
           }    
           
           if(spn == 520327)
           {
               if(fmi == 16)
               //充电插座超温报警
               pSelfData80->Chargingoverheatedfault = 2;
               else if(fmi == 0)
               pSelfData80->Chargingoverheatedfault = 3;
           }
           
           if(spn == 520352)
           {
               if(fmi = 1)
               pSelfData80->Leakagealarm = 3;   
           }
        }
        break;
        
        case 0x18FECAD0:
        {
           if(spn == 521465)
           {
               if(fmi == 16)
               gRealData.dc2dcTemAlert = 1;//DCDC温度报警
               else if(fmi == 0)
               gRealData.dc2dcTemAlert = 2;
           }
           
           if(spn == 521425)
           {
               if(fmi == 12)
               gRealData.brakingAlert = 1;//制动系统报警
           }
           
           if(spn == 521466)
           {
               if(fmi == 12)
				//DC-DC状态报警
                gRealData.dc2dcStateAlert = 1;
           }
           
           if(spn == 520122)
           {
               if(fmi == 15)
               //驱动电机控制器温度报警		
                gRealData.motorCtrTemAlert = 1;					
               else if(fmi == 16)
                gRealData.motorCtrTemAlert = 2;					       
               else if(fmi == 0)
                gRealData.motorCtrTemAlert = 3;					                  
           }
           
           if(spn == 521122)
           {
               if(fmi == 15)
               //驱动电机控制器温度报警		
                gRealData.motorCtrTemAlert = 1;					
               else if(fmi == 16)
                gRealData.motorCtrTemAlert = 2;					       
               else if(fmi == 0)
                gRealData.motorCtrTemAlert = 3;					                  
           }
           
          if(spn == 521019)
           {
               if(fmi == 15)
               //驱动电机温度报警	
                gRealData.motorTempAlert = 1;					
               else if(fmi == 16)
                gRealData.motorTempAlert = 2;					       
               else if(fmi == 0)
                gRealData.motorTempAlert = 3;					                  
           }  
           
           if(spn == 521119)
           {
               if(fmi == 15)
               //驱动电机温度报警
				gRealData.motorTempAlert = 1;					
               else if(fmi == 16)
                gRealData.motorTempAlert = 2;					       
               else if(fmi == 0)
                gRealData.motorTempAlert = 3;					                  
           }  
            
           if(spn == 521018)
           {
               if(fmi == 0)
                   //电机超速报警
               pSelfData80->Motoroverspeedfault = 2;
           }
           
           

        }
        break;
        
        case 0x18FECA17:
        {
            if(spn == 522022)
            {
                //涉水报警
                if(fmi == 15)
                pSelfData80->Wadingalarm = 1;
            }
        }
        break;
        
		case 0x18FA01F3:
			{
				//子系统单体电池总数
				gRealData.subSysData[0].singleVolCnt = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
//                gRealData.subSysData[0].singleVolCnt = 189;
			}
			break;
		case 0x18FA08F3:
			{
				//解析温度探针
				analysisTemper(msg->id,msg->data);					
			}
			break;
		default:
			{
        if((msg->id & 0xFFFF00FF) == 0x18FA00F3)
				{    
					data = (uint8_t)((msg->id & 0x0000FF00)>>8);
					if(data>=0x04 && data <= 0x07)
					{
						analysisVol(msg->id,msg->data);				
					}
				}			
			}
			break;
            #endif
	}
    
			//	if(NcmTxSuppoted() && osKernelGetTickCount() - recvCarActicTime < 5000)
//		sendSendCycleData(ch);
		unpackSHWXcan(ch,msg);									//企标功能及企标数据
}
void unpackSHWXcan(uint8_t ch,CAN_msg *msg)
{
	uint32_t n32Val = 0;
	uint16_t n16Val = 0;
	float nfloatVal = 0.0;
	uint8_t n8Val = 0;

	switch(msg->id)
	{
//        case 0x18FA38D0:
//		{
//			if(msg->data[0] == 1)
//			memcpy(realVin,&msg->data[1],7);
//			else if(msg->data[0] == 8)
//			memcpy(&realVin[7],&msg->data[1],7);
//			else if(msg->data[0] == 15)
//			memcpy(&realVin[14],&msg->data[1],3);
//			if(memcmp(realVin,gSysPara.vinCode,17) != 0 && strlen(realVin) == 17 )   //&& CheckVin(realVin)
//			{
//				memcpy(gSysPara.vinCode,realVin,17);
//				System_Pare_Save();
//			}
////				memcpy(&getVINTime,&g_system_dt,sizeof(g_system_dt));				//首次获取VIN时间
////				fisrtGetVIN = 1;
//				
//				isChgVIN = 1;																								//检测到VIN变更
//		}
//		break;
        
		case 0x18FAC817:
		{
		   pSelfData81->AirPressure1 = calcRealValue(0,8,5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//气压 1
		   pSelfData81->AirPressure2 = calcRealValue(8,8,5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//气压 2 
		   pSelfData81->BatteryVoltage = calcRealValue(24,8,0.2,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//蓄电池电压
		   pSelfData81->FrontDoorStatus = calcRealValue(46,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//前门状态
		   pSelfData81->MiddleDoorStatus = calcRealValue(48,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//中门状态
		   pSelfData81->RearDoorStatus = 	calcRealValue(50,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//后门状态
		   pSelfData81->RearHatchStatus = calcRealValue(42,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//后舱门状态
		   pSelfData81->HandBrakeSignal = calcRealValue(38,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//手刹信号
			pSelfData89.RemainingOilAmount = calcRealValue(56,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//剩余油量
		}
    break;
        
    
    case 0x18FA2ED0:
    {   			
			 pSelfData81->FootBrakeSignal = calcRealValue(26,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//脚刹信号
		   pSelfData8E.EDC_TurnSign = calcRealValue(46,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//EDC 转向标志
			 pSelfData8E.EDC_WorkingMode = calcRealValue(48,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//EDC 工作模式 
		}
    break;
    case 0x18FAC917:
		{			
				pSelfData81->LeftTurnSignal = calcRealValue(32,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//左转向灯 
				pSelfData81->RightTurnSignal = calcRealValue(34,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//右转向灯
				pSelfData81->PositionLight =  calcRealValue(44,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//位置灯
				pSelfData81->FrontFogLight = calcRealValue(46,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//前雾灯
				pSelfData81->RearFogLamp = calcRealValue(48,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); //后雾灯
				pSelfData81->Wiper = calcRealValue(50,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//雨刮
		    pSelfData81->HighBeamLamp = calcRealValue(36,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//远光灯
		    pSelfData81->NearLightLamp = calcRealValue(40,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//近光灯
		}	
	  break;
		case 0x0CF32AD0:
		{
			  pSelfData8D.DefrostEnable = calcRealValue(58,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		//电除霜使能
		    pSelfData82->PrechargeControlSignal = calcRealValue(12,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//预充控制信号
		    pSelfData82->MainContactorPositiveControlSignal = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//主接触器正控制信号
				pSelfData82->MainContactorNegativeControlSignal = calcRealValue(2,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//主接触器负控制信号
		}
    break;
		case 0xCFF7EEE:
    {
		    pSelfData82->MainContactorFeedbackSignal = calcRealValue(0,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//主接触器反馈信号
			  pSelfData82->MainContactorNegativeFeedbackSignal = calcRealValue(7,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//主接触器负反馈信号
		}
    break;
    case 0xCFF0D27:
		{	
        pSelfData82->EmergencyPoweroffRequest = calcRealValue(44,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//紧急下电请求
    }
    break;
    case 0xCFF0F27:
    {
		    pSelfData82->FaultCode  = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//故障码
			  pSelfData82->WarningCode = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//警告码
			  pSelfData82->DriveSystemFaultCode  = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//驱动系统故障码
		}
    break;		
                
        
    case 0x18FA0BF3:
    {
                pSelfData83.SOH = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//电池包正极绝缘值
//        		pSelfData83->SOH = nfloatVal;//calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//SOH
//                pSelfData83->SOH = calcCanValue(0,32,msg->data,HIGHTLOW_MODE,CAN_MODE);//SOH
				pSelfData83.batteryMaximumAllowableDischargeCurrent = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//电池最大允许放电电流
				pSelfData83.batteryMaximumAllowableChargeCurrent = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);;	//电池最大允许充电电流

		}
		break;
        
		case 0x18FAA0E2:
		{	
				pSelfData83.batteryPackPositiveInsulation = calcCanValue(0,32,msg->data,HIGHTLOW_MODE,CAN_MODE);	//电池包正极绝缘值
				pSelfData83.batteryPackNegativeInsulation = calcCanValue(32,32,msg->data,HIGHTLOW_MODE,CAN_MODE);	//电池包负极绝缘值
		}
		break;
		case 0x18F350F3:
		{	
				pSelfData83.BatteryVoltageInsideMainRelay = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//电池电压（主继电器内侧）
				pSelfData83.BatteryVoltageOutsideMainRelay = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//电池电压（主继电器外侧）
		}
    break;
                
    case 0x18FA11F3:
    {			
			  pSelfData83.AccumulatedChargeCapacity = calcRealValue(0,32,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//累积充电电量
				pSelfData83.AccumulatedDischargeCapacity = calcRealValue(32,32,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 累积放电电量
			  pSelfData8F.ChargeCumulativeNumber = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//充电累计次数
		}			
	  break;
    case 0x18FA12F3:
    {
		    pSelfData83.MotorFeedbackQuantity = calcRealValue(0,32,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//电机回馈电量
		}
    break;
		case 0x188127F3:
		{	
       pSelfData83.BatteryEquilibriumState = calcRealValue(18,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 电池均衡状态		
		}
		break;
		case 0x18FA8CD8:
		{	
				pSelfData84.DCDC_OutputCurrent = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 输出电流
				pSelfData84.DCDC_OutputVoltage = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 输出电压
				pSelfData84.DCDC_InputVoltage = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 输入电压
				pSelfData84.DCDC_HeatSinkTemperature = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 散热器温度
	  }
    break;
      #if 1
		case 0xC079A27:
		{
		   pSelfData84.DCDC_EnableSignal = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 使能信号
			 pSelfData85.AirPump_DCAC_EnableSignal = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//气泵 DC/AC 使能信号
			 pSelfData86.OilPump_DCAC_EnableSignal = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//油泵 DC/AC 使能信号
		}
    break;		
    case 0x18FA82DD:
		{	
     	 pSelfData85.AirPumpDCAC_U_PhaseOutputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//气泵 DC/AC U 相输出电流
	     pSelfData85.AirPumpDCAC_V_PhaseOutputCurrent = pSelfData85.AirPumpDCAC_U_PhaseOutputCurrent ;	//气泵 DC/AC V 相输出电流
	     pSelfData85.AirPumpDCAC_W_PhaseOutputCurrent = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//气泵DC/AC W 相输出电流
	     pSelfData85.AirPump_DCAC_HeatSinkTemperature = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//气泵 DC/AC 散热器温度
       pSelfData8F.AirPumpDCAC_LineVoltage = calcRealValue(32,10,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//气泵 DCAC 线电压			
	  }
		break;
		case 0x18FA78DB:
		{	
			
			pSelfData86.OilPumpDCAC_U_PhaseOutputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//油泵 DC/AC U 相输出电流
			pSelfData86.OilPumpDCAC_V_PhaseOutputCurrent = pSelfData86.OilPumpDCAC_U_PhaseOutputCurrent;	//油泵 DC/AC V 相输出电流
			pSelfData86.OilPumpDCAC_W_PhaseOutputCurrent = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//油泵DC/AC W 相输出电流
			pSelfData86.OilPump_DCAC_HeatSinkTemperature = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//油泵 DC/AC 散热器温度
	    pSelfData8F.OilPumpDCAC_LineVoltage = calcRealValue(32,10,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FAE719:
		{	
			n8Val = calcRealValue(0,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//蒸发风机风速状态
			pSelfData87.AirConditionerLowSpeed = 0;	//空调低速
			pSelfData87.AirConditioningMediumSpeed = 0;	//空调中速
			pSelfData87.AirConditionerHighSpeed = 0;	//空调高速
			if(n8Val == 3)
			{
			   pSelfData87.AirConditionerLowSpeed = 1;	//空调低速
			}
      else if(n8Val == 2)
      {
			   pSelfData87.AirConditioningMediumSpeed = 1;	//空调中速
			}
      else if(n8Val == 1)
      {
			   pSelfData87.AirConditionerHighSpeed = 1;	//空调高速 
			}				
			pSelfData87.AirConditioningHeating = calcRealValue(14,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//空调加热
			pSelfData87.AirConditioningRefrigerationDefrosting1 = calcRealValue(10,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//空调制冷 1 化霜
			pSelfData87.AirConditioningFreshAir = calcRealValue(18,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//空调新风
			pSelfData87.AirConditioningSterilization = calcRealValue(16,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//空调杀菌
			pSelfData87.AirConditioningRefrigerationDefrosting2 = calcRealValue(12,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//空调制冷 2 化霜
			pSelfData87.AirConditioningRefrigeration2 = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 空调制冷 2
			pSelfData87.AirConditioningRefrigeration1 = calcRealValue(8,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//空调制冷 1
		}
    break;		
		case 0x18FAE619:
    {
            pSelfData8F.AirConditioningSettingTemperature = calcRealValue(16,8,0.5,-45,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//空调设定温度 			
			pSelfData87.InsideCarTemperature = calcRealValue(0,8,0.5,-45,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//车内温度
			pSelfData87.OutsideCarTemperature = calcRealValue(8,8,0.5,-45,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//车外温度
            gRealData.vehiclestatedata.AmbientTemperatureOutsideVehicle = calcRealValue(8,8,1,-45,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//车外环境温度
	  }
		break;
		case 0x18FFA898:
		{	
			pSelfData88.LubricationPressureState = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//润滑压力状态
			pSelfData88.LubricatingOilLevelState = calcRealValue(2,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//润滑油位状态
			pSelfData88.LubricatingMotorState = calcRealValue(4,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//润滑电机状态
			pSelfData88.LubricationSystemStatus = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 润滑系统状态
		}
    break;
		case 0x18FE5600:
		{
	      pSelfData89.UreaLevel = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//尿素液位
		}
    break;
		case 0x18FEEE00:
		{
		   pSelfData89.EngineWaterTemperature = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//发动机水温
		}
    break;
    case 0x18FEEF00:
    {
		   pSelfData89.EngineOilPressure = calcRealValue(24,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//发动机机油压力
		}
    break;		
    case 0x18FA2745:
    {
				pSelfData8B.TMS_OperatingStatus = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);;	//TMS 工作状态
				//pSelfData8B->TMS_HighVoltageRelayStatus = calcRealValue(56,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//TMS 高压继电器状态(无数据)
				pSelfData8B.WaterOutletTemperature = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//出水温度
				pSelfData8B.WaterInletTemperature = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 回水温度
				pSelfData8B.TMS_FaultCode = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//TMS 故障码
		}
    break;		
		case 0x18FA36D0:
		{	
    	pSelfData8C.DCDC_InstantaneousPower = calcRealValue(0,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 瞬时功率
			pSelfData8C.SteeringOilPumpInstantaneousPower = calcRealValue(8,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//转向油泵瞬时功率
			pSelfData8C.AirPumpInstantaneousPower = calcRealValue(16,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 气泵瞬时功率
			pSelfData8C.DriveSystemInstantaneousPower  = calcRealValue(24,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 驱动系统瞬时功率
			pSelfData8C.DriveSystemRemainingPower  = calcRealValue(40,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//驱动系统剩余功率
            gRealData.motorData[0].motorLoad = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//电机负荷百分比
		}
		break;
		case 0x18FA37D0:
    {
      pSelfData83.RangeDriving = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//续航里程			
			pSelfData8C.DCDC_Power_Consumption  = calcRealValue(0,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//DC/DC 电耗
			pSelfData8C.PowerConsumptionSteeringOilPump  = calcRealValue(10,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//转向油泵电耗
			pSelfData8C.AirPumpPowerConsumption  = calcRealValue(20,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 气泵电耗
			pSelfData8C.DriveSystemPowerConsumption  = calcRealValue(30,6,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 驱动系统电耗
			pSelfData8C.RangeDriving = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);// 续驶里程		
	  }
		break;
		case 0x18FAEBE3:
		{	
			pSelfData8D.DefrosterHighPressureContactorStatus = calcRealValue(40,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//除霜器内高压接触器状态
			pSelfData8D.FanStatus = calcRealValue(42,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//风机状态
			pSelfData8D.HeatingBodyTemperature  = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//发热体温度
			pSelfData8D.DefrosterEnclosureTemperature = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	// 除霜器外壳温度
			pSelfData8D.DefrosterOutletTemperature = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//除霜器出风温度
			pSelfData8D.HeatingBodyCurrent = calcRealValue(24,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//发热体电流
			pSelfData8D.DefrosterWorkingPower = calcRealValue(32,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//除霜器工作功率
	  }
		break; 
		case 0x18FAE933:
		{
			//1是左前  2是右前  3是左后1  4是左后2  5是右后1  6是右后2
      n8Val = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//轮胎编号1  
			n16Val  = calcRealValue(8,8,16,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			nfloatVal = calcRealValue(16,16,(1/128),-128,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
      if(n8Val == 1)
			{	
        pSelfData8A.LeftFrontTirePressure = n16Val;//左前轮胎压力
        pSelfData8A.LeftFrontTireTemperature	= nfloatVal;//左前轮胎温度		
			}
      else if(n8Val == 2)
      {				
				pSelfData8A.RightFrontTirePressure = n16Val;	//右前轮胎压力
				pSelfData8A.RightFrontTireTemperature = nfloatVal;	// 右前轮胎温度
			}
      else if(n8Val == 3)
      {				
				pSelfData8A.LeftRear1TirePressure = n16Val;	//左后 1 轮胎压力
				pSelfData8A.LeftRear1TireTemperature	= nfloatVal;	//左后 1 轮胎温度
			}
      else if(n8Val == 4)
      {				
				pSelfData8A.LeftRear2TirePressure = n16Val;	//左后 2 轮胎压力
				pSelfData8A.LeftRear2TireTemperature	= nfloatVal;	//左后 2 轮胎温度
			}
      else if(n8Val == 5)
      {				
				pSelfData8A.RightRear1TirePressure = n16Val;	//  右后 1 轮胎压力
				pSelfData8A.RightRear1TireTemperature = nfloatVal;	//右后 1 轮胎温度
			}
      else if(n8Val == 6)
      {				
				pSelfData8A.RightRear2TirePressure = n16Val;	// 右后 2 轮胎压力
				pSelfData8A.RightRear2TireTemperature = nfloatVal;	//右后 2 轮胎温度
			}
			pSelfData8F.LeftFrontExplosionproofTireDeviceStatus	= calcRealValue(48,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//左前防爆胎装置状态
			pSelfData8F.RightFrontExplosionproofTireDeviceStatus	= calcRealValue(49,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//右前防爆胎装置状态
			pSelfData8F.LeftRear1ExplosionproofTireDeviceStatus	= calcRealValue(50,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//左后 1 防爆胎装置状态
			pSelfData8F.LeftRear2ExplosionproofTireDeviceStatus	= calcRealValue(51,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//左后 2 防爆胎装置状态
			pSelfData8F.RightRear1ExplosionproofTireDeviceStatus	= calcRealValue(52,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//右后 1 防爆胎装置状态
			pSelfData8F.RightRear2ExplosionproofTireDeviceStatus	= calcRealValue(53,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//右后 2 防爆胎装置状态
		}
    break;
		case 0x18FAE519:
		{	
            pSelfData8F.AirConditioningInstantaneousPower	= calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//空调瞬时功率	
            gRealData.vehiclestatedata.PowerAirConditioningSystem = calcRealValue(56,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//空调系统实时功率
		}
		break;
		case 0x1CFEAC0B:
		{	
			pSelfData8F.DrontAxleLeftBrakeShoeResidualAmount	= calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//前轴左制动蹄片剩余量
	    pSelfData8F.DrontAxleRightBrakeShoeResidualAmount	= calcRealValue(8,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//前轴右制动蹄片剩余量
	    pSelfData8F.RearAxle1LeftBrakeShoeRemainingAmount	= calcRealValue(16,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//后轴 1 左制动蹄片剩余量
	    pSelfData8F.RearAxle1RightBrakeShoeRemainingAmount	= calcRealValue(24,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//后轴 1 右制动蹄片剩余量
	  }
    break;
    case 0x18FA0CF3:
    { 
		  pSelfData8F.TotalBatteryPacks = calcRealValue(48,5,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//电池包总数  
		  
		}
    break; 		
		case 0x18FAA382:
		{
		   	pSelfData90.DetectorNum = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//探测器编号（电池箱号）
				pSelfData90.FireLevel = calcRealValue(16,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//火警级别
				pSelfData90.SystemStatus = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//系统状态
				pSelfData90.SubvalveControlCommandStatus = calcRealValue(32,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//子阀控制命令状态
		}
    break;
		case 0x18F007E8:
		{	
				pSelfData91.LaneDepartureWarningStatus = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//车道偏离预警状态
				pSelfData91.ForwardCollisionWarningStatus = calcRealValue(2,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//前撞预警状态
				pSelfData91.AEB_WorkingStatusAlert = calcRealValue(4,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//AEB 工作状态警示
				pSelfData91.VehicleWarningSign = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//车辆预警标志
				pSelfData91.FrontVehicleRelativeSpeed = calcRealValue(8,16,(1/128),-250,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//与前车相对速度
				pSelfData91.FrontVehicleRelativeDistance = calcRealValue(24,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//与前车距离
				pSelfData91.AEBS_FaultStatus = calcRealValue(40,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//AEBS 故障状态
				pSelfData91.PrimaryControllerFailure = calcRealValue(42,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//主控制器故障
				pSelfData91.CommunicationFailure = calcRealValue(44,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//通讯故障（与车辆通讯）
				pSelfData91.PedestrianWarningSigns = calcRealValue(46,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//行人预警标志
				pSelfData91.TimeIntervalValue = calcRealValue(48,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//时距数值	
    }
    break;
		case 0x18FAEDE8:
		{	
				pSelfData91.LaneDepartureSystemWorkingStatus = calcRealValue(8,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//车道偏离系统工作状态
				pSelfData91.LaneKeepingSystemWorkingStatus = calcRealValue(11,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//车道保持系统工作状态
				pSelfData91.ImageSensorFault = calcRealValue(14,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//图像传感器故障
				pSelfData91.AdaptiveCruiseSystemOperatingState = calcRealValue(16,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//自适应巡航系统工作状态
				pSelfData91.ForwardCollisionWarningSystemOperatingStatus = calcRealValue(19,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//前向碰撞预警系统工作状态
				pSelfData91.ImageSensorCommunicationFaulty = calcRealValue(22,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//图像传感器通讯故障
				pSelfData91.CollisionMitigationSystemWorkingStatus = calcRealValue(24,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//碰撞缓速系统工作状态
				pSelfData91.ThrottleAntimisstepSystemWorkingStatus = calcRealValue(27,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//油门防误踩系统工作状态
				pSelfData91.AuxiliaryControllerFailure = calcRealValue(30,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//辅助控制器故障
				pSelfData91.ImageSensorWorkingStatus = calcRealValue(32,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//图像传感器工作状态
				pSelfData91.Millimeter_waveRadarOperatingStatus = calcRealValue(35,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//毫米波雷达工作状态
				pSelfData91.AuxiliaryControllerCommunicationFaulty = calcRealValue(38,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//辅助控制器通讯故障
				pSelfData91.Millimeter_waveRadarFaulty = calcRealValue(40,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	//毫米波雷达故障
				pSelfData91.Millimeter_waveRadarCommunicationFaulty = calcRealValue(42,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//毫米波雷达通讯故障		
	      
		}
		break;
		case 0x18F009E8:
		{	
		   pSelfData91.CSVU_self_CheckStatus = calcRealValue(16,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//CSVU 自检状态
	  }
    break;		
      
      /***********************************************上海地标数据*****************************************************/
        case 0x18FA4CE4:
        {
            gRealData.vehiclestatedata.InstantaneousHydrogenConsumption = calcRealValue(0,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//瞬时氢耗
            gRealData.vehiclestatedata.AtmosphericPressure = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//大气压力
        }   
            break;
        
        case 0x18FA6289:
        {
            //车载氢系统剩余氢气质量
            gRealData.onboardhydsysdata.OnboardHydrogenSysSurplusQuality = calcRealValue(0,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
        }
        break;
        
        case 0x18FA49E4:
        {
            //燃料电池系统工作状态
            gRealData.fuelcellsysdata[0].FuelCellSysWorkStatus = calcRealValue(4,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //电堆冷却液出口温度
            gRealData.fuelcellsysdata[0].ReactorCoolantOutletTemperature = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //最小电池测量单元的方差
            gRealData.fuelcellsysdata[0].MinCellMeasuredVarianceCells = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //最小电池测量单元的最低电压
            gRealData.fuelcellsysdata[0].MinBatteryMeasuringVoltage = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //最小电池测量单元的电压的平均值
            gRealData.fuelcellsysdata[0].MinBatteryMeasurPressureAverage = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //电堆温度
            gRealData.hydrogenfuelcelldata.ElectricTem = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
        }
        break;
        
        case 0x18FA4AE4:
        {
            //空气压缩机电机控制器的输入电压
            gRealData.fuelcellsysdata[0].AirCompMotorContrInputVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //空气压缩机电机控制器的输入电流
            gRealData.fuelcellsysdata[0].AirCompMotorContrInputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //燃料电池系统输出动力母线电压
            gRealData.fuelcellsysdata[0].FuelCellSystemVoltage = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //燃料电池系统输出动力母线电流
            gRealData.fuelcellsysdata[0].FuelCellSystemCurrent = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
        }
        break;
//        gRealData.vehiclealarmdata.CollisionSignalState
        
        case 0x18F0090B:
        {
            //方向
            gRealData.directionslope.direction = calcRealValue(0,16,0.0009765625,-31.374,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
        }
        break;
        
        case 0x18FA4BE4:
        {
            //空气压缩机电压
            gRealData.hydrogenfuelcelldata.AirComVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //空气压缩机电流
            gRealData.hydrogenfuelcelldata.AirComCur = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //氢气循环泵电压
            gRealData.hydrogenfuelcelldata.HyCyclePumpVol = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            //氢气循环泵电流
            gRealData.hydrogenfuelcelldata.HyCyclePumpCur = calcRealValue(48,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
        }
        break;
        
        case 0x18FECA89:
        {
            spn = calcRealValue(16,19,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            fmi = calcRealValue(35,5,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            if(spn == 521660)
            {
                if(fmi == 17)
                gRealData.vehiclealarmdata.HydrogenBottleLowPressureAlarm = 1;//氢瓶低压报警
                else if(fmi == 18)
                gRealData.vehiclealarmdata.HydrogenBottleLowPressureAlarm = 2;
                else if(fmi == 1)
                gRealData.vehiclealarmdata.HydrogenBottleLowPressureAlarm = 3;
                
                else if(fmi == 15)
                gRealData.vehiclealarmdata.HydrogenBottleHighPressureAlarm = 1;//氢瓶高压报警
                else if(fmi == 16)
                gRealData.vehiclealarmdata.HydrogenBottleHighPressureAlarm = 2;
                else if(fmi == 0)
                gRealData.vehiclealarmdata.HydrogenBottleHighPressureAlarm = 3;
                
                else
                gRealData.vehiclealarmdata.HydrogenBottleLowPressureAlarm = 0;
                gRealData.vehiclealarmdata.HydrogenBottleHighPressureAlarm = 0;
                    
            }
            
            if(spn == 521658)
            {
                if(fmi == 17)
                gRealData.vehiclealarmdata.HydrogenBottleHighTemperatureAlarm = 1;//氢瓶高温报警
                else if(fmi == 18)
                gRealData.vehiclealarmdata.HydrogenBottleHighTemperatureAlarm = 2;
                else if(fmi == 1)
                gRealData.vehiclealarmdata.HydrogenBottleHighTemperatureAlarm = 3;
                
                else if(fmi == 15)
                gRealData.vehiclealarmdata.HydrogenBottleLowTemperatureAlarm = 1;//氢瓶低温报警
                else if(fmi == 16)
                gRealData.vehiclealarmdata.HydrogenBottleLowTemperatureAlarm = 2;
                else if(fmi == 0)
                gRealData.vehiclealarmdata.HydrogenBottleLowTemperatureAlarm = 3;
                
                else
                gRealData.vehiclealarmdata.HydrogenBottleHighTemperatureAlarm = 0; 
                gRealData.vehiclealarmdata.HydrogenBottleLowTemperatureAlarm = 0; 
            }
            
            if((spn >= 521662 && spn <= 521668) || (spn == 521671))
            {
                if(fmi == 15)
                gRealData.vehiclealarmdata.HighHydrogenConcentrationAlarm = 1;//氢浓度过高报警
                else if(fmi == 16)
                gRealData.vehiclealarmdata.HighHydrogenConcentrationAlarm = 2;
                else if(fmi == 0)
                gRealData.vehiclealarmdata.HighHydrogenConcentrationAlarm = 3;   
                else
                gRealData.vehiclealarmdata.HighHydrogenConcentrationAlarm = 0;                    
            }
        }
        break;
        
        case 0x18FECAE4:
        {
            spn = calcRealValue(16,19,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            fmi = calcRealValue(35,5,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            
            if(fmi == 15)
            gRealData.vehiclealarmdata.FuelCellStackOvertemperatureAlarm = 1;//燃料电池电堆超温报警
            else if(fmi == 16)
            gRealData.vehiclealarmdata.FuelCellStackOvertemperatureAlarm = 2;
            else if(fmi == 0)
            gRealData.vehiclealarmdata.FuelCellStackOvertemperatureAlarm = 3;
            else
            gRealData.vehiclealarmdata.FuelCellStackOvertemperatureAlarm = 0;                
        }
        break;
        
        case 0x18F36AE4:
        {
            n8Val = calcRealValue(8,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            if(n8Val == 2)
            gRealData.vehiclealarmdata.FuelCellOutputPowerLimitAlarm = 1;
            else
            gRealData.vehiclealarmdata.FuelCellOutputPowerLimitAlarm = 0;
    
            if(n8Val>=1 && n8Val<=4)
            gRealData.vehiclealarmdata.FuelCellSystemFailureAlarm = 1;
            else 
            gRealData.vehiclealarmdata.FuelCellSystemFailureAlarm = 0;
            
                
                
        }
        break;       
        #endif
    }	

}	
/*
*********************************************************************************************************
*	函 数 名: calcExtremum
*	功能说明: 解析单体电压数据
*	形    参: canID  	 CANID
*			 			uint8_t  CAN数据
*	返 回 值: 无
*********************************************************************************************************
*/
void analysisVol(uint32_t canID,const uint8_t *data)
{
	int i;
	uint16_t volStartIndex = 0;

    if(data[1] != 0x40 && data[1] != 0x43 && data[1] !=0x46)
    {
        volStartIndex = 63*(data[0]-1)+(data[1]-1);
        for(i=0;i<3;++i)
	{
		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
		{
			gRealData.single_vol[volStartIndex++] = calcRealValue((i*16)+16,16,0.001,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE); 
		}
    }
}
//    volStartIndex = ((data[1] -1)*data[0]);
//	for(i=0;i<3;++i)
//	{
//		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
//		{
//			gRealData.single_vol[volStartIndex++] = calcRealValue((i*16)+16,16,1,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE); 
//		}
//	}
    
}

/*
*********************************************************************************************************
*	函 数 名: calcExtremum
*	功能说明: 解析单体温度CAN数据
*	形    参:  canID  	CANID
*			 			 uint8_t  CAN数据
*	返 回 值: 无
*********************************************************************************************************
*/
void analysisTemper(uint32_t canID,const uint8_t *data)
{
    int i;
	uint16_t temperStartIndex = 0;
    temperStartIndex = ((data[0] - 1)*6);
         for(i=0;i<6;++i)
        {
            if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
            {
                gRealData.single_temper[temperStartIndex++] = calcRealValue((i*8)+16,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE); 
            }
        }
}

//void analysisTemper(uint32_t canID,const uint8_t *data)
//{
//	int i;
//	uint16_t temperStartIndex = 0;

//    if(data[1] == 1)
//    {
//        temperStartIndex = ((data[0] - 1)*8);
//         for(i=0;i<6;++i)
//        {
//            if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
//            {
//                gRealData.single_temper[temperStartIndex++] = calcRealValue((i*8)+16,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE); 
//            }
//        }
//	}
//    else if(data[1] == 7)
//    {
//         temperStartIndex = (((data[0] - 1)*8)+6);
//         for(i=0;i<2;++i)
//        {
//            if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
//            {
//                gRealData.single_temper[temperStartIndex++] = calcRealValue((i*8)+16,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE); 
//            }
//        }
//    }
//}


///*******************************************************************************	
//*	函 数 名: sendTimeToCAN
//*	功能说明: 发送时间到CAN总线		发送间隔200ms
//*	形    参: 无
//*	返 回 值: 无
//*******************************************************************************/
//static uint8_t sendTimeToCAN(uint8_t ch,CAN_msg *msg)
//{
//	static uint32_t sendCanMinTime = 0;
//	static uint8_t sendTimeLife = 0;
//	if(isEnableSendCANData() && osKernelGetTickCount() - sendCanMinTime >= 200)
//	{
//		sendCanMinTime = osKernelGetTickCount();
//		sendTimeLife = (sendTimeLife + 1) % 0xFF;
//		msg->id = 0x346;
//		msg->data[0] = sendTimeLife;
//		msg->data[1] = (uint8_t)(g_system_dt.year >> 0);
//		msg->data[2] = (uint8_t)(g_system_dt.year >> 8);
//		msg->data[3] = g_system_dt.month;
//		msg->data[4] = g_system_dt.day;
//		msg->data[5] = g_system_dt.hour;
//		msg->data[6] = g_system_dt.minute;
//		msg->data[7] = g_system_dt.second;
//		startSendCANData(ch,msg);
//		return 1;
//	}
//	return 0;
//}


///*******************************************************************************	
//*	函 数 名: sendLocalToCAN
//*	功能说明: 发送位置信息到CAN总线		发送间隔200ms
//*	形    参: 无
//*	返 回 值: 无
//*******************************************************************************/
//static uint8_t sendLocalToCAN(uint8_t ch,CAN_msg *msg)
//{
//	static uint32_t sendCanMinTime = 0;
//	if(isEnableSendCANData() && osKernelGetTickCount() - sendCanMinTime >= 200)
//	{
//		sendCanMinTime = osKernelGetTickCount();
//		msg->id = 0x347;
//		msg->data[0] = g_tGPS.PositionOk;
//		msg->data[0] |= (g_tGPS.EW == 'E' ? 1 : (g_tGPS.EW == 'W' ? 2 : 0)) << 1;
//		msg->data[0] |= (g_tGPS.NS == 'S' ? 1 : (g_tGPS.EW == 'N' ? 2 : 0)) << 3;
//		msg->data[1] = (uint16_t)(gRealData.longd * 100) >> 0;
//		msg->data[2] = (uint16_t)(gRealData.longd * 100) >> 8;
//		msg->data[3] = (uint16_t)(gRealData.latd * 100) >> 0;
//		msg->data[4] = (uint16_t)(gRealData.latd * 100) >> 8;
//		msg->data[5] = g_tGPS.SpeedKnots * 0.1852f;
//		msg->data[6] = 0;
//		msg->data[6] = 0 << 0;																													//是否需要限速行车
//		msg->data[6] |= 4;																		//请求VIN
//		msg->data[6] |= 8;							//电池组编号请求信号
//		startSendCANData(ch,msg);
//		return 1;
//	}
//	return 0;
//}

//static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,STANDARD_TYPE,DATA_FRAME};
//void sendSendCycleData(uint8_t ch)
//{
//	sendTimeToCAN(ch,&msg_buf);
//	sendLocalToCAN(ch,&msg_buf);
//}

///*
//功能：发送数据到CAN总线
//描述：
//*/
//static uint32_t sendInterVal = 0;
//const uint8_t MAX_INTERVAL = 30;
//void startSendCANData(uint8_t ch,CAN_msg *msg)
//{
//	if(osKernelGetTickCount()-sendInterVal >= MAX_INTERVAL)
//	{
//		sendInterVal = osKernelGetTickCount();
//		CAN_send(1,msg,100);				
//	}
//}

///*
//功能：查询当前是否允许发送
//描述：
//*/
//uint8_t isEnableSendCANData(void)
//{
//	if(osKernelGetTickCount() - sendInterVal >= MAX_INTERVAL)
//	{
//		return 1;
//	}
//	return 0;
//}

/*
功能：获取车型程序版本
描述：
*/

const char* getCarType(void)
{
	return carType1;
}





