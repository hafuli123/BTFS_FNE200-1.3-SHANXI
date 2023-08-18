/*
文 件：analyse_Bms_NJJL_DK_STD.c
功 能：南京金龙通信协议 - 解析国标数据
日 期: 2021/12/9
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/

#include "protocol_GB.h"
#include "NJJL_DK/protocol_GB_EX_NJJL_DK.h"

const char* carType = "F4_NJJL_DaKe_V1.01";

const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 1;											/*是否计算极值 0:关闭 1:开启*/

void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
}
/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val = 0;
	uint16_t n16Val = 0;
	uint32_t n32Val = 0;
	
	uint8_t carSt = 0;
	switch(msg->id)
	{
		case 0x18F101D0:
			{		
				uint8_t bValTmp = 0;	
				//车辆状态
				n8Val = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);								
				if(1 == n8Val)
					gRealData.carState = CARSTA_START;													//1		启动(车辆启动\ACC唤醒\充电唤醒)
				else if(0 == n8Val)
					gRealData.carState = CARSTA_STOP;														//0		熄火
				else
					gRealData.carState = CARSTA_OTHER;
				//车速
				gRealData.speed = calcRealValue(8,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				

				//档位					
				n8Val = calcCanValue(24,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val  )		//或上0，运算时高位用0补齐
					gRealData.stall = (bValTmp | NEUTRAL); 													//0		N	
				else if(1 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);													//1		R	
				else
					gRealData.stall = (bValTmp | DIRVE);														//其他		D
				//else if(15 == n8Val  )
				//	gRealData.stall = (bValTmp | PART);														//0001		P
				
				gRealData.operationState= calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F40217:
			{
				//累计里程
				gRealData.totalMileage = calcRealValue(0,32,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18F201F3:
			{
				//SOC
				gRealData.soc = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//总电压
				gRealData.total_volt = calcRealValue(8,16,0.02,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//总电流
				gRealData.total_current = calcRealValue(24,16,0.1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = gRealData.total_current;	
				//单体电池总数
				n16Val = calcCanValue(40,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < countof(gRealData.single_vol))
					gRealData.subSysData[0].singleVolCnt = n16Val;
			}
			break;

		case 0x18F103D0:
			{								
				//加速踏板行程值		
				gRealData.acceleratorVal = calcRealValue(24,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板状态			
				gRealData.brakingVal = calcRealValue(32,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动制动状态
				SET_ACCE_BREAK();	
			}
			break;
		case 0x18FF2B49:		//【MOTOROLA格式】
			{
				//绝缘电阻
				gRealData.mohm = calcRealValue(8,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);									
				
				//绝缘报警							 
				n8Val = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.insulationFailtAlert = 0;
				else if(1 == n8Val)
					gRealData.insulationFailtAlert = 2;
				else if(2 == n8Val)
					gRealData.insulationFailtAlert = 3;		
			}
			break;
		case 0x18F501F0:
			{
				//驱动电机状态
				n8Val = calcCanValue(42,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( 4== n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;							//4 关闭
				else if(9 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;					//9	耗电
				else if(10 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;				//10	发电
				else if(8 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;						//8	准备	
				//驱动电机温度						
				gRealData.motorData[0].motorTemp = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机控制器温度			
				gRealData.motorData[0].motorCtrTemp = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);								
				//电机控制器输入电压			
				gRealData.motorData[0].motorVol = calcRealValue(16,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F502F0:
			{
				//驱动电机转速							
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩						
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流	
				gRealData.motorData[0].motorCur = calcRealValue(48,16,0.1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}	
			break;
		case 0x18F205F3:
			{
				//充电状态
				n8Val = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				if(0 == n8Val)
					gRealData.chargeState = NO_CHARGE;						//0 未充电
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;					//1 停车充电				
				//else if(2 == n8Val) 
				//	gRealData.chargeState = RUN_CHARGE;						//2	行车充电
				//else if(4 == n8Val) 
				//	gRealData.chargeState = CHARGE_FINISH;  				//4 充电完成				
				//温度差异报警								
				n8Val = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 3 || n8Val == 2)
					gRealData.tempDiffAlert = 2;
				else 
					gRealData.tempDiffAlert = 0;
				
				//电池高温报警						
				n8Val = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 3)
					gRealData.batHighTempAlert = 3;
				else 
					gRealData.batHighTempAlert = 0;

				//车载储能装置类型过压报警					
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 3)
					gRealData.batHighVolAlert = 3;
				else 
					gRealData.batHighVolAlert = 0;	
 								
				//车载储能装置类型欠压报警					
				n8Val = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				if(n8Val == 3)
					gRealData.batLowVolAlert = 3;
				else 
					gRealData.batLowVolAlert = 0;					
			
				//单体电池过压报警
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 3)
					gRealData.singleBatHighVolAlert = 3;
				else 
					gRealData.singleBatHighVolAlert = 0; 
				
				//单体电池欠压报警					 
				n8Val = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 3)
					gRealData.singleBattLowVolAlert = 3;
				else 
					gRealData.singleBattLowVolAlert = 0; 
				
				//电池单体一致性差报警				 
				n8Val = calcCanValue(18,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 3 || n8Val == 2)
					gRealData.singleBatPoorConsisAlert = 2;
				else 
					gRealData.singleBatPoorConsisAlert = 0; 					
			
				//车载储能装置类型过充报警		
				n8Val = calcCanValue(28,2,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				if(n8Val == 3)
					gRealData.batOverCharge = 3;
				else 
					gRealData.batOverCharge = 0; 				
			}
			break;
		case 0x0CF602A1:
			{
				//DCDC状态	
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val  )
				{
					gRealData.dc2dcState = DCDC_WORK;																//1		工作；
				}
				else if(0 == n8Val)
				{
					gRealData.dc2dcState = DCDC_BREAK;																//0		不工作	
				}
			}
			break;
		case 0x187F17F3:
			{
				//最高电压电池子系统号	
				gRealData.maxVolPack_index = 1;				
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(0,16,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号	 
				gRealData.minVolPack_index = 1;	
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(32,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(48,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);								
			}
			break;
		case 0x185F17F3:
			{
				//最高温度子系统号
				gRealData.maxTemperPack_index = 1;			
				//最高温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//最低温度子系统号
				gRealData.minTemperPack_index = 1;					
				//最低温度探针子系统代号
				gRealData.minTemper_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);								
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				

				//可充电储能温度探针个数
				n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				if(n8Val < countof(gRealData.single_temper))
					gRealData.subSysData[0].singleTemCnt = n8Val;
			}
			break;
		default: 
			{
				int i;
				//根据协议计算得到电压位置        
				uint16_t startIndex,temps;
				if((msg->id & 0xFF00FFFF) == 0x180017F3)
				{
					if(((msg->id & 0x00FF0000) >> 16) >=0x80 && ((msg->id & 0x00FF0000) >> 16) <= 0xAF)
					{
						temps = ((msg->id & 0x00FF0000)>>16) - 0x80;
						startIndex = temps * 4;
						/******根据协议计算*******/
						for(i=0;i<4;++i)
						{
							if(startIndex < gRealData.subSysData[0].singleVolCnt)
							{
								gRealData.single_vol[startIndex++] = calcRealValue(i*16,16, 0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
							}
						}								
					}
					else if(((msg->id & 0x00FF0000) >> 16) >= 0x60 && ((msg->id & 0x00FF0000) >> 16) <= 0x63)
					{
						temps = ((msg->id & 0x00FF0000)>>16) - 0x60;
						startIndex = temps * 8;
						/******根据协议计算*******/
						for(i=0;i<8;++i)
						{
							if(startIndex < gRealData.subSysData[0].singleTemCnt)
							{
								gRealData.single_temper[startIndex++] = calcRealValue(i*8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
							}
						}
					}
				}
			}
			break;
	}
	
//	if(gTerminalState.accState == 1)
//		gRealData.carState = CARSTA_START;					//1		启动(车辆启动\ACC唤醒\充电唤醒)
//	else if(gTerminalState.accState == 0)
//		gRealData.carState = CARSTA_STOP;					//0		熄火
	
	unpackYZTcan(ch,msg);
}

