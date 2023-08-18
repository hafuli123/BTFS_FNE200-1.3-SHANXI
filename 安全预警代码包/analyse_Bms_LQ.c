#include "fun_can.h"
#include "bsp_sys.h"
#include "protocol_GB.h"

const char* carType  = "SWD_LQ1.0.0";
const uint8_t CAN1_USED_ANA = 1;					/*是否使用CAN1*/
const uint8_t CAN2_USED_ANA = 1;					/*是否使用CAN2*/
const uint8_t CAN3_USED_ANA = 0;					/*是否使用CAN3*/
const uint32_t CAN1_BAUDRATE = 500000;		/*CAN1波特率*/
const uint32_t CAN2_BAUDRATE = 500000	;		/*CAN2波特率*/
const uint32_t CAN3_BAUDRATE = 500000	;		/*CAN3波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = HIGHTLOW;		//数据高地位
const static CANANA_MODEL CAN_MODE = INTEL;					//CAN接收模式，分为Intel格式和Motorola格式
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;	//CAN数据偏移量，系数计算模式

void CAN_ID_Filter_Set()
{
  CAN1->FMR |= 1 << 0;
  CAN1->FA1R &= ~(1 << 0);
  CAN1->FM1R &= ~(1<< 0);
  CAN1->FS1R |= 1<<0;
  CAN1->FFA1R &= ~(1 << 0);
  CAN1->sFilterRegister[0].FR1 = 0X00000000;
  CAN1->sFilterRegister[0].FR2 = 0X00000000;
  CAN1->FA1R |= 1 << 0;
  CAN1->FMR &= 0 << 0;
}

void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
	gRealData.subBatSysCnt = 1;
	gRealData.subSysData[0].subSysIdx = 1;
	gRealData.subSysData[0].singleVolCnt = 104;
	gRealData.subSysData[0].singleTemCnt = 10;
}

static CAN_msg msg_buf = {0x7A7,{ 0x03, 0x22, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,STANDARD_TYPE,DATA_FRAME};
static uint16_t sendIdx = 0;
static const uint16_t idxPool[] = {0x1E00,0x1E02,0x1E05,0x1E87,0x1E88,0x1E8A,0x1E8B,0x1E8C,0x1E8D,0x1E8E,0x1E8F,0x1E90,0x1E92};
static uint32_t sendCanTime = 0;
static uint16_t sendAddr = 0;

void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	gRealData.carState = 1;
	switch(msg->id)
	{
		case 0x7e7:
			{
				if(msg->data[0] >= 4 && msg->data[1] == 0x62)
				{
					uint16_t dataId = ((msg->data[2] << 8) | msg->data[3]);
					//请求完成
					if(sendAddr == dataId)
					{
						sendAddr = 0;
					}
					if(dataId == 0x1E00)
					{
						gRealData.total_volt = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//总电压
					}
					else if(dataId == 0x1E02)
					{
						gRealData.mohm = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//绝缘电阻
					}
					else if(dataId == 0x1E05)
					{
						gRealData.soc = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//SOC
					}
					else if(dataId == 0x1E8E)
					{
						gRealData.total_current = -calcRealValue(32,16,0.1,-500,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//总电流
					}
					else if(dataId == 0x1E92)
					{
						uint8_t n8val = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//充电状态
						if(n8val == 1)
							gRealData.chargeState = 1;//停车充电
						else if(n8val == 0 || n8val == 3)
							gRealData.chargeState = 3;//未充电
						else if(n8val == 2)
							gRealData.chargeState = 4;//充电完成
						else
							gRealData.chargeState = 0xFF;//无效
					}
					else if(dataId == 0x1E87)
					{
						gRealData.max_singleTemper = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最高温度
					}
					else if(dataId == 0x1E88)
					{
						gRealData.min_singleTemper = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最低温度
					}
					else if(dataId == 0x1E8A)
					{
						gRealData.maxTemperPack_index = 1;
						gRealData.maxTemper_index = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最高温度序号
					}
					else if(dataId == 0x1E8B)
					{
						gRealData.minTemperPack_index = 1;
						gRealData.minTemper_index = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最低温度序号
					}
					else if(dataId == 0x1E8C)
					{
						gRealData.max_singleVol = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最高电压
					}
					else if(dataId == 0x1E8D)
					{
						gRealData.min_singleVol = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最低电压
					}
					else if(dataId == 0x1E8F)
					{
						gRealData.maxVolPack_index = 1;
						gRealData.maxVol_index = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最高电压序号
					}
					else if(dataId == 0x1E90)
					{
						gRealData.minVolPack_index = 1;
						gRealData.minVol_index = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//最低电压序号
					}
					else if(dataId >= 0x1E0D && dataId <= 0x1E74)
					{
						gRealData.single_vol[dataId - 0x1E0D] = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);//单体电压
					}
				}
			}
			break;
		default:
			{
				
			}
			break;
	}
	if((os_time_get() - sendCanTime) >= 40 || sendAddr == 0)
	{
		if(os_time_get() - sendCanTime >= 5)
		{
			sendCanTime = os_time_get();
			if(sendIdx < sizeof(idxPool)/2)
			{
				msg_buf.data[2] = idxPool[sendIdx] >> 8;
				msg_buf.data[3] = idxPool[sendIdx] >> 0;
			}
			else if(sendIdx < sizeof(idxPool) / 2 + (0x1E75 - 0x1E0D))
			{
				msg_buf.data[2] = ((sendIdx -  sizeof(idxPool)/2) + 0x1E0D) >> 8;
				msg_buf.data[3] = ((sendIdx -  sizeof(idxPool)/2) + 0x1E0D) >> 0;
			}
			sendAddr = ((msg_buf.data[2] << 8) | msg_buf.data[3]);
			CAN_send(1,&msg_buf,0x0F00);
			sendIdx++;
			sendIdx %= sizeof(idxPool) / 2 + (0x1E75 - 0x1E0D);
		}
	}
}
