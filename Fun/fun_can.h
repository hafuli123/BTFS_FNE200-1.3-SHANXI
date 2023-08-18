#ifndef __FUN_CAN_H
#define __FUN_CAN_H

#include "protocol_GB.h"
#include "bsp_can.h"

#define MAX_CANFAIL 				50							/*CAN最大失败次数，超过次数后，视为CAN断开，大概5秒左右*/

extern const char* CAR_TYPE;								/*车型 */
extern const uint8_t CALC_EXTREMUN;					/*是否计算极值*/

extern void funCanInit(void);
extern void funCanrun(void);
extern uint8_t fun_can_Get_State(uint8_t ch);
extern uint32_t fun_can_Get_recvCnt(uint8_t ch);

/*计算CAN数值*/
/*说明：
	nstartPos:起始位
	nlen:数据长度
	pcanVal:can数据域
	hightLowMode:是否高位在前，如果高位在前需要转换为低位在前
	canMode:can传输格式（英特尔格式，摩托罗格格式）*/
extern unsigned int calcCanValue(unsigned char nstartPos,unsigned char nlen,const unsigned char* pcanVal,
															BYTE_MODEL hightLowMode,CANANA_MODEL canMode);

/*计算CAN实时数值，有偏移量和系数处理*/
/*说明：
	nstartPos:起始位
	nlen:数据长度
	factor:系数
	offset:偏移量
	pcanVal:can数据域
	hightLowMode:是否高位在前，如果高位在前需要转换为低位在前
	canMode:can传输格式（英特尔格式，摩托罗格格式）*/
extern float calcRealValue(unsigned char nstartPos,unsigned char nlen,float factor,float offset,CALC_MODEL calcModel,
														const unsigned char* pcanVal,BYTE_MODEL hightLowMode,CANANA_MODEL canMode);	


typedef struct
{
	uint32_t code;																																									//代码
	uint32_t timeStamp;																																							//时间戳
}ROTACODE;

extern void updateFaultCode(ROTACODE* roataCode,uint8_t maxCnt,uint32_t newCode,uint32_t timeout);		//更新故障码
extern uint8_t checkFaultCode(ROTACODE* roataCode,uint8_t maxCnt,uint32_t code);											//检测是否存在故障码

extern void setCANmask(uint8_t idx,uint32_t id,uint32_t mask,uint8_t format);													//设置CAN过滤，总共27个
extern void iniCanData(void);																																					//初始化CAN数据
extern void iniFvCanData(void);																																				//初始新能源数据
extern void iniEvCanData(void);																																				//初始新能源数据
extern void unpackCAN(uint8_t ch,CAN_msg *msg);																												//解析CAN数据
extern void unpackFvCAN(uint8_t ch,CAN_msg *msg);																											//解析燃油车数据
extern void unpackEvCAN(uint8_t ch,CAN_msg *msg);																											//解析电动车数据
extern void udsInit(uint8_t ch);																																						//UDS诊断初始化
extern void udsProc(void);																																						//UDS诊断处理

#endif
