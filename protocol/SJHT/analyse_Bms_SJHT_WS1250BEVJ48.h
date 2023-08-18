/*
文件：analyse_Bms_FC_FSQ3310DEFCEV.h
功能：飞驰FC_FSQ3310DEFCEV车型协议
日期：2021/5/18
公司：北理新源(佛山)信息科技有限公司
作者：LGC
*/
#ifndef __ANALYSE_BMS_H
#define __ANALYSE_BMS_H

#include <RTX_CAN.h> 
#include "pdu.h"

#define CAR_TYPE "SJHT_WS1250BEVJ48_FT2.01"
/*
V2.00		更换硬件平台

*/
//extern char CAR_TYPE[50];


static const uint8_t CAN1_USED_ANA = 1;					/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;					/*是否使用CAN2*/
static const uint8_t CAN3_USED_ANA = 0;					/*是否使用CAN3*/

#define CAN1_BAUDRATE				250000				/*CAN1波特率*/
#define CAN2_BAUDRATE				250000				/*CAN1波特率*/
#define CAN3_BAUDRATE				250000				/*CAN1波特率*/

#define CAN1_ID_COUNT		50							/*CAN1 CANID总数*/
#define CAN2_ID_COUNT		50							/*CAN2 CANID总数*/
#define CAN3_ID_COUNT		50							/*CAN3 CANID总数*/
#define MOTORCOUNT		  1								/*驱动电机总数*/
#define BATT_PACK_COUNT		1							/*可充电储能子系统个数*/
#define BATTERY_COUNT		400					/*单体电池总数*/
#define TEMPER_PROBE_COUNT		80			/*温度探针总数*/

/* 设定接收目标 */
void InitRecvObj(void);
/* 根据协议解析CAN数据 */
void unpackCAN(uint8_t ctrl,CAN_msg *msg);

#endif




















