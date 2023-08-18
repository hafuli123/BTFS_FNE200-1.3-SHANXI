/*
�ļ���analyse_Bms_FC_FSQ3310DEFCEV.h
���ܣ��ɳ�FC_FSQ3310DEFCEV����Э��
���ڣ�2021/5/18
��˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
���ߣ�LGC
*/
#ifndef __ANALYSE_BMS_H
#define __ANALYSE_BMS_H

#include <RTX_CAN.h> 
#include "pdu.h"

#define CAR_TYPE "SJHT_WS1250BEVJ48_FT2.01"
/*
V2.00		����Ӳ��ƽ̨

*/
//extern char CAR_TYPE[50];


static const uint8_t CAN1_USED_ANA = 1;					/*�Ƿ�ʹ��CAN1*/
static const uint8_t CAN2_USED_ANA = 1;					/*�Ƿ�ʹ��CAN2*/
static const uint8_t CAN3_USED_ANA = 0;					/*�Ƿ�ʹ��CAN3*/

#define CAN1_BAUDRATE				250000				/*CAN1������*/
#define CAN2_BAUDRATE				250000				/*CAN1������*/
#define CAN3_BAUDRATE				250000				/*CAN1������*/

#define CAN1_ID_COUNT		50							/*CAN1 CANID����*/
#define CAN2_ID_COUNT		50							/*CAN2 CANID����*/
#define CAN3_ID_COUNT		50							/*CAN3 CANID����*/
#define MOTORCOUNT		  1								/*�����������*/
#define BATT_PACK_COUNT		1							/*�ɳ�索����ϵͳ����*/
#define BATTERY_COUNT		400					/*����������*/
#define TEMPER_PROBE_COUNT		80			/*�¶�̽������*/

/* �趨����Ŀ�� */
void InitRecvObj(void);
/* ����Э�����CAN���� */
void unpackCAN(uint8_t ctrl,CAN_msg *msg);

#endif




















