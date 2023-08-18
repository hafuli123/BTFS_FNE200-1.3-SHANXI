#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "stdint.h"

typedef struct
{
	uint8_t PositionOk;
	char     NS;						/* γ�Ȱ���N�������򣩻�S���ϰ��� */
	char     EW;						/* ���Ȱ���E����������W�������� 		*/
	double longd;						/* ���� */
	double latd;						/* γ�� */
	/*�����ٶ���Ϣ */
	float TrackDegTrue;			/* ���汱Ϊ�ο���׼�ĵ��溽�򣬾���0.1�� */
	float SpeedKnots;				/* �������ʾ���0.01��*/
	float Altitude;					/* ���θ߶�,����0.01��*/
	float HDOP;							/* ˮƽ����,����0.01�� */
	uint8_t ViewNumber;			/* �ɼ����Ǹ��� */
	uint8_t UseNumber;			/* ʹ�����Ǹ��� */
	/*UTCʱ��*/
	uint8_t Year;
	uint8_t  Month;
	uint8_t  Day;
	uint8_t  Hour;
	uint8_t  Min;
	uint8_t  Sec;
	//ANTENNA OPEN
	uint8_t antSta;					/* 1:���� 2:��· 3 :�Ͽ� , ��RTKģ��û��������� */
//	uint8_t seq;						/* ��ˮ �����ж϶�λģ���Ƿ����� */
	
	uint8_t GpsQ;						/*��λ����*/
}GPS_T;

/* GPS quality indicator */
enum Gps_Quality{
	GPS_Q_INV = 0x30,		/* fix not availiable or invaild */
	GPS_Q_SPS,		/* GPS SPS Mode */
	GPS_Q_D,			/* Differential GPS, SPS Mode */
	GPS_Q_PPS,		/* GPS PPS Mode */
	GPS_Q_RTK,		/* Real Time Kinematic */
	GPS_Q_FRTK,		/* Float RTK. Satellite */
	GPS_Q_EST			/* Estimated */
};

extern void bsp_InitGPS(void);
extern GPS_T g_tGPS;

extern volatile uint8_t u8_rxFr;

//void rtk_analysis(uint8_t _times);
void rtk_analysis(void);

#endif

