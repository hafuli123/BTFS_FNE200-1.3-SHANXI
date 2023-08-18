#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "stdint.h"

typedef struct
{
	uint8_t PositionOk;
	char     NS;						/* 纬度半球N（北半球）或S（南半球） */
	char     EW;						/* 经度半球E（东经）或W（西经） 		*/
	double longd;						/* 经度 */
	double latd;						/* 纬度 */
	/*地面速度信息 */
	float TrackDegTrue;			/* 以真北为参考基准的地面航向，精度0.1度 */
	float SpeedKnots;				/* 地面速率精度0.01节*/
	float Altitude;					/* 海拔高度,精度0.01米*/
	float HDOP;							/* 水平精度,精度0.01米 */
	uint8_t ViewNumber;			/* 可见卫星个数 */
	uint8_t UseNumber;			/* 使用卫星个数 */
	/*UTC时间*/
	uint8_t Year;
	uint8_t  Month;
	uint8_t  Day;
	uint8_t  Hour;
	uint8_t  Min;
	uint8_t  Sec;
	//ANTENNA OPEN
	uint8_t antSta;					/* 1:正常 2:短路 3 :断开 , 但RTK模块没有这个功能 */
//	uint8_t seq;						/* 流水 用于判断定位模块是否在线 */
	
	uint8_t GpsQ;						/*定位质量*/
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

