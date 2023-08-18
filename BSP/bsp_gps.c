#include "bsp_gps.h"
#include "bsp_uart_fifo.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os2.h"
#include "stm32f4xx_usart.h"

GPS_T g_tGPS;

static uint8_t rtk_chk(uint8_t *_rbuf, uint16_t _len);
static void rtk_analy(void);
static void rtk_GGA(uint8_t * anabuf , uint16_t buflen);
static void rtk_GSV(uint8_t * _abuf , uint16_t _len);
static void rtk_RMC(uint8_t * _abuf , uint16_t _len);

extern volatile uint8_t u8_rxBuf[RX_MAX_LEN];
extern volatile uint8_t u8_rxBit;
extern volatile uint16_t u16_bufLen;

static volatile	uint8_t u8_ucSum , u8_sSum;					/* rtkУ����ʱ��ʹ�� */
static uint8_t * u8_p_spt;														/* ������ȡ���ڽ��ջ���������ֽڵ�ָ�� */
uint8_t u8_strBuf[20];												/* ���ڴӴ��ڻ�������NEMA����������Ҫ��ÿ����Ϣ��� */
																								/* GPS_T�ṹ���ȡ��Ϣʱ���u8_strBuf��ȡ */																								
static volatile uint8_t u8_dgt;											/* ���NMEA���һЩ��Ϣ���ܻ����Ϊ�յ���� */
static volatile uint32_t u32_ang, u32_min, u32_dec;	/* ������Ϣ��λ�����ݡ����θ߶ȵ���Ҫ��ѧ�������ݵ�ʱ��ʹ�� */
static volatile int32_t i32_ang,i32_min,i32_dec;			/* ������Ϣ��λ�����ݡ����θ߶ȵ���Ҫ��ѧ�������ݵ�ʱ��ʹ�� */

//GPSģ���ʼ�������ô��ڽ����ص�����
void bsp_InitGPS(void)
{
	g_tGPS.antSta=0; /* rtk�]�����߼�⹦�� */

//	comSetReciveCallBack(RS3_COM,rtk_analysis);
}

//���������ռ���NMEA���
void rtk_analysis(void)
{
	rtk_analy();
}

/*
	�Դ��ڻ�õ�RTKģ���NMEA�����н���
*/
static void rtk_analy(void)
{
	/* ֡У�� */
	if(rtk_chk((uint8_t *)u8_rxBuf_gga , u16_bufLen_gga)==1){ 
		rtk_GGA((uint8_t *)u8_rxBuf_gga , u16_bufLen_gga);
		/* ���ڵ����� */
//		*u8_rxBuf_gga='$';
//		comSendBuf(RS1_COM,(uint8_t*)u8_rxBuf_gga,u16_bufLen_gga);
	}	
	
	if(rtk_chk((uint8_t *)u8_rxBuf_rmc , u16_bufLen_rmc)==1){ 
		rtk_RMC((uint8_t *)u8_rxBuf_rmc , u16_bufLen_rmc);
		/* ���ڵ����� */
//		*u8_rxBuf_rmc='$';
//		comSendBuf(RS1_COM,(uint8_t*)u8_rxBuf_rmc,u16_bufLen_rmc);			
	}
	
	/* ������������ */
	g_tGPS.ViewNumber = u8_vuNum[0] + u8_vuNum[1] + u8_vuNum[2] + u8_vuNum[3] 
										+u8_vuNum[4]+ u8_vuNum[5]+ u8_vuNum[6]+ u8_vuNum[7]+ u8_vuNum[8] ;
}

//֡У�飬���鴮�ڽ��յ���NMEA����У�У��λ�������Ƿ�����������1�����������0
static uint8_t rtk_chk(uint8_t *_rbuf, uint16_t _len)
{
	u8_sSum=0;
	u8_ucSum=0;
	if(_len<3){
		return 0;
	}
	/* ���û��У���ֽڣ�Ҳ��Ϊ���� */
	if(_rbuf[_len-5]!='*'){
		return 0;
	}
	
	for(int i = 1; i < _len - 5; i++){
		/* ��������ַ�ASCII�ַ� */
		if((_rbuf[i] & 0x80) || (_rbuf[i] == 0))
		{
			return 0;
		}		
		u8_ucSum = u8_ucSum ^ _rbuf[i];		
	}
	sscanf((char*)&_rbuf[_len - 4],"%02hhx",&u8_sSum);
	if(u8_sSum == u8_ucSum){
		return 1;
	}
	return 0;
}




/*
����$GPGGA,092204.999,4250.5589,S,14718.5084,E,1,04,24.4,19.7,M,,,,0000*1F
�ֶ�0��$GPGGA�����ID�����������ΪGlobal Positioning System Fix Data��GGA��GPS��λ��Ϣ
�ֶ�1��UTC ʱ�䣬hhmmss.sss��ʱ�����ʽ
�ֶ�2��γ��ddmm.mmmm���ȷָ�ʽ��ǰ��λ��������0��
�ֶ�3��γ��N����γ����S����γ��
�ֶ�4������dddmm.mmmm���ȷָ�ʽ��ǰ��λ��������0��
�ֶ�5������E����������W��������
�ֶ�6��GPS״̬��0=δ��λ��1=�ǲ�ֶ�λ��2=��ֶ�λ��3=��ЧPPS��6=���ڹ���
�ֶ�7������ʹ�õ�����������00 - 12����ǰ��λ��������0��
�ֶ�8��HDOPˮƽ�������ӣ�0.5 - 99.9��
�ֶ�9�����θ߶ȣ�-9999.9 - 99999.9��
�ֶ�10��������������Դ��ˮ׼��ĸ߶�
�ֶ�11�����ʱ�䣨�����һ�ν��յ�����źſ�ʼ��������������ǲ�ֶ�λ��Ϊ�գ�
�ֶ�12�����վID��0000 - 1023��ǰ��λ��������0��������ǲ�ֶ�λ��Ϊ�գ�
�ֶ�13��У��ֵ
*/
//GGA����ȡ��Ϣ���������Ϣ����GPS_T�ṹ��
static void rtk_GGA(uint8_t * _abuf , uint16_t _len)
{
	u8_p_spt = (uint8_t*)&_abuf[7];
	uint8_t *p ;	
	//get time
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}
	sscanf((const char *)u8_strBuf,"%2hhu%2hhu%2hhu",&g_tGPS.Hour,&g_tGPS.Min,&g_tGPS.Sec);
	g_tGPS.Hour	= (g_tGPS.Hour+8)%24;
	u8_p_spt++; 
	
	//get latitude 
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%2u%2u.%6u",&u32_ang,&u32_min,&u32_dec);	
		g_tGPS.latd= (double)u32_ang+ ((double)u32_min + (double)u32_dec/1000000.000000f)/60.000000f;
	}
	u8_p_spt++;	
	
	//get North-south direction
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}
	if(u8_dgt==1){
		u8_dgt=0;
		g_tGPS.NS = *u8_strBuf;
	}
	u8_p_spt++;	
	
	//get longitude
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%3u%2u.%6u",&u32_ang,&u32_min,&u32_dec);	
		g_tGPS.longd= (double)u32_ang+ ((double)u32_min + (double)u32_dec/1000000.000000f)/60.000000f;
	}
	u8_p_spt++;
	
	//get East-west direction
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}
	if(u8_dgt==1){
		u8_dgt=0;
		g_tGPS.EW = *u8_strBuf;	 
	}
	u8_p_spt++;
	
	//get GPS quality indicator
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		g_tGPS.GpsQ = *u8_strBuf;
	}	
	if( (g_tGPS.GpsQ!='0')&&(g_tGPS.GpsQ!='6')){	
		g_tGPS.PositionOk = 1;
	}
	else{
		g_tGPS.PositionOk = 0;
	}
	u8_p_spt++;
	
	//get Number of satellites in use
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){	
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%2hhu",&g_tGPS.UseNumber);	
	}
	u8_p_spt++;
			
	//get HDOP
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%3u.%2u",&u32_ang,&u32_dec);
		g_tGPS.HDOP=(u32_ang*100+u32_dec)/100.0;
	}
	u8_p_spt++;
	
	//get altitude
	g_tGPS.Altitude = 0;
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%d.%3u",&i32_ang,&u32_dec);
		if(*u8_strBuf=='-'){
			if(u32_dec<10){
				g_tGPS.Altitude=(float)i32_ang-(float)u32_dec/10.0f;
			}
			else if((u32_dec>=10)&&(u32_dec<100)){
				g_tGPS.Altitude=(float)i32_ang-(float)u32_dec/100.0f;
			}
			else{
				g_tGPS.Altitude=(float)i32_ang-(float)u32_dec/1000.0f;
			}
		}
		else{
			if(u32_dec<10){
				g_tGPS.Altitude=(float)i32_ang+(float)u32_dec/10.0f;
			}
			else if((u32_dec>=10)&&(u32_dec<100)){
				g_tGPS.Altitude=(float)i32_ang+(float)u32_dec/100.0f;
			}
			else{
				g_tGPS.Altitude=(float)i32_ang+(float)u32_dec/1000.0f;
			}
		}
	}
	u8_p_spt++;
}

/*
����$GPGSV,3,1,10,20,78,331,45,01,59,235,47,22,41,069,,13,32,252,45*70

$GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71
$GPGSV,2,2,07,09,23,313,42,04,19,159,41,15,12,041,42*41

�ֶ�0��$GPGSV�����ID�����������ΪGPS Satellites in View��GSV���ɼ�������Ϣ
�ֶ�1������GSV��������Ŀ��1 - 3��
�ֶ�2������GSV����Ǳ���GSV���ĵڼ�����1 - 3��
�ֶ�3����ǰ�ɼ�����������00 - 12����ǰ��λ��������0��

�ֶ�4��PRN �루α��������룩��01 - 32����ǰ��λ��������0��
�ֶ�5���������ǣ�00 - 90���ȣ�ǰ��λ��������0��
�ֶ�6�����Ƿ�λ�ǣ�00 - 359���ȣ�ǰ��λ��������0��
�ֶ�7������ȣ�00��99��dbHz

�ֶ�8��PRN �루α��������룩��01 - 32����ǰ��λ��������0��
�ֶ�9���������ǣ�00 - 90���ȣ�ǰ��λ��������0��
�ֶ�10�����Ƿ�λ�ǣ�00 - 359���ȣ�ǰ��λ��������0��
�ֶ�11������ȣ�00��99��dbHz

�ֶ�12��PRN �루α��������룩��01 - 32����ǰ��λ��������0��
�ֶ�13���������ǣ�00 - 90���ȣ�ǰ��λ��������0��
�ֶ�14�����Ƿ�λ�ǣ�00 - 359���ȣ�ǰ��λ��������0��
�ֶ�15������ȣ�00��99��dbHz
�ֶ�16��У��ֵ
*/

/* �ú���ֻ��ȡ�����������ɣ���������Ҫ */
//GSV����ȡ��Ϣ���������Ϣ����GPS_T�ṹ��
static void rtk_GSV(uint8_t * _abuf , uint16_t _len)
{
	u8_p_spt = (uint8_t*)&_abuf[7];
	uint8_t *p, tid;
	//get total number of sentences - ignore
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}
	u8_p_spt++;
	//get Sentence number - ignore
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}
	u8_p_spt++;
	//get Total number of satellites in view
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}
	tid=_abuf[2];
	if( tid=='A'){
		if(_abuf[_len - 6] == '1'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[0]);
		}
		else if(_abuf[_len - 6] == '7'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[1]);
		}		
	}
	else if(tid=='B'){
		if(_abuf[_len - 6] == '1'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[2]);
		}
		else if(_abuf[_len - 6] == '5'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[3]);
		}			
	}
	else if(tid=='P'){
		if(_abuf[_len - 6] == '1'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[4]);
		}
		else if(_abuf[_len - 6] == '8'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[5]);
		}	
	}
	else if(tid=='L'){
		sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[6]);
	}
	else if(tid=='Q'){
		if(_abuf[_len - 6] == '1'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[7]);
		}
		else if(_abuf[_len - 6] == '8'){
			sscanf((const char *)u8_strBuf,"%2hhu",&u8_vuNum[8]);
		}
	}
	g_tGPS.ViewNumber = u8_vuNum[0] + u8_vuNum[1] + u8_vuNum[2] + u8_vuNum[3] 
										+u8_vuNum[4]+ u8_vuNum[5]+ u8_vuNum[6]+ u8_vuNum[7]+ u8_vuNum[8] ;
}

/*
����$GPRMC,024813.640,A,3158.4608,N,11848.3737,E,10.05,324.27,150706,,,A*50
�ֶ�0��$GPRMC�����ID�����������ΪRecommended Minimum Specific GPS/TRANSIT Data��RMC���Ƽ���С��λ��Ϣ
�ֶ�1��UTCʱ�䣬hhmmss.sss��ʽ
�ֶ�2��״̬��A=��λ��V=δ��λ
�ֶ�3��γ��ddmm.mmmm���ȷָ�ʽ��ǰ��λ��������0��
�ֶ�4��γ��N����γ����S����γ��
�ֶ�5������dddmm.mmmm���ȷָ�ʽ��ǰ��λ��������0��
�ֶ�6������E����������W��������
�ֶ�7���ٶȣ��ڣ�Knots
�ֶ�8����λ�ǣ���
�ֶ�9��UTC���ڣ�DDMMYY��ʽ
�ֶ�10����ƫ�ǣ���000 - 180���ȣ�ǰ��λ��������0��
�ֶ�11����ƫ�Ƿ���E=��W=��
�ֶ�16��У��ֵ
*/
//RMC����ȡ��Ϣ���������Ϣ����GPS_T�ṹ��
static void rtk_RMC(uint8_t * _abuf , uint16_t _len)
{
	u8_p_spt = (uint8_t*)&_abuf[7];
	uint8_t *p ;
	//get time
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}
	sscanf((const char *)u8_strBuf,"%2hhu%2hhu%2hhu",&g_tGPS.Hour,&g_tGPS.Min,&g_tGPS.Sec);
	g_tGPS.Hour	= (g_tGPS.Hour+8)%24;
	u8_p_spt++; 	
	
	//get position system status
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}
	if(*u8_strBuf == 'A'){
		g_tGPS.PositionOk = 1;
	}
	else{
		g_tGPS.PositionOk = 0;
	}
	u8_p_spt++; 	
	
	//get latitude 
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%2u%2u.%6u",&u32_ang,&u32_min,&u32_dec);	
		g_tGPS.latd= (double)u32_ang+ ((double)u32_min + (double)u32_dec/1000000.000000f)/60.000000f;
	}
	u8_p_spt++;	
	
	//get North-south direction
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}
	if(u8_dgt==1){
		u8_dgt=0;
		g_tGPS.NS = *u8_strBuf;
	}
	u8_p_spt++;	
	
	//get longitude
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%3u%2u.%6u",&u32_ang,&u32_min,&u32_dec);	
		g_tGPS.longd= (double)u32_ang+ ((double)u32_min + (double)u32_dec/1000000.000000f)/60.000000f;
	}
	u8_p_spt++;
	
	//get East-west direction
//	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}
	if(u8_dgt==1){
		u8_dgt=0;
		g_tGPS.EW = *u8_strBuf;	 
	}
	u8_p_spt++;

	//get knot speed
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%u.%2u",&u32_ang,&u32_dec);
		g_tGPS.SpeedKnots = (float)u32_ang + (float)u32_dec/100.00f;	 
	}
	u8_p_spt++;
	
	//get course degree
	memset(u8_strBuf,0,20);
	p=u8_strBuf;
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
		u8_dgt=1;
	}	
	if(u8_dgt==1){
		u8_dgt=0;
		sscanf((const char *)u8_strBuf,"%u.%2u",&u32_ang,&u32_dec);
		g_tGPS.TrackDegTrue = (float)u32_ang + (float)u32_dec/100.00f;	 
	}
	u8_p_spt++;	
	
	//get date
	memset(u8_strBuf,0,20);
	p=u8_strBuf;	
	while(*u8_p_spt!=','){
		*p = *u8_p_spt;
		p++;u8_p_spt++;
	}		
	sscanf((const char *)u8_strBuf,"%2hhu%2hhu%2hhu",&g_tGPS.Day,&g_tGPS.Month,&g_tGPS.Year);
	u8_p_spt++;

}

/*
*********************************************************************************************************
*	�� �� ��: Analyze0183
*	����˵��: ����0183���ݰ�
*	��    ��:  _ucaBuf  �յ�������
*			 _usLen    ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//static void Analyze0183(uint8_t *_ucaBuf, uint16_t _usLen)
//{
//	if (CheckXor(_ucaBuf, _usLen) != 1)
//	{
//		return;
//	}
//	if(g_tGPS.antSta == 0)
//	{
//		g_tGPS.antSta = 1;//��������
//	}
//	if (memcmp(&_ucaBuf[2], "GGA,",3) == 0)
//	{
//		gpsGPGGA(_ucaBuf, _usLen);
//	}
//	else if (memcmp(&_ucaBuf[2], "GSV,", 3) == 0)
//	{
//		gpsGPGSV(_ucaBuf, _usLen);
//	}
//	else if (memcmp(&_ucaBuf[2], "RMC,", 3) == 0)
//	{
//		g_tGPS.seq++;
//		gpsGPRMC(_ucaBuf, _usLen);
//	}
//	else if(strstr((char*)_ucaBuf,"ANTENNA SHORT") != NULL)
//	{
//		g_tGPS.antSta = 2;//���߶�·
//	}
//	else if(strstr((char*)_ucaBuf,"ANTENNA OPEN") != NULL)
//	{
//		g_tGPS.antSta = 3;//���߶Ͽ�
//	}
//	else if(strstr((char*)_ucaBuf,"ANTENNA OK") != NULL)
//	{
//		g_tGPS.antSta = 1;//��������
//	}
//}

/*
GPS�ϵ��ÿ��һ����ʱ��ͻ᷵��һ����ʽ�����ݣ����ݸ�ʽΪ��

$��Ϣ���ͣ�x��x��x��x��x��x��x��x��x��x��x��x��x

ÿ�п�ͷ���ַ����ǡ�$������������Ϣ���ͣ����������ݣ��Զ��ŷָ�����һ���������������£�

    $GPRMC,080655.00,A,4546.40891,N,12639.65641,E,1.045,328.42,170809,,,A*60



��Ϣ����Ϊ��

GPGSV���ɼ�������Ϣ

GPGLL������λ��Ϣ

GPRMC���Ƽ���С��λ��Ϣ

GPVTG�������ٶ���Ϣ

GPGGA��GPS��λ��Ϣ

GPGSA����ǰ������Ϣ



1�� GPS DOP and Active Satellites��GSA����ǰ������Ϣ

$GPGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>,<7>

<1>ģʽ ��M = �ֶ��� A = �Զ���
<2>��λ��ʽ 1 = δ��λ�� 2 = ��ά��λ�� 3 = ��ά��λ��
<3>PRN ���֣�01 �� 32 �����ʹ���е����Ǳ�ţ����ɽ���12��������Ϣ��
<4> PDOPλ�þ������ӣ�0.5~99.9��
<5> HDOPˮƽ�������ӣ�0.5~99.9��
<6> VDOP��ֱ�������ӣ�0.5~99.9��
<7> Checksum.(���λ).

2�� GPS Satellites in View��GSV���ɼ�������Ϣ

$GPGSV, <1>,<2>,<3>,<4>,<5>,<6>,<7>,?<4>,<5>,<6>,<7>,<8>

<1> GSV��������
<2> ����GSV�ı��
<3> �ɼ����ǵ�������00 �� 12��
<4> ���Ǳ�ţ� 01 �� 32��
<5>�������ǣ� 00 �� 90 �ȡ�
<6>���Ƿ�λ�ǣ� 000 �� 359 �ȡ�ʵ��ֵ��
<7>Ѷ�������ȣ�C/No���� 00 �� 99 dB���ޱ�δ���յ�Ѷ�š�
<8>Checksum.(���λ).
��<4>,<5>,<6>,<7>��������ǻ��ظ����֣�ÿ��������Ŀ����ǡ�����������Ϣ���ڴ�һ�г��֣���δʹ�ã���Щ�ֶλ�հס�


3��Global Positioning System Fix Data��GGA��GPS��λ��Ϣ
$GPGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,<11>,<12>*hh

<1> UTCʱ�䣬hhmmss��ʱ���룩��ʽ
<2> γ��ddmm.mmmm���ȷ֣���ʽ��ǰ���0Ҳ�������䣩
<3> γ�Ȱ���N�������򣩻�S���ϰ���
<4> ����dddmm.mmmm���ȷ֣���ʽ��ǰ���0Ҳ�������䣩
<5> ���Ȱ���E����������W��������
<6> GPS״̬��0=δ��λ��1=�ǲ�ֶ�λ��2=��ֶ�λ��6=���ڹ���
<7> ����ʹ�ý���λ�õ�����������00~12����ǰ���0Ҳ�������䣩
<8> HDOPˮƽ�������ӣ�0.5~99.9��
<9> ���θ߶ȣ�-9999.9~99999.9��
<10> ������������Դ��ˮ׼��ĸ߶�
<11> ���ʱ�䣨�����һ�ν��յ�����źſ�ʼ��������������ǲ�ֶ�λ��Ϊ�գ�
<12> ���վID��0000~1023��ǰ���0Ҳ�������䣬������ǲ�ֶ�λ��Ϊ�գ�


4��Recommended Minimum Specific GPS/TRANSIT Data��RMC���Ƽ���λ��Ϣ
$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh

<1> UTCʱ�䣬hhmmss��ʱ���룩��ʽ
<2> ��λ״̬��A=��Ч��λ��V=��Ч��λ
<3> γ��ddmm.mmmm���ȷ֣���ʽ��ǰ���0Ҳ�������䣩
<4> γ�Ȱ���N�������򣩻�S���ϰ���
<5> ����dddmm.mmmm���ȷ֣���ʽ��ǰ���0Ҳ�������䣩
<6> ���Ȱ���E����������W��������
<7> �������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
<8> ���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼��ǰ���0Ҳ�������䣩
<9> UTC���ڣ�ddmmyy�������꣩��ʽ
<10> ��ƫ�ǣ�000.0~180.0�ȣ�ǰ���0Ҳ�������䣩
<11> ��ƫ�Ƿ���E��������W������
<12> ģʽָʾ����NMEA0183 3.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч��


5�� Track Made Good and Ground Speed��VTG�������ٶ���Ϣ
$GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh

<1> ���汱Ϊ�ο���׼�ĵ��溽��000~359�ȣ�ǰ���0Ҳ�������䣩
<2> �Դű�Ϊ�ο���׼�ĵ��溽��000~359�ȣ�ǰ���0Ҳ�������䣩
<3> �������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
<4> �������ʣ�0000.0~1851.8����/Сʱ��ǰ���0Ҳ�������䣩
<5> ģʽָʾ����NMEA0183 3.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч��

*/


/* ʵ���人���� GPS����
$GPGGA,064518.046,,,,,0,00,,,M,0.0,M,,0000*5A
$GPGLL,,,,,064518.046,V,N*76
$GPGSA,A,1,,,,,,,,,,,,,,,*1E
$GPGSV,3,1,12,18,56,351,,22,51,026,,14,51,206,21,19,48,285,*78
$GPGSV,3,2,12,26,38,041,,24,37,323,,03,37,281,,09,31,097,*78
$GPGSV,3,3,12,21,17,122,,25,13,176,,31,13,054,,20,00,266,*7A
$GPRMC,064518.046,V,,,,,,,250213,,,N*46
$GPVTG,,T,,M,,N,,K,N*2C

//�̵���
$GPGGA,161037.000,3030.6548,N,11402.4568,E,1,04,5.2,51.1,M,-15.5,M,,0000*42
$GPGSA,A,3,05,12,02,25,,,,,,,,,6.0,5.2,2.9*3B
$GPGSV,3,1,10,02,49,314,31,05,37,225,41,12,33,291,32,25,09,318,33*7C
$GPGSV,3,2,10,10,85,027,18,04,57,019,18,17,45,123,20,13,26,075,*7F
$GPGSV,3,3,10,23,14,050,23,40,18,253,33*71
$GPRMC,161037.000,A,3030.6548,N,11402.4568,E,0.00,,010613,,,A*71
$GPVTG,,T,,M,0.00,N,0.0,K,A*13

//��2��
$GPGGA,165538.000,3030.6519,N,11402.4480,E,2,05,1.9,39.5,M,-15.5,M,6.8,0000*68
$GPGSA,A,3,26,05,25,12,02,,,,,,,,2.7,1.9,2.0*3A
$GPGSV,3,1,11,10,63,029,18,02,58,344,23,05,55,247,46,04,50,053,26*75
$GPGSV,3,2,11,12,31,265,39,17,27,139,22,13,22,053,23,25,17,301,37*78
$GPGSV,3,3,11,26,11,180,43,23,04,036,,40,18,253,33*4A
$GPRMC,165538.000,A,3030.6519,N,11402.4480,E,0.00,71.87,010613,,,D*5E
$GPVTG,71.87,T,,M,0.00,N,0.0,K,D*31
$GPGGA,165539.000,3030.6519,N,11402.4480,E,2,05,1.9,39.5,M,-15.5,M,7.8,0000*68
$GPRMC,

�ȷ��� ����: 30�� 30�� 65

3030.6519 = 30�� + 30.6519�֣� 60���ƣ� 
30.6519 ��  --> 30.6519 / 60 = 0.510865�ȡ�  30.510865��

*/
