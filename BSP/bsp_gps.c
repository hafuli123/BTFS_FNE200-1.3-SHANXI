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

static volatile	uint8_t u8_ucSum , u8_sSum;					/* rtk校验码时候使用 */
static uint8_t * u8_p_spt;														/* 用于提取串口接收缓冲区里的字节的指针 */
uint8_t u8_strBuf[20];												/* 用于从串口缓冲区的NEMA语句提出所需要的每种信息语句 */
																								/* GPS_T结构体获取信息时候从u8_strBuf获取 */																								
static volatile uint8_t u8_dgt;											/* 标记NMEA语句一些信息可能会存在为空的情况 */
static volatile uint32_t u32_ang, u32_min, u32_dec;	/* 计算信息中位置数据、海拔高度等需要数学计算数据的时候使用 */
static volatile int32_t i32_ang,i32_min,i32_dec;			/* 计算信息中位置数据、海拔高度等需要数学计算数据的时候使用 */

//GPS模块初始化，设置串口解析回调函数
void bsp_InitGPS(void)
{
	g_tGPS.antSta=0; /* rtk沒有天线检测功能 */

//	comSetReciveCallBack(RS3_COM,rtk_analysis);
}

//分析串口收集的NMEA语句
void rtk_analysis(void)
{
	rtk_analy();
}

/*
	对串口获得的RTK模块的NMEA语句进行解析
*/
static void rtk_analy(void)
{
	/* 帧校验 */
	if(rtk_chk((uint8_t *)u8_rxBuf_gga , u16_bufLen_gga)==1){ 
		rtk_GGA((uint8_t *)u8_rxBuf_gga , u16_bufLen_gga);
		/* 串口调试用 */
//		*u8_rxBuf_gga='$';
//		comSendBuf(RS1_COM,(uint8_t*)u8_rxBuf_gga,u16_bufLen_gga);
	}	
	
	if(rtk_chk((uint8_t *)u8_rxBuf_rmc , u16_bufLen_rmc)==1){ 
		rtk_RMC((uint8_t *)u8_rxBuf_rmc , u16_bufLen_rmc);
		/* 串口调试用 */
//		*u8_rxBuf_rmc='$';
//		comSendBuf(RS1_COM,(uint8_t*)u8_rxBuf_rmc,u16_bufLen_rmc);			
	}
	
	/* 卫星数量总数 */
	g_tGPS.ViewNumber = u8_vuNum[0] + u8_vuNum[1] + u8_vuNum[2] + u8_vuNum[3] 
										+u8_vuNum[4]+ u8_vuNum[5]+ u8_vuNum[6]+ u8_vuNum[7]+ u8_vuNum[8] ;
}

//帧校验，检验串口接收到的NMEA语句中，校验位的数据是否合理，合理输出1，不合理输出0
static uint8_t rtk_chk(uint8_t *_rbuf, uint16_t _len)
{
	u8_sSum=0;
	u8_ucSum=0;
	if(_len<3){
		return 0;
	}
	/* 如果没有校验字节，也认为出错 */
	if(_rbuf[_len-5]!='*'){
		return 0;
	}
	
	for(int i = 1; i < _len - 5; i++){
		/* 不允许出现非ASCII字符 */
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
例：$GPGGA,092204.999,4250.5589,S,14718.5084,E,1,04,24.4,19.7,M,,,,0000*1F
字段0：$GPGGA，语句ID，表明该语句为Global Positioning System Fix Data（GGA）GPS定位信息
字段1：UTC 时间，hhmmss.sss，时分秒格式
字段2：纬度ddmm.mmmm，度分格式（前导位数不足则补0）
字段3：纬度N（北纬）或S（南纬）
字段4：经度dddmm.mmmm，度分格式（前导位数不足则补0）
字段5：经度E（东经）或W（西经）
字段6：GPS状态，0=未定位，1=非差分定位，2=差分定位，3=无效PPS，6=正在估算
字段7：正在使用的卫星数量（00 - 12）（前导位数不足则补0）
字段8：HDOP水平精度因子（0.5 - 99.9）
字段9：海拔高度（-9999.9 - 99999.9）
字段10：地球椭球面相对大地水准面的高度
字段11：差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空）
字段12：差分站ID号0000 - 1023（前导位数不足则补0，如果不是差分定位将为空）
字段13：校验值
*/
//GGA语句获取信息，把相关信息填入GPS_T结构体
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
例：$GPGSV,3,1,10,20,78,331,45,01,59,235,47,22,41,069,,13,32,252,45*70

$GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71
$GPGSV,2,2,07,09,23,313,42,04,19,159,41,15,12,041,42*41

字段0：$GPGSV，语句ID，表明该语句为GPS Satellites in View（GSV）可见卫星信息
字段1：本次GSV语句的总数目（1 - 3）
字段2：本条GSV语句是本次GSV语句的第几条（1 - 3）
字段3：当前可见卫星总数（00 - 12）（前导位数不足则补0）

字段4：PRN 码（伪随机噪声码）（01 - 32）（前导位数不足则补0）
字段5：卫星仰角（00 - 90）度（前导位数不足则补0）
字段6：卫星方位角（00 - 359）度（前导位数不足则补0）
字段7：信噪比（00－99）dbHz

字段8：PRN 码（伪随机噪声码）（01 - 32）（前导位数不足则补0）
字段9：卫星仰角（00 - 90）度（前导位数不足则补0）
字段10：卫星方位角（00 - 359）度（前导位数不足则补0）
字段11：信噪比（00－99）dbHz

字段12：PRN 码（伪随机噪声码）（01 - 32）（前导位数不足则补0）
字段13：卫星仰角（00 - 90）度（前导位数不足则补0）
字段14：卫星方位角（00 - 359）度（前导位数不足则补0）
字段15：信噪比（00－99）dbHz
字段16：校验值
*/

/* 该函数只提取卫星数量即可，其他不需要 */
//GSV语句获取信息，把相关信息填入GPS_T结构体
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
例：$GPRMC,024813.640,A,3158.4608,N,11848.3737,E,10.05,324.27,150706,,,A*50
字段0：$GPRMC，语句ID，表明该语句为Recommended Minimum Specific GPS/TRANSIT Data（RMC）推荐最小定位信息
字段1：UTC时间，hhmmss.sss格式
字段2：状态，A=定位，V=未定位
字段3：纬度ddmm.mmmm，度分格式（前导位数不足则补0）
字段4：纬度N（北纬）或S（南纬）
字段5：经度dddmm.mmmm，度分格式（前导位数不足则补0）
字段6：经度E（东经）或W（西经）
字段7：速度，节，Knots
字段8：方位角，度
字段9：UTC日期，DDMMYY格式
字段10：磁偏角，（000 - 180）度（前导位数不足则补0）
字段11：磁偏角方向，E=东W=西
字段16：校验值
*/
//RMC语句获取信息，把相关信息填入GPS_T结构体
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
*	函 数 名: Analyze0183
*	功能说明: 分析0183数据包
*	形    参:  _ucaBuf  收到的数据
*			 _usLen    数据长度
*	返 回 值: 无
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
//		g_tGPS.antSta = 1;//天线正常
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
//		g_tGPS.antSta = 2;//天线短路
//	}
//	else if(strstr((char*)_ucaBuf,"ANTENNA OPEN") != NULL)
//	{
//		g_tGPS.antSta = 3;//天线断开
//	}
//	else if(strstr((char*)_ucaBuf,"ANTENNA OK") != NULL)
//	{
//		g_tGPS.antSta = 1;//天线正常
//	}
//}

/*
GPS上电后，每隔一定的时间就会返回一定格式的数据，数据格式为：

$信息类型，x，x，x，x，x，x，x，x，x，x，x，x，x

每行开头的字符都是‘$’，接着是信息类型，后面是数据，以逗号分隔开。一行完整的数据如下：

    $GPRMC,080655.00,A,4546.40891,N,12639.65641,E,1.045,328.42,170809,,,A*60



信息类型为：

GPGSV：可见卫星信息

GPGLL：地理定位信息

GPRMC：推荐最小定位信息

GPVTG：地面速度信息

GPGGA：GPS定位信息

GPGSA：当前卫星信息



1、 GPS DOP and Active Satellites（GSA）当前卫星信息

$GPGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>,<7>

<1>模式 ：M = 手动， A = 自动。
<2>定位型式 1 = 未定位， 2 = 二维定位， 3 = 三维定位。
<3>PRN 数字：01 至 32 表天空使用中的卫星编号，最多可接收12颗卫星信息。
<4> PDOP位置精度因子（0.5~99.9）
<5> HDOP水平精度因子（0.5~99.9）
<6> VDOP垂直精度因子（0.5~99.9）
<7> Checksum.(检查位).

2、 GPS Satellites in View（GSV）可见卫星信息

$GPGSV, <1>,<2>,<3>,<4>,<5>,<6>,<7>,?<4>,<5>,<6>,<7>,<8>

<1> GSV语句的总数
<2> 本句GSV的编号
<3> 可见卫星的总数，00 至 12。
<4> 卫星编号， 01 至 32。
<5>卫星仰角， 00 至 90 度。
<6>卫星方位角， 000 至 359 度。实际值。
<7>讯号噪声比（C/No）， 00 至 99 dB；无表未接收到讯号。
<8>Checksum.(检查位).
第<4>,<5>,<6>,<7>项个别卫星会重复出现，每行最多有四颗卫星。其余卫星信息会于次一行出现，若未使用，这些字段会空白。


3、Global Positioning System Fix Data（GGA）GPS定位信息
$GPGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,<11>,<12>*hh

<1> UTC时间，hhmmss（时分秒）格式
<2> 纬度ddmm.mmmm（度分）格式（前面的0也将被传输）
<3> 纬度半球N（北半球）或S（南半球）
<4> 经度dddmm.mmmm（度分）格式（前面的0也将被传输）
<5> 经度半球E（东经）或W（西经）
<6> GPS状态：0=未定位，1=非差分定位，2=差分定位，6=正在估算
<7> 正在使用解算位置的卫星数量（00~12）（前面的0也将被传输）
<8> HDOP水平精度因子（0.5~99.9）
<9> 海拔高度（-9999.9~99999.9）
<10> 地球椭球面相对大地水准面的高度
<11> 差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空）
<12> 差分站ID号0000~1023（前面的0也将被传输，如果不是差分定位将为空）


4、Recommended Minimum Specific GPS/TRANSIT Data（RMC）推荐定位信息
$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh

<1> UTC时间，hhmmss（时分秒）格式
<2> 定位状态，A=有效定位，V=无效定位
<3> 纬度ddmm.mmmm（度分）格式（前面的0也将被传输）
<4> 纬度半球N（北半球）或S（南半球）
<5> 经度dddmm.mmmm（度分）格式（前面的0也将被传输）
<6> 经度半球E（东经）或W（西经）
<7> 地面速率（000.0~999.9节，前面的0也将被传输）
<8> 地面航向（000.0~359.9度，以真北为参考基准，前面的0也将被传输）
<9> UTC日期，ddmmyy（日月年）格式
<10> 磁偏角（000.0~180.0度，前面的0也将被传输）
<11> 磁偏角方向，E（东）或W（西）
<12> 模式指示（仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效）


5、 Track Made Good and Ground Speed（VTG）地面速度信息
$GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh

<1> 以真北为参考基准的地面航向（000~359度，前面的0也将被传输）
<2> 以磁北为参考基准的地面航向（000~359度，前面的0也将被传输）
<3> 地面速率（000.0~999.9节，前面的0也将被传输）
<4> 地面速率（0000.0~1851.8公里/小时，前面的0也将被传输）
<5> 模式指示（仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效）

*/


/* 实测武汉地区 GPS数据
$GPGGA,064518.046,,,,,0,00,,,M,0.0,M,,0000*5A
$GPGLL,,,,,064518.046,V,N*76
$GPGSA,A,1,,,,,,,,,,,,,,,*1E
$GPGSV,3,1,12,18,56,351,,22,51,026,,14,51,206,21,19,48,285,*78
$GPGSV,3,2,12,26,38,041,,24,37,323,,03,37,281,,09,31,097,*78
$GPGSV,3,3,12,21,17,122,,25,13,176,,31,13,054,,20,00,266,*7A
$GPRMC,064518.046,V,,,,,,,250213,,,N*46
$GPVTG,,T,,M,,N,,K,N*2C

//蔡甸区
$GPGGA,161037.000,3030.6548,N,11402.4568,E,1,04,5.2,51.1,M,-15.5,M,,0000*42
$GPGSA,A,3,05,12,02,25,,,,,,,,,6.0,5.2,2.9*3B
$GPGSV,3,1,10,02,49,314,31,05,37,225,41,12,33,291,32,25,09,318,33*7C
$GPGSV,3,2,10,10,85,027,18,04,57,019,18,17,45,123,20,13,26,075,*7F
$GPGSV,3,3,10,23,14,050,23,40,18,253,33*71
$GPRMC,161037.000,A,3030.6548,N,11402.4568,E,0.00,,010613,,,A*71
$GPVTG,,T,,M,0.00,N,0.0,K,A*13

//第2次
$GPGGA,165538.000,3030.6519,N,11402.4480,E,2,05,1.9,39.5,M,-15.5,M,6.8,0000*68
$GPGSA,A,3,26,05,25,12,02,,,,,,,,2.7,1.9,2.0*3A
$GPGSV,3,1,11,10,63,029,18,02,58,344,23,05,55,247,46,04,50,053,26*75
$GPGSV,3,2,11,12,31,265,39,17,27,139,22,13,22,053,23,25,17,301,37*78
$GPGSV,3,3,11,26,11,180,43,23,04,036,,40,18,253,33*4A
$GPRMC,165538.000,A,3030.6519,N,11402.4480,E,0.00,71.87,010613,,,D*5E
$GPVTG,71.87,T,,M,0.00,N,0.0,K,D*31
$GPGGA,165539.000,3030.6519,N,11402.4480,E,2,05,1.9,39.5,M,-15.5,M,7.8,0000*68
$GPRMC,

度分秒 换算: 30度 30分 65

3030.6519 = 30度 + 30.6519分， 60进制， 
30.6519 分  --> 30.6519 / 60 = 0.510865度。  30.510865度

*/
