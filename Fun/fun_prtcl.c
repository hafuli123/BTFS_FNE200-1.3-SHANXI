#include "cmsis_os2.h"
#include "fun_prtcl.h"
#include "protocol_GB32960.h"
#include "protocol_GB17691.h"
#include "terminal_Server.h"
#include "bsp_rtc.h"
#include "string.h"
#include "algo_verify.h"
#include "rl_fs.h"
#include "stdio.h"
#include "fun_can.h"
#include "bsp_power.h"

//数据缓存指针
static RealData realCache;								//缓存读取暂存
static FILE *f_cache = NULL;							//缓存文件句柄
static uint8_t cacheWrite = 0;						//缓存写入位置
static uint8_t cacheWriteCnt = 0;					//实际缓存条数
static const uint8_t CACHE_MAX_CNT = 31; 	//最大缓存条数
static void saveCache(void);

static uint8_t second;
static uint8_t sendBuff[1460];
static void* ev1Cfg = NULL;
static void* ev2Cfg = NULL;
static void* ev3Cfg = NULL;
static void* fv1Cfg = NULL;
static void* fv2Cfg = NULL;

static int8_t wakelock = -1;
static uint8_t active1 = 0;
static uint8_t active2 = 0;
static uint8_t active3 = 0;
static uint8_t active4 = 0;
static uint8_t active5 = 0;

//协议初始化
void funPrtclInit(void)
{
	fdelete("M0:cache.tmp",NULL);
	InitTermServer();
	ev1Cfg =  gb32960Init(EV1_SERVER,sendBuff,sizeof(sendBuff),gSysPara.domain[EV1_SERVER],&gSysPara.port[EV1_SERVER],gSysPara.vinCode,gSysPara.realDataInterval,gSysPara.heartInterval);
	ev2Cfg =  gb32960Init(EV2_SERVER,sendBuff,sizeof(sendBuff),gSysPara.domain[EV2_SERVER],&gSysPara.port[EV2_SERVER],gSysPara.vinCode,gSysPara.realDataInterval,gSysPara.heartInterval);
	ev3Cfg =  gb32960Init(EV3_SERVER,sendBuff,sizeof(sendBuff),gSysPara.domain[EV3_SERVER],&gSysPara.port[EV3_SERVER],gSysPara.vinCode,gSysPara.realDataInterval,gSysPara.heartInterval);
	fv1Cfg =  gb17691Init(FVHJ_SERVER,sendBuff,sizeof(sendBuff),gSysPara.domain[FVHJ_SERVER],&gSysPara.port[FVHJ_SERVER],gSysPara.vinCode,((gSysPara.linkSwitch & SIGNATURE_BIT) > 0) + 1);
	fv2Cfg =  gb17691Init(FV17691_SERVER,sendBuff,sizeof(sendBuff),gSysPara.domain[FV17691_SERVER],&gSysPara.port[FV17691_SERVER],gSysPara.vinCode,0);
	wakelock = bsp_wakelock_create();
}

//协议运行
void funPrtclrun(void)
{
	termServerTask();
	RTC_Time_Get(&g_system_dt);
	if(second != g_system_dt.second && (active1 || active2 || active3 || active4 || active5))
	{
		second = g_system_dt.second;
		saveCache();
	}
	if(gSysPara.linkSwitch & (1 << EV1_SERVER))
		active1 = gb32960Run(ev1Cfg);
	else if(active1 == 1)
	{
		active1 = 0;
		cleanPara(ev1Cfg);
	}
	
	if(gSysPara.linkSwitch & (1 << EV2_SERVER))
		active2 = gb32960Run(ev2Cfg);
	else if(active2 == 1)
	{
		active2 = 0;
		cleanPara(ev2Cfg);
	}
	
	if(gSysPara.linkSwitch & (1 << EV3_SERVER))
		active3 = gb32960Run(ev3Cfg);
	else if(active3 == 1)
	{
		active3 = 0;
		cleanPara(ev3Cfg);
	}

	if(gSysPara.linkSwitch & (1 << FVHJ_SERVER))
		active4 = gb17691Run(fv1Cfg);
	else if(active4 == 1)
	{
		active4 = 0;
		cleanPara(fv1Cfg);
	}
	
	if(gSysPara.linkSwitch & (1 << FV17691_SERVER))
		active5 = gb17691Run(fv2Cfg);
	else if(active5 == 1)
	{
		active5 = 0;
		cleanPara(fv2Cfg);
	}	
	osDelay(1);
	if(active1 || active2 || active3 || active4 || active5)
	{
		bsp_wakelock_lock(wakelock);
	}
	else
	{
		bsp_wakelock_unlock(wakelock);
	}
}

/*保存缓存*/
void saveCache(void)
{
	gRealData.year = g_system_dt.year;
	gRealData.month = g_system_dt.month;
	gRealData.day = g_system_dt.day;
	gRealData.hour = g_system_dt.hour;
	gRealData.minute = g_system_dt.minute;
	gRealData.second = g_system_dt.second;	
	if(f_cache == NULL)
	{
		f_cache = fopen("M0:cache.tmp","wb+");
	}
	if(f_cache != NULL)
	{
		if(fseek(f_cache,cacheWrite * sizeof(gRealData),SEEK_SET) == 0)
		{
			if(fwrite(&gRealData,sizeof(gRealData),1,f_cache) > 0)
			{
				cacheWrite = (cacheWrite + 1) % CACHE_MAX_CNT;
				if(cacheWriteCnt < CACHE_MAX_CNT)
				{
					cacheWriteCnt++;
				}
			}
			else
			{
				f_cache = NULL;
			}
		}
		else
		{
			f_cache = NULL;
		}
	}
}

/*返回有效读取位置0xFF,无效 参数：期望读取的位置*/
RealData* getRealCache(uint8_t dataIdx,uint8_t* outDataIdx)
{
	if(dataIdx == 0)
	{
		*outDataIdx = 0;
		return &gRealData;
	}
	if(f_cache != NULL && cacheWriteCnt > 0)
	{
		//计算缓存位置
		uint8_t readTemp = cacheWrite;
		*outDataIdx = 0;
		while(1)
		{
			//读取偏移位置
			readTemp = (readTemp == 0) ? (cacheWriteCnt - 1): (readTemp - 1);
			//目标位置
			if(*outDataIdx >= dataIdx)
			{
				break;
			}
			//条数不足
			if(*outDataIdx >= (cacheWriteCnt - 1))
			{
				break;
			}
			//下一条
			(*outDataIdx)++;
		}
		//缓存读出
		if(fseek(f_cache,readTemp * sizeof(realCache),SEEK_SET) == 0)
		{
			if(fread(&realCache,sizeof(realCache),1,f_cache))
			{
				return &realCache;
			}
		}
	}
	*outDataIdx = 0xFF;
	return NULL;
}
