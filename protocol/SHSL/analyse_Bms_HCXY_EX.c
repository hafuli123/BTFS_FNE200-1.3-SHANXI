/*
文 件：analyse_Bms_HCXY_EX.c
功 能：地上铁-华晨鑫源 - 企标控制及自定义数据采集功能
日 期: 2022/2/17
公 司：北理新源(佛山)信息科技有限公司
作 者: LGC
*/

#include "HCXY_DST/protocol_GB_EX_HCXY.h"

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;		/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;					/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;	/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/
	
//STANDARD_TYPE 标准帧；EXTENDED_TYPE 扩展帧
static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,STANDARD_TYPE,DATA_FRAME};
static void getCANLog_SendToHttps(uint8_t ch,CAN_msg *msg);			/* 采集CAN报文到https服务器 */
static uint8_t isOutLine = 0;												//是否网络异常
   
/*
功能：CAN数据接收、解析、发送

*/
void unpackDSTcan(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val = 0;
	uint32_t n32Val = 0;
	
	if(gUserDara.store_flag == 0xAA)		//用户数据有读取时再发送		
	{
		sendIntervalData(1);							//发送CAN数据功能接口						
	}
	
	if(pSelfData0A == NULL )
	{
		return;
	}
	switch(msg->id)
	{
		case 0x234:
			{
				//锁车指令许可位
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)				//禁止运营时，锁车状态置为 解析限速等级
				{
					//限速等级	 -- 锁车状态
					n8Val = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n8Val < 3)
						pSelfData0A->sLockCarST  = n8Val + 1;
					else
						pSelfData0A->sLockCarST  = 0;
				}
				else if(n8Val == 0)		//允许运营时，锁车状态置为 未限速
				{
					pSelfData0A->sLockCarST  = 0;
				}

				//当前程序版本 - 车辆控制器版本状态
				pSelfData0A->sVCUVerST = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//远程锁车状态
				n8Val = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
				//库存管理模式
				if(pSelfData0A->sLockCarST == 1)
				{
					n8Val = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n8Val == 1)
						pSelfData0A->sLockCarST = 6;				//重启一级限速（库存融）
				}
				//补电指令许可位
				sAllowAutoChg = calcCanValue(18,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//延时监控模式
				n8Val = calcCanValue(19,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//远程终端设备号
				n32Val = calcCanValue(24,32,msg->data,HIGHTLOW_MODE,CAN_MODE);

				//整车控制器life
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x177:
			{
				//充电状态
				uint8_t n8Val1 = calcCanValue(45,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				uint8_t n8Val2 = calcCanValue(47,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//快速或慢速充电
				if(0 == n8Val1 && 0 == n8Val2)
					pSelfData0A->sQorS_CHGSign = 0;					//无充电
				else if(1 == n8Val2 || 2 == n8Val2)
					pSelfData0A->sQorS_CHGSign = 1;					//快充
				else if(1 == n8Val1 || 2 == n8Val1)
					pSelfData0A->sQorS_CHGSign = 2;					//慢充
			}
			break;
		case 0x237:
			{
				//车型识别
				pSelfData0A->sCarType = calcCanValue(16,5,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
			
		case 0x175:
			{
				calculCHGPower();
			}
			break;
			
	}
	
//	getCANLog_SendToHttps(ch,msg);							//采集实车CAN报文到https服务器
}

#define OUTLINECSQ				8										//网络异常限速判定CSQ 小于8  
#define OUTLINEMILE				50									//网络异常限速判定里程 50km   
#define OUTLIMETIME				172800							//网络异常限速判定时间 48小时 = 172800

static uint8_t outLineLimitSpeedFun(uint8_t currCMD)
{
	uint8_t retCMD = currCMD;
	uint32_t outlineTimes = 0;
	
	//非一级限速 且非 库存融限速时执行 网络限速判定
	if(gUserDara.lockCMD != 0x55 && gUserDara.lockCMD != 0x60)
	{
		/* 网络异常限速 csq = 8 联网临界值 */
		if(Fun_Gprs_Csq() < OUTLINECSQ || Fun_Gprs_Csq() == 99 ||Fun_Gprs_GetSta() <= FUN_GPRS_GET_SIM)	
		{
			if(isOutLine == 1)																			//断网时间点参数更新
			{
				isOutLine = 0;
				gUserDara.outLineTime = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
				saveUserData();
			}		
			if((gRealData.totalMileage - gUserDara.outLineMile) > OUTLINEMILE)
			{
				if(g_system_dt.minute%2 == 0)
				{
					outlineTimes = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
					//if(outlineTimes - gUserDara.outLineTime >= 172800)		//大于48小时 172800秒
					if(outlineTimes - gUserDara.outLineTime >= OUTLIMETIME)		//大于48小时 172800秒
					{
						isOutLine = 2;
					}
				}			
				if(isOutLine == 2)																			//满足网络异常48小时触发一级限速
				{
					gUserDara.runCMD = 1;																	//禁止运营
					retCMD = 0;																						//限速一级
					pSelfData0A->sLockCarSign = 1;		
				}				
			}
		}
		else
		{
			isOutLine = 1;
			gUserDara.outLineMile = gRealData.totalMileage;						
		}
	
	}
	return retCMD;
}



/*******************************************************************************
功能: 地上铁华晨鑫源自定义功能 入口函数
描述：发送控制指令
*******************************************************************************/
void sendIntervalData(unsigned char canCh)
{
	uint8_t currCMD = 0,stockMode = 0;
	uint32_t tempCMD = 0;
	
	static uint32_t sendHeartBeatTime = 0;
	static uint8_t heartNum = 0;				//Life

	if(isEnableSendCANData() && osKernelGetTickCount() - sendHeartBeatTime >= 100)//发送周期信息  0x348  100ms执行一次	
	{
		sendHeartBeatTime = osKernelGetTickCount();	
		memset(msg_buf.data,0,sizeof(msg_buf.data));

		//未限速
		if(gUserDara.lockCMD == 0xAA)	
		{
			gUserDara.runCMD = 0;																			//允许运营			
			pSelfData0A->sLockCarSign = 0;														//未限速															
		}
		else if(gUserDara.lockCMD == 0x60)													//库存融限速
		{
			stockMode = 1;																						//库存管理模式 0:判定充电/1：不判充电
			currCMD = 0;																							//6：重启一级限速
			pSelfData0A->sLockCarSign = 6;																					
		}
		else if(gUserDara.lockCMD == 0x55)	
		{
			currCMD = 0;																							//1级
			pSelfData0A->sLockCarSign = 1;														
		}	
		else if(gUserDara.lockCMD == 0x56)
		{
			currCMD = 1;																							//2级
			pSelfData0A->sLockCarSign = 2;														
		}
		else if(gUserDara.lockCMD == 0x57)
		{
			currCMD = 2;																							//3级
			pSelfData0A->sLockCarSign = 3;														
		}
		if(gUserDara.changeCMD == 0x55)															//锁车版本才会触发网络限速
			currCMD = outLineLimitSpeedFun(currCMD);									//网络限速逻辑判定
		
		tempCMD |= (gUserDara.runCMD == 1 ? 1:0 ) << 0;							//锁车指令许可位：0：允许运营/1：禁止运营
		tempCMD |= (gUserDara.changeCMD == 0x55 ? 1:0) << 2;				//当前程序版本：0：非锁车/1：锁车
		tempCMD |= currCMD << 4;																		//限速等级：0:一级/1:二级/2:三级/3:无效		
		
		currCMD = 0;
		if(gUserDara.maintCMD == 0)
			currCMD = 0;
		else if(gUserDara.maintCMD>=0x55 && gUserDara.maintCMD <= 0x5C)
			currCMD = (gUserDara.maintCMD - 0x55) + 1;
		tempCMD |= currCMD << 6;																											//保养提示
														
		currCMD = 0;
		if(gUserDara.payCMD == 0)
			currCMD = 0;
		else if(gUserDara.payCMD>=0x55 && gUserDara.payCMD <= 0x5D)
			currCMD = (gUserDara.payCMD - 0x55) + 1;
		tempCMD |= currCMD << 10;																											//缴费提示
 
		currCMD = 0;
			
		if(gUserDara.yearCheckCMD == 0)
			currCMD = 0;
		else if(gUserDara.yearCheckCMD>=0x55 && gUserDara.yearCheckCMD <= 0x58)
			currCMD = (gUserDara.yearCheckCMD - 0x55) + 1;
		tempCMD |= currCMD << 14;																	//年审提示
		
		tempCMD |= (gUserDara.bindingCMD==0x55 ? 1:0) << 17;			//捆绑控制 0:正常/1:绑定
	
		currCMD = 0;
		if(autoChgState == 0 ||BSP_Iostatus(IN_ACC_ID) == ON)			//不补电，或者有ACC时 停止补电请求
			currCMD = 0;
		else
			currCMD = 1;
		
		tempCMD |= currCMD << 18;																	//补电请求 0:停止补电/1：请求补电	
		tempCMD |= (stockMode==1 ? 1:0) << 19;										//库存管理模式 0:判定充电/1：不判充电
		tempCMD |= (gRealData.totalMileage - gUserDara.outLineMile > OUTLINEMILE ? 1:0) << 21;		//网络异常限速提示
		msg_buf.data[0] = (uint8_t)(tempCMD>>0);
		msg_buf.data[1] = (uint8_t)(tempCMD>>8);
		msg_buf.data[2] = (uint8_t)(tempCMD>>16);
		
		//终端设备号
		msg_buf.data[3] = gFrimPara.terminalId[11];
		msg_buf.data[4] = gFrimPara.terminalId[10];
		msg_buf.data[5] =	gFrimPara.terminalId[9];
		msg_buf.data[6] = gFrimPara.terminalId[8];
		
		if(heartNum==255)
			heartNum=0;						
		msg_buf.data[7] = heartNum++;										
		msg_buf.id = 0x348;
		
		startSendCANData(canCh,&msg_buf);
	}
}

char databuff[50] = {0};
uint8_t httpLinkSt = 0;								//https连接状态
uint32_t httpCanLogStartTime = 0;			//https采集开始时间
uint32_t sendlen = 0;
uint32_t sendlenss = 0;
uint8_t switchss = 1;
static void getCANLog_SendToHttps(uint8_t ch,CAN_msg *msg)
{
	if(getCanLogSwitch == 1)
	{
		getCanLogSwitch = 2;
		char fileName[30];
		sprintf(fileName,"%s-%02d%02d%02d%02d%02d.txt",gFrimPara.terminalId,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
		
		//开启http 上送
		if(switchss == 0)
			httpLinkSt = Fun_Gprs_Http_Post_File_Start("http://basic.oicp.io:19004/test/","usr","123",fileName,70000+30);
		else if(switchss == 1)
			httpLinkSt = Fun_Gprs_Http_Post_File_Start("https://adas-file.flybees.com.cn:20001/upload/d0cc3e0817758ff5/1648635072/C9F41CA22B01E63360DA5A6D7295A8D050BE41117613B67F0FA2A74F4806FB1F",NULL,NULL,fileName,70000+30);
		else
			httpLinkSt = Fun_Gprs_Http_Post_File_Start(gUserDara.httpURL,gUserDara.httpUserName,gUserDara.httpPassWord,fileName,70000+30);		
		sendlen = 0;
	}
	if(getCanLogSwitch == 2)
	{
		if(httpLinkSt < 3 && httpLinkSt > 0)
		{
			if(httpLinkSt == 1)
			{
				sprintf(databuff,"GetCANLogTime:%04d%02d%02d%02d%02d%02d\r\n",g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);				
				httpCanLogStartTime = osKernelGetTickCount();
				httpLinkSt = 2;
			}
			else
				sprintf(databuff,"%05d %03x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",(osKernelGetTickCount() - httpCanLogStartTime), msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
			sendlenss = Fun_Gprs_Http_Post_File_Input(databuff,strlen(databuff));
			sendlen+= strlen(databuff);
			if(sendlenss == 0 || sendlen > 71000)
			{
				getCanLogSwitch = 0;
				httpLinkSt = 0;
				sendlen = 0; 
			}

		}
		else
			getCanLogSwitch = 0;
	}	
}




/*static void getCANLOGs(uint8_t ch,CAN_msg *msg)
{
	uint8_t ret = 0;

	if(getCanLogSign == 1)
	{
		getCanLogSign = 2;
		CanLogTime = osKernelGetTickCount();
		memset(saveBuff,0,sizeof(saveBuff));
		sprintf(path,"M0:\\%02x\\%s%02d%02d%02d%02d.%s",0xA2,gFrimPara.terminalId,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second,"LOG");
		if(bsp_storage_state() > 0)
		{
			files = fopen(path, "ab");
			if(files)
			{
				if(ftell(files) == 0)
				{
					//日志头
					sprintf(databuff,"GetCANLogTime:%04d%02d%02d%02d%02d%02d\r\n",g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
					byteswritten = fwrite(databuff,1, strlen(databuff), files);		
				}
			}
		}	
	}
	if(getCanLogSign == 2 && bsp_storage_state() > 0)
	{		
		CanLogTime1 = osKernelGetTickCount();
		if(CanLogTime1>0)
			CanLogTime2 = CanLogTime1 - CanLogTime;
		if(CanLogTime2 < GETCANLOGTIMELENTH)
		{
			if(files == NULL)
			{
				sta = funmount("M0:");
				sta = funinit("M0:");			
				osDelay(2);
				sta = finit("M0:");
				sta = fmount("M0:");
				if(sta == fsOK)
				{
					files = fopen(path, "ab");
				}		
			}				
			if(files != NULL)
			{
				//采集CAN日志
				sprintf(databuff,"%05d %03x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",(osKernelGetTickCount() - CanLogTime), msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
				memcpy(&saveBuff[saIndex],databuff,strlen(databuff));
				saIndex += strlen(databuff);
				if(saIndex + 150 > sizeof(saveBuff))
				{
					uint16_t bufLen = strlen(saveBuff);
					byteswritten = fwrite(saveBuff,1,bufLen,  files);	
					if(byteswritten != bufLen)
					{
						bufLen = fflush(files);	
						if(bufLen == 0)
						{
							saIndex = bufLen;		
						}
						fclose(files);
						files = NULL;
					}
					memset(saveBuff,0,sizeof(saveBuff));
					saIndex = 0;
				}
			}
		}
		else
		{
			if(files != NULL)
			{
				getCanLogSign = 0;
				byteswritten = fwrite(saveBuff,1, strlen(saveBuff), files);	
				fflush(files);
				fclose(files);
				files = NULL;
				memset(saveBuff,0,sizeof(saveBuff));
				saIndex = 0;				
			}
		}
	}
}

*/



