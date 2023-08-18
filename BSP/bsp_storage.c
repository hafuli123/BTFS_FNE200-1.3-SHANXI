#include "bsp_storage.h"

#include "rl_fs.h"
#include "stm32f4xx_flash.h"

#include "bsp_sys.h"
#include "bsp_io.h"
#include "bsp_rtc.h"

#include "algo_verify.h"

#include "stdio.h"
#include "string.h"

#include "pdu.h"
#include "fun_can.h"

#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

typedef struct GET_FILE_POS{
	uint32_t flag; 		//存储标志
	uint32_t oldPos;  //已经读取位置
	uint32_t newPos;	//下次读取位置
}GetFilePos;

typedef struct {
	uint64_t flag:8; 		//存储标志
	uint64_t sta:8;			//状态标志
	uint64_t stamp:48;	//时间戳，48位可存放精确到毫秒，用于时间戳查找文件
	
	uint64_t len:16;		//报文长度
	uint64_t idx:32;		//包序号，用于序号查找文件
	uint64_t bcc:8;			//数据校验
}GetLogInfo;

//备份数据,上电时候恢复，注意，最大不超过64字节
typedef struct{
  uint16_t store_flag:16;					//存储标志
	uint16_t pos:16;								//记录位置
	uint32_t timeStamp;							//时间戳，用于恢复时间
	double longd;										//经度
	double latd;										//纬度
	float totalMileage;							//累计里程
}BKP_DATA;

typedef struct
{
	uint8_t state 			: 2; 				//0 不可用 1可用
	uint8_t chkSta			: 2;				//0 正常 2 初始状态
	uint8_t protectSta	: 2;				//0 正常 2 初始状态
	uint8_t rwSta				: 2;				//0 正常 2 初始状态
	uint8_t swSta				: 2;				//0 正常 2 初始状态
	uint64_t sdFreeCap  : 64;				//SD卡剩余容量
	uint64_t sdCapacity : 54;				//SD卡容量
}storage_sta_t;

struct ftpFile{
	uint32_t newFile_Flag;					//终端升级标志，开机后0xAA55表示升级失败，固件程序不匹配 0x55AA：升级成功  0x0002:FTP服务器故障 0x0004:固件程序不匹配
	uint32_t fileLength;						//升级文件长度
};

const char* fileSuffix[] = {"REC","LOG","CAN","DAT"};
BKP_DATA gBkpData;
storage_sta_t storage_sta = {0};

extern uint8_t szMainBuf[2048];
extern TERMINAL_PARA gFrimPara;
extern APP_PARA gSysPara;
extern RealData gRealData;

static struct ftpFile fileTagSave;
static uint8_t upDateFirstStart = 0;
static uint8_t isControlFormatU = 0;

static void readUpdateFile(void);

void bsp_storage_init(void)
{
	fsDriveInfo info;
	fsStatus sta;
	//文件系统初始状态
	storage_sta.state = 0;
	storage_sta.chkSta = 2;
	storage_sta.protectSta = 2;
	storage_sta.rwSta = 2;
	storage_sta.swSta = 2;
	sta = finit("M0:");
	if(sta == fsOK)
	{
		sta = fmount("M0:");
		if(sta == fsNoFileSystem)
		{
			//建立文件系统
			sta = fformat("M0:",0);
		}
		if(sta == fsOK)
		{
			finfo("M0:",&info);
			storage_sta.sdCapacity = info.capacity;
			storage_sta.sdFreeCap = ffree("M0:");
			readUpdateFile();
			//初始化EMMC成功
			storage_sta.state = 1;
			storage_sta.protectSta = 0;
			storage_sta.rwSta = 0;
			storage_sta.swSta = 0;
			storage_sta.chkSta = 0;
		}
		else
		{
			//文件系统加载失败
			storage_sta.chkSta = 3;
			storage_sta.protectSta = 2;
			storage_sta.rwSta = 2;
			storage_sta.swSta = 2;
		}
	}
	Flash_Read(FW_UPDATE_FLAG_ADDR,(uint8_t*)&fileTagSave,sizeof(fileTagSave));
	if(fileTagSave.newFile_Flag == 0x55AA)
	{
		upDateFirstStart = 1;
		fileTagSave.newFile_Flag = 0;
		Flash_Write(FW_UPDATE_FLAG_ADDR,(uint8_t*)&fileTagSave,0x800);
	}
}

uint64_t bsp_storage_capacity(void)
{
	return storage_sta.sdCapacity;
}

uint8_t bsp_storage_state(void)
{
	return storage_sta.state;
}

uint8_t bsp_first_start(void)
{
	return upDateFirstStart;
}

/*
功能：格式化U盘
*/
uint8_t bsp_formatU_state(void)
{
	fsStatus sta;
	fsDriveInfo info;

	isControlFormatU = 1;					//格式化执行开始			

	//文件系统初始状态
	storage_sta.state = 0;
	storage_sta.chkSta = 2;
	storage_sta.protectSta = 2;
	storage_sta.rwSta = 2;
	storage_sta.swSta = 2;

	//建立文件系统
	sta = fformat("M0:",0);

	if(sta == fsOK)
	{
		finfo("M0:",&info);
		storage_sta.sdCapacity = info.capacity;
		storage_sta.sdFreeCap = ffree("M0:");
		readUpdateFile();
		//初始化EMMC成功
		storage_sta.state = 1;
		storage_sta.protectSta = 0;
		storage_sta.rwSta = 0;
		storage_sta.swSta = 0;
		storage_sta.chkSta = 0;	
	}
	else
	{
		//文件系统加载失败
		storage_sta.chkSta = 3;
		storage_sta.protectSta = 2;
		storage_sta.rwSta = 2;
		storage_sta.swSta = 2;
	}
		isControlFormatU = 0;					//格式化执行结束
}


const char BOOTLOADER_VER[] __attribute__((at(0x8021000))) = "F4 V1.00";
/* 外部flash升级文件校验 */
uint8_t updateFile_Crc(uint32_t ftp_FileLength)
{
	uint8_t crcBit,rtn = 0, addCrc = 0;
	uint32_t i;
	char* pBootloaderVer = (char*)(FTP_FILE_SAVE_ADDR + 0x1000);
	if(ftp_FileLength > 1)
	{
		for(i = 0;i < ftp_FileLength - 1;i++)
		{
			Flash_Read(FTP_FILE_SAVE_ADDR+i,&crcBit,1);
			
			addCrc += crcBit;
		}
		/* 取出文件校验码 */
		Flash_Read(FTP_FILE_SAVE_ADDR + ftp_FileLength - 1,&crcBit,1);
		if(addCrc == crcBit && memcmp(BOOTLOADER_VER,pBootloaderVer,sizeof(BOOTLOADER_VER)) == 0)
		{
			rtn = 1;
		}
	}
	return rtn;
}

uint8_t saveBinFile(uint32_t pos,uint16_t sigelSize,uint32_t fileSize,uint8_t* buff)
{
	Flash_Write(FTP_FILE_SAVE_ADDR + pos,buff,sigelSize);
	if(pos + sigelSize >= fileSize)
	{
		if(updateFile_Crc(fileSize) == 1)								//累和校验
		{
			fileTagSave.newFile_Flag = 0xAA55;						//升级标志
			fileTagSave.fileLength = fileSize;										
			Flash_Write(FW_UPDATE_FLAG_ADDR,(uint8_t*)&fileTagSave,0x800);
		}
		else
		{
			return 0;
		}
	}
	return 1;
}

uint8_t saveDbcFile(uint32_t pos,uint16_t sigelSize,uint32_t fileSize,uint8_t* buff)
{
	Flash_Write(DBC_FILE_FLAG_ADDR + pos,buff,sigelSize);
	return 1;
}

fsFileInfo info;
FILE *f;

//读升级文件，包括DBC文件
void readUpdateFile(void)
{
	uint8_t type = 0;
	int i,bufPosition,ftp_FileLength;
	info.fileID = 0;
	if(ffind("M0:*.bin",&info) != fsOK)
	{
		if(ffind("M0:*.dbc",&info))
		{
			type = 1;
		}
	}
	if(strstr(info.name,".dbc")==NULL && strstr(info.name,".bin")==NULL)
	{
		return;
	}
	if(info.fileID != 0 && info.size > 0 && info.size < FTP_FILE_MAX_SIZE)
	{
		if((f = fopen (info.name,"r"))!= NULL)
		{
			memset(szMainBuf,0,sizeof(szMainBuf));
			bufPosition = 0;
			ftp_FileLength = 0;
			/* 数据域 */
			for(i = 0;i < info.size;i += 2048)
			{
				bufPosition = fread(szMainBuf,1,2048,f);
				if(bufPosition > 0)
				{
					/* 写入FLASH */
					if(type == 0)
						saveBinFile(ftp_FileLength,2048,info.size,szMainBuf);
					else
						saveBinFile(ftp_FileLength,2048,info.size,szMainBuf);
					ftp_FileLength += bufPosition;
					bufPosition = 0;
					memset(szMainBuf,0,sizeof(szMainBuf));
				}
			}
			fclose(f);
			fdelete(info.name,NULL);
			//升级完提示拔卡
			for(i= 0;i < 50;i++)
			{
				whistle(40,40);
			}
		}
		BoardReset();
	}
}

//存储日志，链路号，离线标志，数据,数据长度,文件类型
//格式:文件头+（记录头+记录内容）
uint8_t saveHistory(uint8_t link,uint8_t flag,uint8_t* buff,uint16_t len,uint32_t idx,FILE_TYPE fileType)
{
	fsStatus sta;
	FILE* fil;
	uint8_t ret = 0;
	char path[30] = {0};
	uint32_t byteswritten;                /* File write counts */	
	GetFilePos getPos = {0};
	GetLogInfo getLogInfo = {0};
	
	if(isControlFormatU == 1)							//执行格式化时，不允许操作U盘数据
		return 0;
	if(storage_sta.state > 0 && len > 0 && len < 8192)
	{
		//创建文件 按时间
		sprintf(path,"M0:\\%d\\%02d%02d%02d.%s",link,g_system_dt.year,g_system_dt.month,g_system_dt.day,fileSuffix[fileType]);
		fil = fopen(path, "ab");
		if(fil == NULL)
		{
			sta = funmount("M0:");
			sta = funinit("M0:");			
			osDelay(2);
			sta = finit("M0:");
			sta = fmount("M0:");
			if(sta == fsOK)
			{
				fil = fopen(path, "ab");
			}		
		}
		if(fil != NULL)
		{
			//判断是否新建文件
			if(ftell(fil) == 0)
			{
				getPos.flag = 0xAA;
				getPos.oldPos = sizeof(getPos);
				getPos.newPos = sizeof(getPos);
				fwrite(&getPos,1, sizeof(getPos),fil);
			}
			//写入报文头
			getLogInfo.flag = 0xAA;
			getLogInfo.len = len;
			getLogInfo.sta = flag;
			getLogInfo.stamp = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
			getLogInfo.stamp = getLogInfo.stamp * 1000 + osKernelGetTickCount() % 1000;
			getLogInfo.idx = idx;
			//计算BCC校验
			getLogInfo.bcc = getBccCode(buff,0,len);
			byteswritten = fwrite(&getLogInfo,1 ,sizeof(getLogInfo),fil);
			//写入报文
			byteswritten = fwrite(buff,1, len, fil);
			fflush(fil);
			if(len == byteswritten)
			{
				ret = 1;
			}
			fclose(fil);
		}
		osDelay(5);
	}	

	storage_sta.swSta = !ret;
	return ret;	
}

//flag:0 - 读补发数据 1 - 从头读 2 - 从当前位置读取 返回：读取长度，0：异常，文件需要重新同步
uint16_t readHistory(uint8_t link,char* fileName,uint8_t* buff,uint16_t maxLen,FILE_TYPE fileType)
{
	uint8_t sta = 0;
	uint64_t size;
	FILE* fil;
	uint16_t retLen = 0;
	uint32_t bytesSet;
	GetFilePos getPos = {0};
	GetLogInfo getLogInfo = {0};
	char path[30] = {0};
	
	if(storage_sta.state == 0 || isControlFormatU == 1)			//执行格式化时，不允许操作U盘数据
		return 0;
	sprintf(path,"M0:\\%d\\%s",link,fileName);
	fil = fopen(path,"rb+");
	if(fil != NULL)
	{
		//读取头，获取文件补发位置
		bytesSet = fread(&getPos,1,sizeof(getPos),fil);
		fseek(fil,0,SEEK_END );
		size = ftell(fil);
		if(sizeof(getPos) == bytesSet && getPos.newPos >= sizeof(getPos) && getPos.newPos < size)
		{
			sta = 1;
			//备份新读取地址
			getPos.oldPos = getPos.newPos;
			//偏移指定位置开始读数据
			fseek(fil,getPos.newPos,SEEK_SET);
			//直到读到补发数据
			while(1)
			{
				//数据头
				bytesSet = fread(&getLogInfo,1,sizeof(getLogInfo),fil);
				if(bytesSet == sizeof(getLogInfo) && getLogInfo.flag == 0xAA &&  getLogInfo.len > 0 && getLogInfo.len <= maxLen)
				{
					//下一条位置
					getPos.newPos += bytesSet + getLogInfo.len;
					bytesSet = fread(buff,1,getLogInfo.len,fil);
					if(bytesSet > 0 && getLogInfo.len == bytesSet)
					{
						if(getBccCode(buff,0,getLogInfo.len) == getLogInfo.bcc)
						{
							if(getLogInfo.sta == 0 || getLogInfo.sta == 0xFF)
							{
								//离线报文
								retLen = getLogInfo.len;
								fseek(fil,0,SEEK_SET);
								bytesSet = fwrite(&getPos,1,sizeof(getPos),fil);
								fflush(fil);
								break;
							}
						}
						else
						{
							;//BCC校验失败
						}
					}
					else
					{
						break;
					}
				}
				else
				{
					//文件读取失败或文件标志错误，退出
					break;
				}
			}
		}
		fclose(fil);
		if(retLen == 0)
		{
			//补发完成或出错，删除文件
			fdelete(path,NULL);
		}
	}
	osDelay(5);
	storage_sta.swSta = !sta;
	return retLen;	
}

//同步数据 0：未发现补发数据 返回值 1需要补发
uint8_t syncHistory(uint8_t link,char* fileName,uint8_t NameLen,FILE_TYPE fileType)
{
	FILE* fil;
	uint8_t cnt = 0;
	uint8_t ret = 0;
	fsFileInfo info = {0};
	char minName[30] = {0};
	char path[30] = {0};
	char  tempName[30];
	GetFilePos getPos;
	uint32_t bytesread;

	if(storage_sta.state == 0 || isControlFormatU == 1)//执行格式化时，不允许操作U盘数据
		return 0;
	fileName[0] = 0;
	//遍历目录
	sprintf(path,"M0:\\%d\\*.%s",link,fileSuffix[fileType]);
	while (ffind (path,&info) == 0)
	{
		if(strlen(info.name) < 10 || strlen(info.name) >= NameLen || strspn(info.name,"0123456789") < 6)
		{
			sprintf(tempName,"M0:\\%d\\%s",link,info.name);
			fdelete(tempName,NULL);
			continue;//年月日时分秒加后缀小于10,长度不对，前8个字符不是日期删除
		}
		else if(fileType == REC || fileType == CAN)
		{
			//读取文件，判断是否补发
			sprintf(tempName,"M0:\\%d\\%s",link,info.name);
			fil = fopen(tempName,"rb");
			if(fil != NULL)
			{
				bytesread = fread(&getPos,1,sizeof(getPos),fil);
				if(bytesread == sizeof(getPos))
				{
					if(getPos.newPos < info.size)//文件需要补发
					{
						if(fileName[0] == 0 || memcmp(info.name,fileName,6) < 0)
						{
							ret = 1;
							strcpy(fileName,info.name);
						}
					}
				}
				fclose(fil);
			}
		}
		cnt++;
		//存储早期文件，用于删除
		if(minName[0] == 0 || memcmp(info.name,minName,8) < 0)
		{
			strcpy(minName,info.name);
		}
	}
	//过期文件删除
	if(cnt > 7 && minName[0] != 0)
	{
		sprintf(tempName,"M0:\\%d\\%s",link,minName);
		fdelete(tempName,NULL);
		//判断补发文件是否删除
		if(strcmp(minName,fileName) == 0)
		{
			fileName[0] = 0;
		}
	}
	return ret;	
}

#define BACKUP_SRAM_SIZE		(4*1024)		//备份SRAM大小

void recoverData(void)
{
	uint32_t addr = 0;
	/* 电源接口时钟使能 (Power interface clock enable) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	/* DBP 位置 1，使能对备份域的访问 */
	PWR_BackupAccessCmd(ENABLE);
	/* 通过将 RCC AHB1 外设时钟使能寄存器 (RCC_AHB1ENR) 中的 BKPSRAMEN 位置 1， 使能备份 SRAM 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	/* 应用程序必须等待备份调压器就绪标志 (BRR) 置 1，指示在待机模式和 VBAT 模式下会保持写入 RAM 中的数据。 */
	PWR->CSR |= PWR_CSR_BRE;
	while(PWR_GetFlagStatus(PWR_FLAG_BRR) != SET);
	//读取SRAM数据
	memcpy(&gBkpData,(uint8_t *)BKPSRAM_BASE, sizeof(gBkpData));
	//读取FLASH数据
	if(gBkpData.store_flag != 0xA5A5)
	{
		for(addr = 0;addr < 128 * 1024 ;addr += 64)
		{
			BKP_DATA * pBkpData = (BKP_DATA*)(BACKUP_DATA_ADDR + addr);
			if(pBkpData->store_flag == 0xA5A5)
			{
				//flash数据有效,读取最大值
				memcpy(&gBkpData,pBkpData, sizeof(gBkpData));
			}
		}
		if(gBkpData.store_flag == 0xA5A5)
		{
			RTC_INFOR dt;
			//时间失效，校时
			RTC_localtime(gBkpData.timeStamp,&dt);
			RTC_Time_Set(&dt);
		}
	}
	if(gBkpData.store_flag == 0xA5A5)
	{
		gRealData.latd = gBkpData.latd;
		gRealData.longd = gBkpData.longd;
		gRealData.totalMileage = gBkpData.totalMileage;
	}
}

void saveRecoverData(void)
{
	static uint32_t saveSysTimeStamp = 0,latd = 0,totalMileage = 0;
	uint8_t saveFlag = 0;
	//FLASH定时存储
	if(osKernelGetTickCount() - saveSysTimeStamp >= 30000 && fun_can_Get_State(0xFF) == 0 && (latd != gRealData.latd || totalMileage != gRealData.totalMileage))
	{
		BKP_DATA* pData = (BKP_DATA*)(gBkpData.pos % 2048 * 64 + BACKUP_DATA_ADDR);
		if(gBkpData.pos >= 2048 || pData->store_flag != 0xA5A5)
		{
			//已存储数据出错，或未存储
			gBkpData.pos = 0;			
		}
		else
		{
			gBkpData.pos++;
		}
		gBkpData.pos %= 2048;
		saveFlag = 1;
		saveSysTimeStamp = osKernelGetTickCount();
		latd = gRealData.latd;
		totalMileage = gRealData.totalMileage;
	}
	//存储标志
	gBkpData.store_flag = 0xA5A5;
	//数据更新
	gBkpData.timeStamp = RTC_mktime(g_system_dt.year,g_system_dt.month,g_system_dt.day,g_system_dt.hour,g_system_dt.minute,g_system_dt.second);
	gBkpData.latd = gRealData.latd;
	gBkpData.longd = gRealData.longd;
	gBkpData.totalMileage = gRealData.totalMileage;
	if(saveFlag)
	{
		Flash_Write(gBkpData.pos % 2048 * 64 + BACKUP_DATA_ADDR,(uint8_t*)&gBkpData,64);
	}
	memcpy((uint8_t *)BKPSRAM_BASE, &gBkpData, sizeof(gBkpData));
}

/*备份寄存器分区
0x000:终端
0x080:链路0
0x100:链路1
0x180:链路2
0x200:链路3
0x280:链路4
*/

//存储配置，流水号，补发配置等信息，用于重启
uint8_t saveConfig(uint8_t link,void* cfg,uint16_t len,uint8_t type)
{
	uint8_t ret = 1;
	uint32_t addr =  (link + 1) * 0x80 + (type * 0x40);
	memcpy((uint8_t *)BKPSRAM_BASE + addr,cfg ,len);
	return ret;
}

//读取配置，流水号，补发文件，补发位置等信息，用于重启恢复
uint8_t readConfig(uint8_t link,void * cfg,uint16_t cfgLen,uint8_t type)
{
	uint8_t ret = 1;
	uint32_t addr =  (link + 1) * 0x80 + (type * 0x40);
	memcpy(cfg ,(uint8_t *)BKPSRAM_BASE + addr,cfgLen);
	return ret;
}

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte  bootloader	*/
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte  应用参数 		*/
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte  终端参数 		*/
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte  升级校验 		*/
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte  用户数据	 	*/


#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte 自身程序			*/
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbyte 升级文件			*/
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbyte */

#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */
/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }
  return sector;
}

#define StartAddr                   ((uint32_t)0x08004000)
#define EndAddr                     ((uint32_t)0x080F0000)
#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* 读FLASH
   输入：rBuf:读缓冲区 rLen:读数据长度
   输出：_TRUE-->读取成功 _FLASE->读取失败
*/
uint8_t Flash_Read(uint32_t rAddress,uint8_t *rBuf,uint16_t rLen)
{
	uint32_t Address = rAddress;
	while(rLen && Address < EndAddr)
	{
		*rBuf = *(uint8_t*)Address;
		rBuf++;
		rLen -= 1;
		Address += 1;
	}
	return TRUE;
}

/* 写FLASH 带非法地址保护，扇区首地址自动擦除 可连续写
   输入：wBuf:写缓冲区 wLen:写数据长度
   输出：_TRUE-->读取成功 _FLASE->读取失败
*/
uint8_t Flash_Write(uint32_t wAddress,uint8_t *wBuf,uint16_t wLen)
{
	uint8_t ret;
	uint32_t Address;
	FLASH_Status FLASHStatus;
	uint32_t Data,RamSource;
	
	FLASHStatus = FLASH_COMPLETE;
	Address = wAddress;
	//非法地址APP区域和BOOTLOADER区域
	if((wAddress >= ADDR_FLASH_SECTOR_5 && wAddress < ADDR_FLASH_SECTOR_6) || wAddress < StartAddr || wAddress >= EndAddr)
	{
		return FALSE;
	}
	DISABLE_INT();
	/**  Unlocks Flash for write access **/
  FLASH_Unlock(); 
  /* Clear pending flags (if any) */  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	
	if(ADDR_FLASH_SECTOR_1 == wAddress || ADDR_FLASH_SECTOR_2 == wAddress || ADDR_FLASH_SECTOR_3 == wAddress || ADDR_FLASH_SECTOR_4 == wAddress || ADDR_FLASH_SECTOR_6 == wAddress||
 		 ADDR_FLASH_SECTOR_7 == wAddress || ADDR_FLASH_SECTOR_8 == wAddress || ADDR_FLASH_SECTOR_9 == wAddress || ADDR_FLASH_SECTOR_10 == wAddress|| ADDR_FLASH_SECTOR_11 == wAddress)
	{
		//扇区首地址，需要擦除才能写，其他地址擦除后没写过可以继续写
		uint32_t UserSector = GetSector(wAddress);
		FLASHStatus = FLASH_EraseSector(UserSector,VoltageRange_3);
	}
	
	if(FLASHStatus == FLASH_COMPLETE)
	{
		RamSource = (uint32_t)wBuf;
		while(wLen && Address < EndAddr)
		{
			Data = *(u32*)RamSource;
			if(FLASH_ProgramWord(Address, Data) != FLASH_COMPLETE)
			{
				ret =  FALSE;
				break;
			}
			if(wLen < 4)
			{
				ret =  TRUE;
				break;
			}
			wLen -= 4;
	    Address = Address + 4;
	    RamSource += 4;
		}
		ret =  TRUE;
	}
	else
		ret =  FALSE;	
	ENABLE_INT();
	return ret;
}

/* 系统参数保存
*/
void System_Pare_Save(void)
{
	gSysPara.store_flag = 0xAA;
	Flash_Write(SYS_PARA_ADDR,(uint8_t*)&gSysPara,0x800);
}

/* 终端参数保存 */
void Device_ID_Save(void)
{
	gFrimPara.store_flag = 0xAA;
	Flash_Write(DEV_ID_ADDR,(uint8_t*)&gFrimPara,0x800);
}

/* 用户参数保存 */
void User_ID_Save(void* data,uint16_t size)
{
//	Flash_Write(USER_DATA_ADDR,(uint8_t*)&data,((size / 4) + 1) * 4);
	Flash_Write(USER_DATA_ADDR,(uint8_t*)data,0x800);
}

/* 用户参数读取 */
void User_ID_Read(void* data,uint16_t size)
{
	memcpy(data,(void*)USER_DATA_ADDR,size);
}


/*系统参数恢复
*/
void System_Pare_Restore(void)
{
	strcpy((char*)&gSysPara.apn,"CMNET");
	//SERVER
	strcpy((char*)&gSysPara.domain[0],"tuoguan.bitnei.cn");
	gSysPara.port[0] = 19006;
	
	strcpy((char*)&gSysPara.domain[1],"demo.bitnei.cn");
	gSysPara.port[1] = 19006;
	
	//FactoryTime
	gSysPara.localSaveInterval = 10000;      //本地存储时间周期(ms)
	gSysPara.realDataInterval = 10;          //实时信息上报周期(s)
	gSysPara.warnRealDataInterval = 1000;    //报警时信息上报周期(ms)
	gSysPara.heartInterval = 30;             //心跳周期(s)
	
	/*车辆信息 */
	strcpy((char*)&gSysPara.vinCode,gFrimPara.terminalId);
	/*保存系统参数*/
	System_Pare_Save();
}

/* 系统参数读取 */
void sys_parameter_read(void)
{
	//系统参数
	Flash_Read(SYS_PARA_ADDR,(uint8_t*)&gSysPara,sizeof(APP_PARA));
	if(gSysPara.store_flag != 0xAA)
	{
		memset(&gSysPara,0,sizeof(APP_PARA));
	}
	//终端参数
	Flash_Read(DEV_ID_ADDR,(uint8_t*)&gFrimPara,sizeof(TERMINAL_PARA));
	if(gFrimPara.store_flag != 0xAA)
	{
		memset(&gFrimPara,0,sizeof(TERMINAL_PARA));
	}
	if(strlen(gSysPara.hardWareVer) >= sizeof(gSysPara.hardWareVer) || strlen(gSysPara.hardWareVer) == 0)
	{
		strcpy(gSysPara.hardWareVer,"v1.0");
	}
}
