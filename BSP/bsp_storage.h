#ifndef _BSP_STORAGE_H
#define _BSP_STORAGE_H

#include "stdint.h"

#define SYS_PARA_ADDR               0x08004000	/* 终端系统参数存储位置 */
#define DEV_ID_ADDR                 0x08008000  /* 终端ID存储位置 */
#define BACKUP_DATA_ADDR         		0x08070000  /* 备份数据存储区域 128K */
#define FW_UPDATE_FLAG_ADDR         0x0800C000  /* 升级标志,文件长度存储区*/
#define FTP_FILE_SAVE_ADDR          0x08040000  /* 升级文件存储区 196K 		*/
#define DBC_FILE_FLAG_ADDR          0x080A0000  /* DBC文件存储区 128K 		*/
#define FTP_FILE_MAX_SIZE						0x30000			/* 升级文件最大大小 196K 	*/

#define USER_DATA_ADDR							0x08010000	/* 用户数据存储区域 64K */	

typedef enum
{
    REC = 0x00,			//用于补发
		LOG = 0x01,			//发送记录
		CAN = 0x02,			//CAN数据用于补发
		DAT = 0x03			//CAN数据用于存储
}FILE_TYPE;

void bsp_storage_init(void);
uint64_t bsp_storage_capacity(void);
uint8_t bsp_storage_state(void);
uint8_t bsp_first_start(void);

uint8_t saveConfig(uint8_t link,void* cfg,uint16_t len,uint8_t type);
uint8_t readConfig(uint8_t link,void * cfg,uint16_t cfgLen,uint8_t type);

uint8_t saveHistory(uint8_t link,uint8_t flag,uint8_t* buff,uint16_t len,uint32_t idx,FILE_TYPE flieType);
uint16_t readHistory(uint8_t link,char* fileName,uint8_t* buff,uint16_t maxLen,FILE_TYPE flieType);
uint8_t syncHistory(uint8_t link,char* fileName,uint8_t NameLen,FILE_TYPE fileType);

void recoverData(void);
void saveRecoverData(void);

uint8_t saveBinFile(uint32_t pos,uint16_t sigelSize,uint32_t fileSize,uint8_t* buff);
uint8_t saveDbcFile(uint32_t pos,uint16_t sigelSize,uint32_t fileSize,uint8_t* buff);

uint8_t Flash_Write(uint32_t wAddress,uint8_t *wBuf,uint16_t wLen);
uint8_t Flash_Read(uint32_t rAddress,uint8_t *rBuf,uint16_t rLen);
void sys_parameter_read(void);
uint8_t updateFile_Crc(uint32_t ftp_FileLength);

void Device_ID_Save(void);
void System_Pare_Save(void);
void System_Pare_Restore(void);
void User_ID_Save(void* data,uint16_t size);
void User_ID_Read(void* data,uint16_t size);
uint8_t bsp_formatU_state(void);									//格式化U盘

#endif
