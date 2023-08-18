/*
 * newant_M2.c
 *
 *  Created on: 2019年1月29日
 *      Author: xtgaxm
 */
#include <RTX_CAN.h>
#include "pdu.h"
#include "newant_SM2.h"
#include "common.h"
#include "se.h"

/*
升级版
0x0001 SE密钥对
0x0002 外部1密钥对
0x0003 外部2密钥对

旧
0x0001 透明文件
0x0002 公钥（企业平台）
0x0003 SE 公钥
0x0004 SE 私钥
0x0005 公钥（国家平台）
*/

#define TMS_SE_KEY 						0x0001
#define TMS_REAL_KEY 					0x0002
#define TMS_SETUP_KEY 				0x0003

#define TMSC_BIN_KEY 					0x0001
#define TMSC_REAL_PUBKEY 			0x0002
#define TMSC_SE_PUBKEY 				0x0003
#define TMSC_SE_PRIKEY 				0x0004
#define TMSC_SETUP_PUBKEY 		0x0002

#define TMSC 0
#define TMS 1

static uint8_t sm2Data[SM2_CIPHER_MAX];
static uint8_t sm2Type = TMS;
static OS_MUT sm2Mutex;
/* *********************************************************
 *  函数名		:	SM2_IOInit
 *  输出结果	:	无
 *  函数描述	:	用于初始化SM2加密芯片
 *  返回值		:	无
 *  输入参数	:	com1
 *  时间			:	2019年1月29日	下午3:00:54
 * ********************************************************* */
void SM2_IOInit(void)
{
	os_mut_init (&sm2Mutex);
	if(tms_initialize(NULL)== SUCCEED)
	{
		gTerminalState.scyState = 1;
	}
	sm2Type = gFrimPara.seVer;
}

/* *********************************************************
 *  函数名		:	SM2_GedID
 *  输出结果	: 	无
 *  函数描述	:	获取加密模块的ID等硬件信息
 *  返回值		:	无
 *  输入参数	:	ID		加密模块的ID信息结构体指针
 *  时间			:2019年2月1日	上午11:31:21
 * ********************************************************* */
uint8_t SM2_GetID(uint8_t* buf)
{
	memcpy(buf,gFrimPara.scyId,16);
	return 16;
}

/* *********************************************************
 *  函数名		:	SM2_GenerateKey
 *  输出结果	:	生成公私密钥对并存储
 *  函数描述	:	生成公私钥对
*  	返回值		:	0:失败 1:成功
 *  输入参数	:	无
 *  时间		:	2019年2月1日	下午4:56:37
 * ********************************************************* */
uint8_t SM2_GenerateKey()
{
	uint8_t rv = 1;
	os_mut_wait (&sm2Mutex, 0xffff);
	if(sm2Type == TMS)
	{
		if (SUCCEED != tms_sm2_genkeypair(TMS_SE_KEY)) {
			rv = 0;
		}
	}
	else
	{
		if (SUCCEED != tmscmpt_sm2_genkeypair(TMSC_SE_PUBKEY,TMSC_SE_PRIKEY)) {
			rv = 0;
		}
	}
	os_mut_release (&sm2Mutex);
	return rv;
}


/* *********************************************************
 *  函数名		:	SM2_GedID
 *  输出结果	: 	无
 *  函数描述	:	获取加密模块的ID等硬件信息
 *  返回值		:	无
 *  输入参数	:	ID		加密模块的ID信息结构体指针
 *  时间			:	2019年2月1日	上午11:31:21
 * ********************************************************* */
uint8_t SM2_SetID(uint8_t* buf)
{
	memcpy(gFrimPara.scyId,buf,16);
	Device_ID_Save();
	return 1;
}

/* *********************************************************
 *  函数名		:	SM2_GetPubKey
*  输出结果	: 0:失败 1:成功
 *  函数描述	:	获取加密模块的公钥值
 *  返回值		:	无
 *  输入参数	:	公钥存储地址
 *  时间			:	2019年2月1日	上午11:31:21
 * ********************************************************* */
uint8_t SM2_GetPubKey(u8 select,u8* buf)
{
	uint32_t len = 64;
	if(select >= 3)
		return 0;
	os_mut_wait (&sm2Mutex, 0xffff);
	if(sm2Type == TMS)
	{
		uint16_t key[3] = {TMS_SE_KEY,TMS_REAL_KEY,TMS_SETUP_KEY};
		if (SUCCEED != tms_sm2_export_pubkey(key[select],buf,&len)) {
			len = 0;
		}
	}
	else
	{
		uint16_t key[3] = {TMSC_SE_PUBKEY,TMSC_REAL_PUBKEY,TMSC_SETUP_PUBKEY};
		if (SUCCEED != tmscmpt_sm2_export_pubkey(key[select],buf,&len)) {
			len = 0;
		}
	}
	os_mut_release (&sm2Mutex);
	return len;
}


/* *********************************************************
 *  函数名		:	SM2_WritePrivKey
 *  输出结果	:
 *  函数描述	:	私钥签名
 *  返回值		:
 *  输入参数	:	com1
 *  时间			:	2019年2月1日	下午4:56:37
 * ********************************************************* */
uint8_t* SM2_PrivKeySign(uint8_t *source,uint16_t courceLen)
{
	uint32_t len = 64;
	uint8_t* data = sm2Data;
	os_mut_wait (&sm2Mutex, 0xffff);
	if(sm2Type == TMS)
	{
		if (SUCCEED != tms_sm2_with_sm3_signature(source,courceLen,sm2Data,&len,TMS_SE_KEY,(uint8_t *)gFrimPara.scyId)) {
			data = NULL;
		}
	}
	else
	{
		if (SUCCEED != tmscmpt_sm2_with_sm3_signature(source,courceLen,sm2Data,&len,TMSC_SE_PRIKEY,(uint8_t *)gFrimPara.scyId)) {
			data = NULL;
		}
	}
	os_mut_release (&sm2Mutex);
	return data;
}

/* *********************************************************
 *  函数名		:	SM2_SetPubKey
 *  输出结果	:	公钥长度及公钥值
 *  函数描述	:	设置芯片公钥值
*  返回值		:	0:失败 1:成功
*  输入参数	:	select 0:平台公钥 1:备案公钥，PubKey 公钥存放地址
 *  时间			:	2019年2月1日	下午4:56:37
 * ********************************************************* */
uint8_t SM2_SetPubKey(uint8_t select,uint8_t * PubKey)
{
	uint8_t len = 1;
	if(select >= 2)
		return 0;	
	os_mut_wait (&sm2Mutex, 0xffff);
	if(sm2Type == TMS)
	{
		uint16_t key[2] = {TMS_REAL_KEY,TMS_SETUP_KEY};
		if (SUCCEED != tms_sm2_import_pubkey(key[select],PubKey,64)) {
			len = 0;
		}
	}
	else
	{
		uint16_t key[2] = {TMSC_REAL_PUBKEY,TMSC_SETUP_PUBKEY};
		if (SUCCEED != tmscmpt_sm2_import_pubkey(key[select],PubKey,64)) {
			len = 0;
		}
	}
	os_mut_release (&sm2Mutex);
	return len;
}


/* *********************************************************
 *  函数名		:	SM2_PubKeyEncryp
 *  输出结果	:	加密数据
 *  函数描述	:	公钥加密
 *  返回值		:	加密结果
 *  输入参数	:	公钥类型 0:平台公钥 1:备案公钥，需加密数据及长度
 *  时间			:	2019年2月1日	下午5:15:54
 * ********************************************************* */
uint8_t* SM2_PubKeyEncryp(uint8_t select,uint8_t *source,uint16_t courceLen)
{
	uint32_t len = sizeof(sm2Data);
	uint8_t* data = sm2Data;
	if(select >= 3)
		return 0;	
	os_mut_wait (&sm2Mutex, 0xffff);
	if(sm2Type == TMS)
	{
		uint16_t key[3] = {TMS_REAL_KEY,TMS_SETUP_KEY,TMS_SE_KEY};
		if (SUCCEED != tms_sm2_encrypt(source,courceLen,sm2Data,&len,key[select])) {
			data = NULL;
		}
	}
	else
	{
		uint16_t key[3] = {TMSC_REAL_PUBKEY,TMSC_SETUP_PUBKEY,TMSC_SE_PUBKEY};
		if (SUCCEED != tmscmpt_sm2_encrypt(source,courceLen,sm2Data,&len,key[select])) {
			data = NULL;
		}
	}
	os_mut_release (&sm2Mutex);
	return data;
}

/* *********************************************************
 *  函数名		:	SM2_PubKeyEncryp
 *  输出结果	:	解密数据
 *  函数描述	:	数据解密
 *  返回值		:	解密结果
 *  输入参数	:	需密数据及长度
 *  时间			:	2019年2月1日	下午5:15:54
 * ********************************************************* */
uint8_t* SM2_PriDecryp(uint8_t *source,uint16_t courceLen)
{
	uint32_t len = sizeof(sm2Data);
	uint8_t* data = sm2Data;
	os_mut_wait (&sm2Mutex, 0xffff);
	if(sm2Type == TMS)
	{
		if (SUCCEED != tms_sm2_decrypt(source,courceLen,sm2Data,&len,TMS_SE_KEY)) {
			data = NULL;
		}
	}
	else
	{
		if (SUCCEED != tmscmpt_sm2_decrypt(source,courceLen,sm2Data,&len,TMSC_SE_PRIKEY)) {
			data = NULL;
		}
	}
	os_mut_release (&sm2Mutex);
	return data;
}
