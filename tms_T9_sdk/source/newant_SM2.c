/*
 * newant_M2.c
 *
 *  Created on: 2019��1��29��
 *      Author: xtgaxm
 */
#include <RTX_CAN.h>
#include "pdu.h"
#include "newant_SM2.h"
#include "common.h"
#include "se.h"

/*
������
0x0001 SE��Կ��
0x0002 �ⲿ1��Կ��
0x0003 �ⲿ2��Կ��

��
0x0001 ͸���ļ�
0x0002 ��Կ����ҵƽ̨��
0x0003 SE ��Կ
0x0004 SE ˽Կ
0x0005 ��Կ������ƽ̨��
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
 *  ������		:	SM2_IOInit
 *  ������	:	��
 *  ��������	:	���ڳ�ʼ��SM2����оƬ
 *  ����ֵ		:	��
 *  �������	:	com1
 *  ʱ��			:	2019��1��29��	����3:00:54
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
 *  ������		:	SM2_GedID
 *  ������	: 	��
 *  ��������	:	��ȡ����ģ���ID��Ӳ����Ϣ
 *  ����ֵ		:	��
 *  �������	:	ID		����ģ���ID��Ϣ�ṹ��ָ��
 *  ʱ��			:2019��2��1��	����11:31:21
 * ********************************************************* */
uint8_t SM2_GetID(uint8_t* buf)
{
	memcpy(buf,gFrimPara.scyId,16);
	return 16;
}

/* *********************************************************
 *  ������		:	SM2_GenerateKey
 *  ������	:	���ɹ�˽��Կ�Բ��洢
 *  ��������	:	���ɹ�˽Կ��
*  	����ֵ		:	0:ʧ�� 1:�ɹ�
 *  �������	:	��
 *  ʱ��		:	2019��2��1��	����4:56:37
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
 *  ������		:	SM2_GedID
 *  ������	: 	��
 *  ��������	:	��ȡ����ģ���ID��Ӳ����Ϣ
 *  ����ֵ		:	��
 *  �������	:	ID		����ģ���ID��Ϣ�ṹ��ָ��
 *  ʱ��			:	2019��2��1��	����11:31:21
 * ********************************************************* */
uint8_t SM2_SetID(uint8_t* buf)
{
	memcpy(gFrimPara.scyId,buf,16);
	Device_ID_Save();
	return 1;
}

/* *********************************************************
 *  ������		:	SM2_GetPubKey
*  ������	: 0:ʧ�� 1:�ɹ�
 *  ��������	:	��ȡ����ģ��Ĺ�Կֵ
 *  ����ֵ		:	��
 *  �������	:	��Կ�洢��ַ
 *  ʱ��			:	2019��2��1��	����11:31:21
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
 *  ������		:	SM2_WritePrivKey
 *  ������	:
 *  ��������	:	˽Կǩ��
 *  ����ֵ		:
 *  �������	:	com1
 *  ʱ��			:	2019��2��1��	����4:56:37
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
 *  ������		:	SM2_SetPubKey
 *  ������	:	��Կ���ȼ���Կֵ
 *  ��������	:	����оƬ��Կֵ
*  ����ֵ		:	0:ʧ�� 1:�ɹ�
*  �������	:	select 0:ƽ̨��Կ 1:������Կ��PubKey ��Կ��ŵ�ַ
 *  ʱ��			:	2019��2��1��	����4:56:37
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
 *  ������		:	SM2_PubKeyEncryp
 *  ������	:	��������
 *  ��������	:	��Կ����
 *  ����ֵ		:	���ܽ��
 *  �������	:	��Կ���� 0:ƽ̨��Կ 1:������Կ����������ݼ�����
 *  ʱ��			:	2019��2��1��	����5:15:54
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
 *  ������		:	SM2_PubKeyEncryp
 *  ������	:	��������
 *  ��������	:	���ݽ���
 *  ����ֵ		:	���ܽ��
 *  �������	:	�������ݼ�����
 *  ʱ��			:	2019��2��1��	����5:15:54
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
