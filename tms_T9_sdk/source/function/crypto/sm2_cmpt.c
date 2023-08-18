/*
文件：sm2_cmpt.h
功能：THD89驱动,旧版兼容
日期：2021/2/1
公司：佛山新源
作者：csj
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

int tmscmpt_sm2_genkeypair(unsigned short sm2PubID,unsigned short sm2PriID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;
    apduBuf[0] = 0x80;
    apduBuf[1] = 0xC8;
    apduBuf[2] = 0x00;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = 0x01;//密钥类型：生成非对称密钥
		apduBuf[dataLen++] = 0x07;//算法：SM2
    apduBuf[dataLen++] = (unsigned char)(sm2PubID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2PubID & 0x00FF);
    apduBuf[dataLen++] = (unsigned char)(sm2PriID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2PriID & 0x00FF);
    apduBuf[dataLen++] = 0x01;
    apduBuf[dataLen++] = 0x00;
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;;
    }
		
    return rv;
}

int tmscmpt_sm2_with_sm3_signature(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int *sign_data_len,
                    unsigned short sm2ID, unsigned char userid[16])
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;;
    }

    if (data == NULL) {
        return FAILED;;
    }

    if (data_len <= 0x00) {
        return FAILED;;
    }

    if (sign_data_len == NULL) {
        return FAILED;;
    }

    if (sign_data == NULL) {//返回实际响应长度
        *sign_data_len = SM2_SIGNATURE_LEN;
        return rv;
    }
    else {//判断输出Buffer是否足够大
        if (*sign_data_len < SM2_SIGNATURE_LEN) {
            return FAILED;;
        }
    }

    *sign_data_len = 0;

		if(data_len < 248)
		{
			apduBuf[0] = 0x80;
			apduBuf[1] = 0xFA;
			apduBuf[2] = 0x00;
			apduBuf[3] = 0x00;

			dataLen = APDU_HEADER_SIZE;
			apduBuf[dataLen++] = 0x02;//操作类型：签名
			apduBuf[dataLen++] = 0x08;//SM2 算法， 内部 SM3 哈希
			apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
			apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
			
			apduBuf[dataLen++] = (unsigned char)(data_len >> 8);
			apduBuf[dataLen++] = (unsigned char)(data_len & 0x00FF);
			memcpy(apduBuf + dataLen, data, data_len);
			dataLen += data_len;
			
			apduBuf[dataLen++] = (unsigned char)(SM2_USERID_SIZE >> 8);
			apduBuf[dataLen++] = (unsigned char)(SM2_USERID_SIZE & 0x00FF);
			memcpy(apduBuf + dataLen, userid, SM2_USERID_SIZE);
			dataLen += SM2_USERID_SIZE;
			
			apduBuf[4] = dataLen - APDU_HEADER_SIZE;

			apdu.max_wait_time = 1;
			apdu.tx = apduBuf;
			apdu.tx_len = dataLen;
			apdu.rx = sign_data;
			apdu.rx_len = *sign_data_len;
			rv = transmit_apdu(&apdu);
			if(rv != SUCCEED)    {
					return FAILED;;
			}
		}
		else
		{
			apduBuf[0] = 0x80;
			apduBuf[1] = 0xFA;
			apduBuf[2] = PKG_FIRST;
			apduBuf[3] = 0x00;

			dataLen = APDU_HEADER_SIZE;
			apduBuf[dataLen++] = 0x02;//操作类型：签名
			apduBuf[dataLen++] = 0x08;//SM2 算法， 内部 SM3 哈希
			apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
			apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
			apduBuf[dataLen++] = (unsigned char)(data_len >> 8);
			apduBuf[dataLen++] = (unsigned char)(data_len & 0x00FF);
			
			data[data_len++] = (unsigned char)(SM2_USERID_SIZE >> 8);
			data[data_len++] = (unsigned char)(SM2_USERID_SIZE & 0x00FF);
			memcpy(data + data_len, userid, SM2_USERID_SIZE);
			data_len += SM2_USERID_SIZE;
			
			memcpy(apduBuf + dataLen, data, 0xF9);
			dataLen += 0xF9;
			off += 0xF9;
			
			apduBuf[4] = dataLen - APDU_HEADER_SIZE;
			
			apdu.max_wait_time = 1;
			apdu.tx = apduBuf;
			apdu.tx_len = dataLen;
			apdu.rx = RecvBuf;
			apdu.rx_len = sizeof(RecvBuf);
			rv = transmit_apdu(&apdu);
			if(rv != SUCCEED)    {
					return FAILED;;
			}

			while (off < data_len) {

					if ((data_len - off) > APDU_BODY_MAX_SIZE) {
							len = APDU_BODY_MAX_SIZE;
							apduBuf[2] = PKG_MID;//mid block
					} else {
							len = (data_len - off);
							apduBuf[2] = PKG_LAST;//last block
					}

					apduBuf[4] = (unsigned char)len;
					memcpy(apduBuf + APDU_HEADER_SIZE, data + off, len);
					off += len;

					apdu.tx = apduBuf;
					apdu.tx_len = APDU_HEADER_SIZE + len;
					apdu.rx = sign_data;
					apdu.rx_len = *sign_data_len;
					rv = transmit_apdu(&apdu);
					if(rv != SUCCEED)    {
							return FAILED;;
					}
			}
		}
		
		*sign_data_len = apdu.rx_len;
    
    return rv;
}


int tmscmpt_sm2_encrypt(unsigned char *data, unsigned int data_len,
                    unsigned char *encrypted_data, unsigned int *encrypted_data_len,
                    unsigned short sm2ID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;;
    }

    if (data == NULL) {
        return FAILED;;
    }

    if (data_len <= 0x00 || data_len > SM2_PLAIN_MAX) {
        return FAILED;;
    }

    if (encrypted_data_len == NULL) {
        return FAILED;;
    }

    if (encrypted_data == NULL) {//返回实际响应长度
        *encrypted_data_len = (data_len + SM2_CIPHER_ATTACHMENT);
        return rv;
    }
    else {//判断输出Buffer是否足够大
        if (*encrypted_data_len < (data_len + SM2_CIPHER_ATTACHMENT)) {
            return FAILED;;
        }
    }

		if(data_len < 250)
		{
			apduBuf[0] = 0x80;
			apduBuf[1] = 0xFA;
			apduBuf[2] = 0x00;
			apduBuf[3] = 0x00;

			dataLen = APDU_HEADER_SIZE;
			apduBuf[dataLen++] = 0x00;//操作类型：加密
			apduBuf[dataLen++] = 0x07;//算法SM2算法
			apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
			apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
			apduBuf[dataLen++] = (unsigned char)(data_len >> 8);
			apduBuf[dataLen++] = (unsigned char)(data_len & 0x00FF);
			memcpy(apduBuf + dataLen, data, data_len);
			dataLen += data_len;
			apduBuf[4] = dataLen - APDU_HEADER_SIZE;

			apdu.max_wait_time = 1;
			apdu.tx = apduBuf;
			apdu.tx_len = dataLen;
			apdu.rx = encrypted_data;
			apdu.rx_len = *encrypted_data_len;
			rv = transmit_apdu(&apdu);
			if(rv != SUCCEED)    {
					return FAILED;;
			}
		}
		else
		{
			apduBuf[0] = 0x80;
			apduBuf[1] = 0xFA;
			apduBuf[2] = PKG_FIRST;
			apduBuf[3] = 0x00;

			dataLen = APDU_HEADER_SIZE;
			apduBuf[dataLen++] = 0x00;//操作类型：加密
			apduBuf[dataLen++] = 0x07;//算法SM2算法
			apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
			apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
			apduBuf[dataLen++] = (unsigned char)(data_len >> 8);
			apduBuf[dataLen++] = (unsigned char)(data_len & 0x00FF);
			
			memcpy(apduBuf + dataLen, data, 0xF9);
			dataLen += 0xF9;
			off += 0xF9;
			
			apduBuf[4] = dataLen - APDU_HEADER_SIZE;
			
			apdu.max_wait_time = 1;
			apdu.tx = apduBuf;
			apdu.tx_len = dataLen;
			apdu.rx = RecvBuf;
			apdu.rx_len = sizeof(RecvBuf);
			rv = transmit_apdu(&apdu);
			if(rv != SUCCEED)    {
					return FAILED;;
			}

			while (off < data_len) {

					if ((data_len - off) > APDU_BODY_MAX_SIZE) {
							len = APDU_BODY_MAX_SIZE;
							apduBuf[2] = PKG_MID;//mid block
					} else {
							len = (data_len - off);
							apduBuf[2] = PKG_LAST;//last block
					}

					apduBuf[4] = (unsigned char)len;
					memcpy(apduBuf + APDU_HEADER_SIZE, data + off, len);
					off += len;

					apdu.tx = apduBuf;
					apdu.tx_len = APDU_HEADER_SIZE + len;
					apdu.rx = encrypted_data;
					apdu.rx_len = *encrypted_data_len;
					rv = transmit_apdu(&apdu);
					if(rv != SUCCEED)    {
							return FAILED;;
					}
			}
		}

    *encrypted_data_len = apdu.rx_len;
    
    return rv;
}


int tmscmpt_sm2_decrypt(unsigned char *data, unsigned int data_len,
                    unsigned char *decrypted_data, unsigned int *decrypted_data_len, 
                    unsigned short sm2ID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;;
    }

    if (data == NULL) {
        return FAILED;;
    }

    if (data_len <= 0x00 || data_len > SM2_CIPHER_MAX) {
        return FAILED;;
    }

    if (decrypted_data_len == NULL) {
        return FAILED;;
    }

    if (decrypted_data == NULL) {//返回实际响应长度
        *decrypted_data_len = (data_len - SM2_CIPHER_ATTACHMENT);
        return rv;
    }
    else {//判断输出Buffer是否足够大
        if (*decrypted_data_len < (data_len - SM2_CIPHER_ATTACHMENT)) {
            return FAILED;;
        }
    }

		if(data_len < 250)
		{
			apduBuf[0] = 0x80;
			apduBuf[1] = 0xFA;
			apduBuf[2] = 0x00;
			apduBuf[3] = 0x00;

			dataLen = APDU_HEADER_SIZE;
			apduBuf[dataLen++] = 0x01;//操作类型：解密
			apduBuf[dataLen++] = 0x07;//算法SM2算法
			apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
			apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
			apduBuf[dataLen++] = (unsigned char)(data_len >> 8);
			apduBuf[dataLen++] = (unsigned char)(data_len & 0x00FF);
			memcpy(apduBuf + dataLen, data, data_len);
			dataLen += data_len;
			apduBuf[4] = dataLen - APDU_HEADER_SIZE;

			apdu.max_wait_time = 1;
			apdu.tx = apduBuf;
			apdu.tx_len = dataLen;
			apdu.rx = decrypted_data;
			apdu.rx_len = *decrypted_data_len;
			rv = transmit_apdu(&apdu);
			if(rv != SUCCEED)    {
					return FAILED;;
			}
		}
		else
		{
			apduBuf[0] = 0x80;
			apduBuf[1] = 0xFA;
			apduBuf[2] = PKG_FIRST;
			apduBuf[3] = 0x00;

			dataLen = APDU_HEADER_SIZE;
			apduBuf[dataLen++] = 0x01;//操作类型：解密
			apduBuf[dataLen++] = 0x07;//算法SM2算法
			apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
			apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
			apduBuf[dataLen++] = (unsigned char)(data_len >> 8);
			apduBuf[dataLen++] = (unsigned char)(data_len & 0x00FF);
			
			memcpy(apduBuf + dataLen, data, 0xF9);
			dataLen += 0xF9;
			off += 0xF9;
			
			apduBuf[4] = dataLen - APDU_HEADER_SIZE;
			
			apdu.max_wait_time = 1;
			apdu.tx = apduBuf;
			apdu.tx_len = dataLen;
			apdu.rx = RecvBuf;
			apdu.rx_len = sizeof(RecvBuf);
			rv = transmit_apdu(&apdu);
			if(rv != SUCCEED)    {
					return FAILED;;
			}

			while (off < data_len) {

					if ((data_len - off) > APDU_BODY_MAX_SIZE) {
							len = APDU_BODY_MAX_SIZE;
							apduBuf[2] = PKG_MID;//mid block
					} else {
							len = (data_len - off);
							apduBuf[2] = PKG_LAST;//last block
					}

					apduBuf[4] = (unsigned char)len;
					memcpy(apduBuf + APDU_HEADER_SIZE, data + off, len);
					off += len;

					apdu.tx = apduBuf;
					apdu.tx_len = APDU_HEADER_SIZE + len;
					apdu.rx = decrypted_data;
					apdu.rx_len = *decrypted_data_len;
					rv = transmit_apdu(&apdu);
					if(rv != SUCCEED)    {
							return FAILED;;
					}
			}
		}

    *decrypted_data_len = apdu.rx_len;
    
    return rv;
}


int tmscmpt_sm2_export_pubkey(unsigned short sm2ID, unsigned char *pub_key, unsigned int *pub_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;

    if (isInitialize()) {
        return FAILED;;
    }

    if (pub_key_len == NULL) {
        return FAILED;;
    }

    if (pub_key == NULL) {//返回实际响应长度
        *pub_key_len = SM2_PUBKEY_SIZE;
    }
    else {//判断输出Buffer是否足够大
        if (*pub_key_len < SM2_PUBKEY_SIZE) {
            return FAILED;;
        }
    }

    *pub_key_len = 0;

    apduBuf[0] = 0x80;
    apduBuf[1] = 0xCE;
    apduBuf[2] = 0x01;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    
    apduBuf[4] = 0x02;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;;
    }

    if(apdu.rx_len) {
        if (pub_key) {
            memcpy(pub_key, apdu.rx, apdu.rx_len);
        }

        if (pub_key_len) {
            *pub_key_len = apdu.rx_len;
        }
    }
    
    return rv;
}

int tmscmpt_sm2_import_pubkey(unsigned short sm2ID, unsigned char *pub_key, unsigned int pub_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;

    if (isInitialize()) {
        return FAILED;;
    }

    if (pub_key == NULL) {
        return FAILED;;
    }

    if (pub_key_len != SM2_PUBKEY_SIZE) {
        return FAILED;;
    }

    apduBuf[0] = 0x80;
    apduBuf[1] = 0xCC;
    apduBuf[2] = 0x00;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = 0x01;//密钥类型，导入公钥
		apduBuf[dataLen++] = 0x07;//算法，SM2
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    apduBuf[dataLen++] = (unsigned char)(pub_key_len >> 8);
    apduBuf[dataLen++] = (unsigned char)(pub_key_len & 0x00FF);
    memcpy(apduBuf + dataLen, pub_key, pub_key_len);
    dataLen += pub_key_len;
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;;
    }
    
    return rv;
}
