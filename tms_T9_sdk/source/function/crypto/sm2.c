#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

int tms_sm2_genkeypair(unsigned short sm2ID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;
    apduBuf[0] = 0x80;
    apduBuf[1] = 0xA2;
    apduBuf[2] = 0x00;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = SM2_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    apduBuf[dataLen++] = (unsigned char)(SM2_BIT_LENGTH >> 8);
    apduBuf[dataLen++] = (unsigned char)(SM2_BIT_LENGTH & 0x00FF);
    apduBuf[dataLen++] = 0x00;
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        rv = FAILED;
    }
    
    return rv;
}

int tms_sm2_import_pubkey(unsigned short sm2ID, unsigned char *pub_key, unsigned int pub_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (pub_key == NULL) {
        return FAILED;
    }

    if (pub_key_len != SM2_PUBKEY_SIZE) {
        return FAILED;
    }

    apduBuf[0] = 0x80;
    apduBuf[1] = 0xA6;
    apduBuf[2] = 0x00;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = SM2_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    apduBuf[dataLen++] = 0x00;
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
        return FAILED;
    }
    
    return rv;
}

int tms_sm2_import_prikey(unsigned short sm2ID, unsigned char *pri_key, unsigned int pri_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (pri_key == NULL) {
        return FAILED;
    }

    if (pri_key_len != SM2_PRIKEY_SIZE) {
        return FAILED;
    }

    apduBuf[0] = 0x80;
    apduBuf[1] = 0xA8;
    apduBuf[2] = 0x00;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = SM2_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    apduBuf[dataLen++] = 0x00;
    memcpy(apduBuf + dataLen, pri_key, pri_key_len);
    dataLen += pri_key_len;
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;
    }
    
    return rv;
}

int tms_sm2_export_pubkey(unsigned short sm2ID, unsigned char *pub_key, unsigned int *pub_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (pub_key_len == NULL) {
        return FAILED;
    }

    if (pub_key == NULL) {//返回实际响应长度
        *pub_key_len = SM2_PUBKEY_SIZE;
        return rv;
    }
    else {//判断输出Buffer是否足够大
        if (*pub_key_len < SM2_PUBKEY_SIZE) {
            return FAILED;
        }
    }

    *pub_key_len = 0;

    apduBuf[0] = 0x80;
    apduBuf[1] = 0xAC;
    apduBuf[2] = 0x00;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = SM2_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
		if(rv == 0x6974 && sm2ID == 0x0001)
		{
			if(tms_sm2_genkeypair(0x0001) == SUCCEED)
				tms_sm2_export_pubkey(sm2ID,pub_key,pub_key_len);
		}
    if(rv != SUCCEED)    {
        return FAILED;
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

int tms_sm2_encrypt(unsigned char *data, unsigned int data_len,
                    unsigned char *encrypted_data, unsigned int *encrypted_data_len,
                    unsigned short sm2ID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (data == NULL) {
        return FAILED;
    }

    if (data_len <= 0x00 || data_len > SM2_PLAIN_MAX) {
        return FAILED;
    }

    if (encrypted_data_len == NULL) {
        return FAILED;
    }

    if (encrypted_data == NULL) {//返回实际响应长度
        *encrypted_data_len = (data_len + SM2_CIPHER_ATTACHMENT);
    }
    else {//判断输出Buffer是否足够大
        if (*encrypted_data_len < (data_len + SM2_CIPHER_ATTACHMENT)) {
            return FAILED;
        }
    }

    apduBuf[0] = 0x80;
    apduBuf[1] = 0x40;
    apduBuf[2] = PKG_FIRST;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;
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
            return FAILED;
        }
    }
        
    *encrypted_data_len = apdu.rx_len;
    
    return rv;
}

int tms_sm2_decrypt(unsigned char *data, unsigned int data_len,
                    unsigned char *decrypted_data, unsigned int *decrypted_data_len, 
                    unsigned short sm2ID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (data == NULL) {
        return FAILED;
    }

    if (data_len <= 0x00 || data_len > SM2_CIPHER_MAX) {
        return FAILED;
    }

    if (decrypted_data_len == NULL) {
        return FAILED;
    }

    if (decrypted_data == NULL) {//返回实际响应长度
        *decrypted_data_len = (data_len - SM2_CIPHER_ATTACHMENT);
        return rv;
    }
    else {//判断输出Buffer是否足够大
        if (*decrypted_data_len < (data_len - SM2_CIPHER_ATTACHMENT)) {
            return FAILED;
        }
    }

    apduBuf[0] = 0x80;
    apduBuf[1] = 0x42;
    apduBuf[2] = PKG_FIRST;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;
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
            return FAILED;
        }
    }    

    *decrypted_data_len = apdu.rx_len;
    
    return rv;
}

int tms_sm2_with_sm3_signature(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int *sign_data_len,
                    unsigned short sm2ID, unsigned char userid[16])
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char default_id[] = {"1234567812345678"};
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (data == NULL) {
        return FAILED;
    }

    if (data_len <= 0x00) {
        return FAILED;
    }

    if (sign_data_len == NULL) {
        return FAILED;
    }

    if (sign_data == NULL) {//返回实际响应长度
        *sign_data_len = SM2_SIGNATURE_LEN;
        return rv;
    }
    else {//判断输出Buffer是否足够大
        if (*sign_data_len < SM2_SIGNATURE_LEN) {
            return FAILED;
        }
    }

    *sign_data_len = 0;

    apduBuf[0] = 0x80;
    apduBuf[1] = 0x44;
    apduBuf[2] = PKG_FIRST;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    apduBuf[dataLen++] = (unsigned char)(SM2_USERID_SIZE >> 8);
    apduBuf[dataLen++] = (unsigned char)(SM2_USERID_SIZE & 0x00FF);
    if (userid) {
        memcpy(apduBuf + dataLen, userid, SM2_USERID_SIZE);
    }
    else {
        memcpy(apduBuf + dataLen, default_id, SM2_USERID_SIZE);
    }
    dataLen += SM2_USERID_SIZE;
  
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;
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
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            return FAILED;
        }
    }    
    
    if(apdu.rx_len) {
        if (sign_data) {
            memcpy(sign_data, apdu.rx, apdu.rx_len);
        }

        if (sign_data_len) {
            *sign_data_len = apdu.rx_len;
        }
    }
    
    return rv;
}

int tms_sm2_with_sm3_verify(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int sign_data_len,
                    unsigned short sm2ID, unsigned char userid[16])
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char default_id[] = {"1234567812345678"};
    int dataLen = 0, off = 0, len = 0;

    if (isInitialize()) {
        return FAILED;
    }

    if (data == NULL) {
        return FAILED;
    }

    if (data_len <= 0x00) {
        return FAILED;
    }

    if (sign_data == NULL) {
        return FAILED;
    }

    if (sign_data_len != SM2_SIGNATURE_LEN) {
        return FAILED;
    }

    apduBuf[0] = 0x80;
    apduBuf[1] = 0x48;
    apduBuf[2] = PKG_FIRST;
    apduBuf[3] = 0x00;

		dataLen = APDU_HEADER_SIZE;
    apduBuf[dataLen++] = (unsigned char)(sm2ID >> 8);
    apduBuf[dataLen++] = (unsigned char)(sm2ID & 0x00FF);
    memcpy(apduBuf + dataLen, sign_data, sign_data_len);
    dataLen += sign_data_len;
    apduBuf[dataLen++] = (unsigned char)(SM2_USERID_SIZE >> 8);
    apduBuf[dataLen++] = (unsigned char)(SM2_USERID_SIZE & 0x00FF);
    if (userid) {
        memcpy(apduBuf + dataLen, userid, SM2_USERID_SIZE);
    }
    else {
        memcpy(apduBuf + dataLen, default_id, SM2_USERID_SIZE);
    }
    dataLen += SM2_USERID_SIZE;
  
    apduBuf[4] = dataLen - APDU_HEADER_SIZE;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.tx_len = dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;
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
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            return FAILED;
        }
    }    
    
    return rv;
}

