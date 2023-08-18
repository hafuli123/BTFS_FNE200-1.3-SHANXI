#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

int tms_ecc_genkeypair(unsigned short eccID, tms_curve_param curve)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if ((curve < NIST_P256) || (curve > BRAINPOOL_256)) {
        printf("[%s] %d :parameter curve(%d) is invalid.\n", __func__, __LINE__, curve);
        rv = FAILED;
        goto end;
    }

    buf[0] = 0x80;
    buf[1] = 0xA2;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = ECC_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(eccID >> 8);
    apduBuf[dataLen++] = (unsigned char)(eccID & 0x00FF);
    apduBuf[dataLen++] = (unsigned char)(ECC_BIT_LENGTH >> 8);
    apduBuf[dataLen++] = (unsigned char)(ECC_BIT_LENGTH & 0x00FF);
    apduBuf[dataLen++] = curve;
  
    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

end:
    
    return rv;
}

int tms_ecc_import_pubkey(unsigned short eccID, unsigned char *pub_key, unsigned int pub_key_len, tms_curve_param curve)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (pub_key == NULL) {
        printf("[%s] %d :parameter pub_key is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (pub_key_len != ECC_PUBKEY_SIZE) {
        printf("[%s] %d :parameter pub_key_len(%d) should be %d bytes.\n", __func__, __LINE__, pub_key_len, ECC_PUBKEY_SIZE);
        rv = FAILED;
        goto end;
    }

    if ((curve < NIST_P256) || (curve > BRAINPOOL_256)) {
        printf("[%s] %d :parameter curve(%d) is invalid.\n", __func__, __LINE__, curve);
        rv = FAILED;
        goto end;
    }

    buf[0] = 0x80;
    buf[1] = 0xA6;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = ECC_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(eccID >> 8);
    apduBuf[dataLen++] = (unsigned char)(eccID & 0x00FF);
    apduBuf[dataLen++] = curve;
    memcpy(apduBuf + dataLen, pub_key, pub_key_len);
    dataLen += pub_key_len;
  
    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

end:
    
    return rv;
}

int tms_ecc_import_prikey(unsigned short eccID, unsigned char *pri_key, unsigned int pri_key_len, tms_curve_param curve)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (pri_key == NULL) {
        printf("[%s] %d :parameter pri_key is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (pri_key_len != ECC_PRIKEY_SIZE) {
        printf("[%s] %d :parameter pri_key_len(%d) should be %d bytes.\n", __func__, __LINE__, pri_key_len, ECC_PRIKEY_SIZE);
        rv = FAILED;
        goto end;
    }

    if ((curve < NIST_P256) || (curve > BRAINPOOL_256)) {
        printf("[%s] %d :parameter curve(%d) is invalid.\n", __func__, __LINE__, curve);
        rv = FAILED;
        goto end;
    }

    buf[0] = 0x80;
    buf[1] = 0xA8;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = ECC_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(eccID >> 8);
    apduBuf[dataLen++] = (unsigned char)(eccID & 0x00FF);
    apduBuf[dataLen++] = curve;
    memcpy(apduBuf + dataLen, pri_key, pri_key_len);
    dataLen += pri_key_len;
  
    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

end:
    
    return rv;
}

int tms_ecc_export_pubkey(unsigned short eccID, unsigned char *pub_key, unsigned int *pub_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (pub_key_len == NULL) {
        printf("[%s] %d :parameter pub_key_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (pub_key == NULL) {//返回实际响应长度
        *pub_key_len = ECC_PUBKEY_SIZE;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*pub_key_len < ECC_PUBKEY_SIZE) {
            printf("[%s] %d :pub_key_len(%d) is less than %d bytes. \n", __func__, __LINE__, *pub_key_len, ECC_PUBKEY_SIZE);
            rv = FAILED;
            goto end;
        }
    }

    *pub_key_len = 0;

    buf[0] = 0x80;
    buf[1] = 0xAC;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = ECC_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(eccID >> 8);
    apduBuf[dataLen++] = (unsigned char)(eccID & 0x00FF);

    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

    if(apdu.rx_len) {
        if (pub_key) {
            memcpy(pub_key, apdu.rx, apdu.rx_len);
        }

        if (pub_key_len) {
            *pub_key_len = apdu.rx_len;
        }
    }

end:
    
    return rv;
}

int tms_ecc_signature(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int *sign_data_len,
                    unsigned short eccID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (data == NULL) {
        printf("[%s] %d :parameter data is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (data_len <= 0x00 || data_len > SHA256_HASH_SIZE) {
        printf("[%s] %d :parameter data_len(%d) should be less than or equal to %d bytes.\n", __func__, __LINE__, data_len, SHA256_HASH_SIZE);
        rv = FAILED;
        goto end;
    }

    if (sign_data_len == NULL) {
        printf("[%s] %d :parameter sign_data_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (sign_data == NULL) {//返回实际响应长度
        *sign_data_len = ECC_SIGNATURE_LEN;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*sign_data_len < ECC_SIGNATURE_LEN) {
            printf("[%s] %d :sign_data_len(%d) is less than %d bytes. \n", __func__, __LINE__, *sign_data_len, ECC_SIGNATURE_LEN);
            rv = FAILED;
            goto end;
        }
    }

    *sign_data_len = 0;

    buf[0] = 0x80;
    buf[1] = 0x22;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = (unsigned char)(eccID >> 8);
    apduBuf[dataLen++] = (unsigned char)(eccID & 0x00FF);
    memcpy(apduBuf+dataLen, data, data_len);
    dataLen += data_len;

    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }
    
    if(apdu.rx_len) {
        if (sign_data) {
            memcpy(sign_data, apdu.rx, apdu.rx_len);
        }
        
        if (sign_data_len) {
            *sign_data_len = apdu.rx_len;
        }
    }

end:
    
    return rv;
}

int tms_ecc_verify(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int sign_data_len,
                    unsigned short eccID)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (data == NULL) {
        printf("[%s] %d :parameter data is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (data_len <= 0x00 || data_len > SHA256_HASH_SIZE) {
        printf("[%s] %d :parameter data_len(%d) should be less than or equal to %d bytes.\n", __func__, __LINE__, data_len, SHA256_HASH_SIZE);
        rv = FAILED;
        goto end;
    }

    if (sign_data == NULL) {
        printf("[%s] %d :parameter sign_data is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (sign_data_len != ECC_SIGNATURE_LEN) {
        printf("[%s] %d :parameter sign_data_len(%d) should be %d bytes.\n", __func__, __LINE__, sign_data_len, ECC_SIGNATURE_LEN);
        rv = FAILED;
        goto end;
    }

    buf[0] = 0x80;
    buf[1] = 0x24;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = (unsigned char)(eccID >> 8);
    apduBuf[dataLen++] = (unsigned char)(eccID & 0x00FF);
    memcpy(apduBuf+dataLen, sign_data, sign_data_len);
    dataLen += sign_data_len;
    memcpy(apduBuf+dataLen, data, data_len);
    dataLen += data_len;

    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }
    
end:
    
    return rv;
}





