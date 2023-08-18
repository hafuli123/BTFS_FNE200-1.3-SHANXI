#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "common.h"
#include "driver.h"

int tms_sm3_init(void)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int off = 0, len = 0;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0xFC;
    buf[2] = PKG_FIRST;//first block
    buf[3] = HASH_SM3;
    buf[4] = 0x00;

    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.tx_len = APDU_HEADER_SIZE;
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

int tms_sm3_update(unsigned char *data, unsigned int data_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int off = 0, len = 0;

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

    if (data_len <= 0x00) {
        printf("[%s] %d :parameter data_len(%d) is invalid.\n", __func__, __LINE__, data_len);
        rv = FAILED;
        goto end;
    }

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0xFC;
    buf[2] = PKG_MID;//mid block
    buf[3] = HASH_SM3;

    if (data_len <= APDU_BODY_MAX_SIZE) {
        buf[4] = (unsigned char)data_len;
        memcpy(buf + APDU_HEADER_SIZE, data, data_len);

        apdu.max_wait_time = 1;
        apdu.tx = buf;
        apdu.tx_len = data_len + APDU_HEADER_SIZE;
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
            rv = FAILED;
            goto end;
        }
    }
    else {
        while (off < data_len) {

            if ((data_len - off) > APDU_BODY_MAX_SIZE) {
                len = APDU_BODY_MAX_SIZE;
            } else {
                len = (data_len - off);
            }

            buf[4] = (unsigned char)len;
            memcpy(buf + APDU_HEADER_SIZE, data+off, len);
            off += len;

            apdu.max_wait_time = 1;
            apdu.tx = buf;
            apdu.tx_len = APDU_HEADER_SIZE + len;
            apdu.rx = RecvBuf;
            apdu.rx_len = sizeof(RecvBuf);
            rv = transmit_apdu(&apdu);
            if(rv != SUCCEED)    {
                printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
                rv = FAILED;
                goto end;
            }
        }  
    }

end:
    
    return rv;
}

int tms_sm3_final(unsigned char *data, unsigned int data_len, unsigned char *hash, unsigned int *hash_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int off = 0, len = 0;

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

    if (data_len <= 0x00) {
        printf("[%s] %d :parameter data_len(%d) is invalid.\n", __func__, __LINE__, data_len);
        rv = FAILED;
        goto end;
    }

    if (hash_len == NULL) {
        printf("[%s] %d :parameter hash_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (hash == NULL) {//返回实际响应长度
        *hash_len = SM3_HASH_SIZE;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*hash_len < SM3_HASH_SIZE) {
            printf("[%s] %d :hash_len(%d) is less than %d bytes. \n", __func__, __LINE__, *hash_len, SM3_HASH_SIZE);
            rv = FAILED;
            goto end;
        }
    }

    *hash_len = 0;

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0xFC;
    buf[2] = PKG_MID;//mid block
    buf[3] = HASH_SM3;

    if (data_len <= APDU_BODY_MAX_SIZE) {
        buf[2] = PKG_LAST;//last block
        buf[4] = (unsigned char)data_len;
        memcpy(buf + APDU_HEADER_SIZE, data, data_len);

        apdu.max_wait_time = 1;
        apdu.tx = buf;
        apdu.tx_len = data_len + APDU_HEADER_SIZE;
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
            rv = FAILED;
            goto end;
        }
    }
    else {
        while (off < data_len) {

            if ((data_len - off) > APDU_BODY_MAX_SIZE) {
                len = APDU_BODY_MAX_SIZE;
            } else {
                len = (data_len - off);
                buf[2] = PKG_LAST;//last block
            }

            buf[4] = (unsigned char)len;
            memcpy(buf + APDU_HEADER_SIZE, data+off, len);
            off += len;

            apdu.max_wait_time = 1;
            apdu.tx = buf;
            apdu.tx_len = APDU_HEADER_SIZE + len;
            apdu.rx = RecvBuf;
            apdu.rx_len = sizeof(RecvBuf);
            rv = transmit_apdu(&apdu);
            if(rv != SUCCEED)    {
                printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
                rv = FAILED;
                goto end;
            }
        }  
    }

    if(apdu.rx_len) {
        if (hash) {
            memcpy(hash, apdu.rx, apdu.rx_len);
        }

        if (hash_len) {
            *hash_len = apdu.rx_len;
        }
    }

end:
    
    return rv;
}


int tms_sm3(unsigned char *data, unsigned int data_len, unsigned char *hash, unsigned int *hash_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int off = 0, len = 0;

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

    if (data_len <= 0x00) {
        printf("[%s] %d :parameter data_len(%d) is invalid.\n", __func__, __LINE__, data_len);
        rv = FAILED;
        goto end;
    }

    if (hash_len == NULL) {
        printf("[%s] %d :parameter hash_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (hash == NULL) {//返回实际响应长度
        *hash_len = SM3_HASH_SIZE;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*hash_len < SM3_HASH_SIZE) {
            printf("[%s] %d :hash_len(%d) is less than %d bytes. \n", __func__, __LINE__, *hash_len, SM3_HASH_SIZE);
            rv = FAILED;
            goto end;
        }
    }

    *hash_len = 0;

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0xFC;
    buf[2] = PKG_UNIQUE;
    buf[3] = HASH_SM3;

    if (data_len <= APDU_BODY_MAX_SIZE) {
        buf[4] = (unsigned char)data_len;
        memcpy(buf + APDU_HEADER_SIZE, data, data_len);

        apdu.max_wait_time = 1;
        apdu.tx = buf;
        apdu.tx_len = data_len + APDU_HEADER_SIZE;
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
            rv = FAILED;
            goto end;
        }
    }
    else {
        while (off < data_len) {

            if ((data_len - off) > APDU_BODY_MAX_SIZE) {
                len = APDU_BODY_MAX_SIZE;
                if (off == 0x00) {
                    buf[2] = PKG_FIRST;//first block
                } else {
                    buf[2] = PKG_MID;//mid block
                }
            } else {
                len = (data_len - off);
                buf[2] = PKG_LAST;//last block
            }

            buf[4] = (unsigned char)len;
            memcpy(buf + APDU_HEADER_SIZE, data+off, len);
            off += len;

            apdu.max_wait_time = 1;
            apdu.tx = buf;
            apdu.tx_len = APDU_HEADER_SIZE + len;
            apdu.rx = RecvBuf;
            apdu.rx_len = sizeof(RecvBuf);
            rv = transmit_apdu(&apdu);
            if(rv != SUCCEED)    {
                printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
                rv = FAILED;
                goto end;
            }
        }  
    }
    
    if(apdu.rx_len) {
        if (hash) {
            memcpy(hash, apdu.rx, apdu.rx_len);
        }

        if (hash_len) {
            *hash_len = apdu.rx_len;
        }
    }

end:
    
    return rv;
}



