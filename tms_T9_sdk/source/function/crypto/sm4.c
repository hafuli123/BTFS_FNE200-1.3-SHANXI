#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

#define MODE_ECB    0xAA
#define MODE_CBC    0x55
#define FILE_KEY        0x00
#define EXT_KEY         0x01

unsigned char flag_sm4;
static unsigned char init_mode;

int tms_sm4_genkey(unsigned short keyid)
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

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0xA0;
    buf[2] = 0x00;
    buf[3] = 0x00;

    //set apdu body
    apduBuf[dataLen++] = SM4_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(keyid >> 8);
    apduBuf[dataLen++] = (unsigned char)(keyid & 0x00FF);
    apduBuf[dataLen++] = SYM_BLOCK_LEN;
    
    buf[4] = (unsigned char)dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.max_wait_time = 1;
    apdu.tx = buf;
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

int tms_sm4_import_key(unsigned char *keyval, unsigned int key_len, unsigned short keyid)
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

    if (keyval == NULL) {
        printf("[%s] %d :parameter keyval is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (key_len != SYM_BLOCK_LEN) {
        printf("[%s] %d :parameter key_len(%d) is invalid.\n", __func__, __LINE__, key_len);
        rv = FAILED;
        goto end;
    }

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0xA4;
    buf[2] = 0x00;
    buf[3] = 0x00;

    //set apdu body
    apduBuf[dataLen++] = SM4_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(keyid >> 8);
    apduBuf[dataLen++] = (unsigned char)(keyid & 0x00FF);
    memcpy(apduBuf + dataLen, keyval, key_len);
    dataLen += key_len;
    
    buf[4] = (unsigned char)dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.max_wait_time = 1;
    apdu.tx = buf;
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

int tms_sm4_init_ext(unsigned char *key_val, unsigned int key_len,
                    unsigned char *iv_val, unsigned int iv_len, unsigned int enc)
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

    if (key_val == NULL) {
        printf("[%s] %d :parameter keyval is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (key_len != SYM_BLOCK_LEN) {
        printf("[%s] %d :parameter key_len(%d) is invalid.\n", __func__, __LINE__, key_len);
        rv = FAILED;
        goto end;
    }

    if (iv_val != NULL) {//CBC mode
        if (iv_len != SYM_BLOCK_LEN) {
            printf("[%s] %d :cbc mode parameter iv_len(%d) is invalid.\n", __func__, __LINE__, iv_len);
            rv = FAILED;
            goto end;
        }
    }

    if (enc != 0x00 && enc != 0x01) {
        printf("[%s] %d :parameter enc(%d) is invalid.\n", __func__, __LINE__, enc);
        rv = FAILED;
        goto end;
    }

    if (iv_val == NULL) {
        //ECB模式
        
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x68;
        buf[2] = PKG_FIRST;
        buf[3] = EXT_KEY;

        //set apdu body
        dataLen = 0;
        apduBuf[dataLen++] = enc;
        apduBuf[dataLen++] = 0x00;
        memcpy(apduBuf + dataLen, key_val, key_len);
        dataLen += key_len;
        flag_sm4 = MODE_ECB;
    }
    else {
        //CBC模式
        
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x62;
        buf[2] = PKG_FIRST;
        buf[3] = EXT_KEY;

        //set apdu body
        dataLen = 0;
        apduBuf[dataLen++] = enc;
        apduBuf[dataLen++] = 0x00;
        memcpy(apduBuf + dataLen, key_val, key_len);
        dataLen += key_len;
        memcpy(apduBuf + dataLen, iv_val, iv_len);
        dataLen += iv_len;
        flag_sm4 = MODE_CBC;
    }
    
    buf[4] = (unsigned char)dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);
    
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

    init_mode = EXT_KEY;
        
end:
    
    return rv;
}

int tms_sm4_init(unsigned short keyid, unsigned char *iv_val, unsigned int iv_len, unsigned int enc)
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

    if (iv_val != NULL) {//CBC mode
        if (iv_len != SYM_BLOCK_LEN) {
            printf("[%s] %d :cbc mode parameter iv_len(%d) is invalid.\n", __func__, __LINE__, iv_len);
            rv = FAILED;
            goto end;
        }
    }

    if (enc != 0x00 && enc != 0x01) {
        printf("[%s] %d :parameter enc(%d) is invalid.\n", __func__, __LINE__, enc);
        rv = FAILED;
        goto end;
    }

    if (iv_val == NULL) {
        //ECB模式
        
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x68;
        buf[2] = PKG_FIRST;
        buf[3] = FILE_KEY;

        //set apdu body
        dataLen = 0;
        apduBuf[dataLen++] = enc;
        apduBuf[dataLen++] = (unsigned char)(keyid >> 8);
        apduBuf[dataLen++] = (unsigned char)(keyid & 0x00FF);
        flag_sm4 = MODE_ECB;
    }
    else {
        //CBC模式
        
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x62;
        buf[2] = PKG_FIRST;
        buf[3] = FILE_KEY;

        //set apdu body
        dataLen = 0;
        apduBuf[dataLen++] = enc;
        apduBuf[dataLen++] = (unsigned char)(keyid >> 8);
        apduBuf[dataLen++] = (unsigned char)(keyid & 0x00FF);
        memcpy(apduBuf + dataLen, iv_val, iv_len);
        dataLen += iv_len;
        flag_sm4 = MODE_CBC;
    }
    
    buf[4] = (unsigned char)dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);
    
    apdu.tx_len = APDU_HEADER_SIZE + dataLen;
    apdu.max_wait_time = 1;
    apdu.tx = buf;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

    init_mode = FILE_KEY;

end:
    
    return rv;
}

int tms_sm4_update(unsigned char * indata, unsigned int indata_len, 
                                unsigned char *outdata, unsigned int * outdata_len)
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

    if (indata == NULL) {
        printf("[%s] %d :parameter indata is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if ((indata_len <= 0x00) || (indata_len % SYM_BLOCK_LEN != 0)) {
        printf("[%s] %d :parameter indata_len(%d) is invalid.\n", __func__, __LINE__, indata_len);
        rv = FAILED;
        goto end;
    }

    if (outdata_len == NULL) {
        printf("[%s] %d :parameter outdata_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (outdata == NULL) {//返回实际响应长度
        *outdata_len = indata_len;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*outdata_len < indata_len) {
            printf("[%s] %d :outdata_len(%d) is less than %d bytes. \n", __func__, __LINE__, *outdata_len, indata_len);
            rv = FAILED;
            goto end;
        }
    }

    *outdata_len = 0;

    if (flag_sm4 == MODE_ECB) {
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x68;
        buf[2] = PKG_MID;
        buf[3] = init_mode;
    }
    else {
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x62;
        buf[2] = PKG_MID;
        buf[3] = init_mode;
    }

    if (indata_len <= SYM_BODY_MAX_SIZE) {
        buf[4] = (unsigned char)indata_len;
        memcpy(buf + APDU_HEADER_SIZE, indata, indata_len);

        apdu.max_wait_time = 1;
        apdu.tx = buf;
        apdu.tx_len = APDU_HEADER_SIZE + indata_len;
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
            rv = FAILED;
            goto end;
        }

        if(apdu.rx_len) {
            if (outdata) {
                memcpy(outdata, apdu.rx, apdu.rx_len);
            }
            if (outdata_len) {
                *outdata_len = apdu.rx_len;
            }
        }
    }
    else {
        while (off < indata_len) {

            if ((indata_len - off) >= SYM_BODY_MAX_SIZE) {
                len = SYM_BODY_MAX_SIZE;
            } else {
                len = (indata_len - off);
            }

            buf[4] = (unsigned char)len;
            memcpy(buf + APDU_HEADER_SIZE, indata+off, len);

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

            if (apdu.rx_len) {
                if (outdata) {
                    memcpy(outdata + off, apdu.rx, apdu.rx_len);
                }
                if (outdata_len) {
                    *outdata_len += apdu.rx_len;
                }
            }
            off += len;
        }
    }

end:
    
    return rv;
}
int tms_sm4_final(unsigned char *indata, unsigned int indata_len,
                                unsigned char *outdata, unsigned int *outdata_len)
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

    if (indata == NULL) {
        printf("[%s] %d :parameter indata is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if ((indata_len <= 0x00) || (indata_len % SYM_BLOCK_LEN != 0)) {
        printf("[%s] %d :parameter indata_len(%d) is invalid.\n", __func__, __LINE__, indata_len);
        rv = FAILED;
        goto end;
    }

    if (outdata_len == NULL) {
        printf("[%s] %d :parameter outdata_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (outdata == NULL) {//返回实际响应长度
        *outdata_len = indata_len;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*outdata_len < indata_len) {
            printf("[%s] %d :outdata_len(%d) is less than %d bytes. \n", __func__, __LINE__, *outdata_len, indata_len);
            rv = FAILED;
            goto end;
        }
    }

    *outdata_len = 0;

    if (flag_sm4 == MODE_ECB) {
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x68;
        buf[2] = PKG_MID;
        buf[3] = init_mode;
    }
    else {
        //set apdu header
        buf[0] = 0x80;
        buf[1] = 0x62;
        buf[2] = PKG_MID;
        buf[3] = init_mode;
    }

    if (indata_len <= SYM_BODY_MAX_SIZE) {
        buf[2] = PKG_LAST;
        buf[4] = (unsigned char)indata_len;
        memcpy(buf + APDU_HEADER_SIZE, indata, indata_len);

        apdu.max_wait_time = 1;
        apdu.tx = buf;
        apdu.tx_len = APDU_HEADER_SIZE + indata_len;
        apdu.rx = RecvBuf;
        apdu.rx_len = sizeof(RecvBuf);
        rv = transmit_apdu(&apdu);
        if(rv != SUCCEED)    {
            printf("[%s] %d :transmit_apdu failed %04X.\n", __func__, __LINE__, rv);
            rv = FAILED;
            goto end;
        }

        if(apdu.rx_len) {
            if (outdata) {
                memcpy(outdata, apdu.rx, apdu.rx_len);
            }
            if (outdata_len) {
                *outdata_len = apdu.rx_len;
            }
        }
    }
    else {
        
        while (off < indata_len) {

            if ((indata_len - off) >= SYM_BODY_MAX_SIZE) {
                len = SYM_BODY_MAX_SIZE;
            } else {
                buf[2] = PKG_LAST;
                len = (indata_len - off);
            }

            buf[4] = (unsigned char)len;
            memcpy(buf + APDU_HEADER_SIZE, indata+off, len);

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

            if (apdu.rx_len) {
                if (outdata) {
                    memcpy(outdata + off, apdu.rx, apdu.rx_len);
                }
                if (outdata_len) {
                    *outdata_len += apdu.rx_len;
                }
            }
            off += len;
        }
    }
    
end:
    
    return rv;
}



