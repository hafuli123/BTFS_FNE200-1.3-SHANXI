#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

int tms_sha256(unsigned char *data, unsigned int data_len, unsigned char *hash, unsigned int *hash_len);


int tms_rsa_genkeypair(unsigned short rsaID)
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

    buf[0] = 0x80;
    buf[1] = 0xA2;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = RSA_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    apduBuf[dataLen++] = (unsigned char)(RSA_BIT_LENGTH >> 8);
    apduBuf[dataLen++] = (unsigned char)(RSA_BIT_LENGTH & 0x00FF);
    apduBuf[dataLen++] = 0x00;
  
    buf[4] = dataLen;
    memcpy(buf + APDU_HEADER_SIZE, apduBuf, dataLen);
#ifdef INPUT_VOLT_1_8
    apdu.max_wait_time = 28;//28 sec delay @1.8V
#else
    apdu.max_wait_time = 10;//10 sec delay @3.3V/5V
#endif
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

int tms_rsa_import_pubkey(unsigned short rsaID, unsigned char *modulus, unsigned int modulus_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;
    int pubkey_len = RSA_MOD_LEN;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (modulus == NULL) {
        printf("[%s] %d :parameter modulus is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (modulus_len != pubkey_len) {
        printf("[%s] %d :parameter modulus_len(%d) should be %d bytes.\n", __func__, __LINE__, modulus_len, pubkey_len);
        rv = FAILED;
        goto end;
    }

    buf[0] = 0x80;
    buf[1] = 0xA6;
    buf[2] = PKG_FIRST;
    buf[3] = 0x00;

    apduBuf[dataLen++] = RSA_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    apduBuf[dataLen++] = 0x00;
    memcpy(apduBuf + dataLen, modulus, modulus_len);
    dataLen += modulus_len;
  
    while (off < dataLen) {

        if ((dataLen - off) > APDU_BODY_MAX_SIZE) {
            len = APDU_BODY_MAX_SIZE;
            if (off == 0x00) {
                buf[2] = PKG_FIRST;//first block
            }
            else {
                buf[2] = PKG_MID;//mid block
            }
        } else {
            len = (dataLen - off);
            buf[2] = PKG_LAST;//last block
        }

        buf[4] = (unsigned char)len;
        memcpy(buf + APDU_HEADER_SIZE, apduBuf+off, len);
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

end:
    
    return rv;
}

int tms_rsa_import_prikey(unsigned short rsaID, unsigned char *pri_crt_key, unsigned int pri_crt_key_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;
    int prikey_len = ((RSA_BIT_LENGTH >> 4) * 5);

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (pri_crt_key == NULL) {
        printf("[%s] %d :parameter pri_crt_key is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (pri_crt_key_len != prikey_len) {
        printf("[%s] %d :parameter pri_crt_key_len(%d) should be %d bytes.\n", __func__, __LINE__, pri_crt_key_len, prikey_len);
        rv = FAILED;
        goto end;
    }

    buf[0] = 0x80;
    buf[1] = 0xA8;
    buf[2] = PKG_FIRST;
    buf[3] = 0x00;

    apduBuf[dataLen++] = RSA_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    apduBuf[dataLen++] = 0x00;
  
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

    while (off < pri_crt_key_len) {

        if ((pri_crt_key_len - off) > APDU_BODY_MAX_SIZE) {
            len = APDU_BODY_MAX_SIZE;
            buf[2] = PKG_MID;//mid block
        } else {
            len = (pri_crt_key_len - off);
            buf[2] = PKG_LAST;//last block
        }

        buf[4] = (unsigned char)len;
        memcpy(buf + APDU_HEADER_SIZE, pri_crt_key+off, len);
        off += len;

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

end:
    
    return rv;
}

int tms_rsa_export_pubkey(unsigned short rsaID, unsigned char *modulus, unsigned int *modulus_len)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0,len;
    int pubkey_len = RSA_MOD_LEN;

    printf("<%s>\n", __func__);

    if (isInitialize()) {
        rv = FAILED;
        goto end;
    }

    if (modulus_len == NULL) {
        printf("[%s] %d :parameter modulus_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (modulus == NULL) {//返回实际响应长度
        *modulus_len = pubkey_len;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*modulus_len < pubkey_len) {
            printf("[%s] %d :modulus_len(%d) is less than %d bytes. \n", __func__, __LINE__, *modulus_len, pubkey_len);
            rv = FAILED;
            goto end;
        }
    }

    *modulus_len = 0;

    buf[0] = 0x80;
    buf[1] = 0xAC;
    buf[2] = 0x00;
    buf[3] = 0x00;

    apduBuf[dataLen++] = RSA_FILE_TAG;
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
  
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
        if (modulus) {
            memcpy(modulus, apdu.rx, apdu.rx_len);
        }
        
        if (modulus_len) {
            *modulus_len = apdu.rx_len;
        }
    }

end:
    
    return rv;
}

int tms_rsa_encrypt(unsigned char *data, unsigned int data_len,
                    unsigned char *encrypted_data, unsigned int *encrypted_data_len,
                    unsigned short rsaID, tms_pad_type padding)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char hash[SHA256_HASH_SIZE];
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;
    int hash_len;
    int plain_len = (RSA_MOD_LEN - RSA_PKCS1_PADDING_SIZE);

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

    if (data_len <= 0x00 || data_len > plain_len) {
        printf("[%s] %d :parameter data_len(%d) should be less than or equal to %d bytes.\n", __func__, __LINE__, data_len, plain_len);
        rv = FAILED;
        goto end;
    }

    if (encrypted_data_len == NULL) {
        printf("[%s] %d :parameter encrypted_data_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (encrypted_data == NULL) {//返回实际响应长度
        *encrypted_data_len = RSA_MOD_LEN;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*encrypted_data_len < RSA_MOD_LEN) {
            printf("[%s] %d :encrypted_data_len(%d) is less than %d bytes. \n", __func__, __LINE__, *encrypted_data_len, RSA_MOD_LEN);
            rv = FAILED;
            goto end;
        }
    }

    if (padding < PAD_PKCS1 || padding >= PAD_MAX) {
        printf("[%s] %d :parameter padding(%d) is invalid.\n", __func__, __LINE__, padding);
        rv = FAILED;
        goto end;
    }

    *encrypted_data_len = 0;
    
    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0x2A;
    buf[2] = 0x00;
    buf[3] = 0x00;

    //set apdu body
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    switch(padding) {
        case PAD_PKCS1:
            if (SUCCEED != rsa_pkcs1_add_padding_type2(data, data_len, RSA_MOD_LEN, apduBuf + dataLen)) {
                rv = FAILED;
                goto end;
            }
            dataLen += RSA_MOD_LEN;
            break;
    }
    
    while (off < dataLen) {

        if ((dataLen - off) > APDU_BODY_MAX_SIZE) {
            len = APDU_BODY_MAX_SIZE;
            if (off == 0x00) {
                buf[2] = PKG_FIRST;//first block
            } else {
                buf[2] = PKG_MID;//mid block
            }
        } else {
            len = (dataLen - off);
            buf[2] = PKG_LAST;//last block
        }

        buf[4] = (unsigned char)len;
        memcpy(buf + APDU_HEADER_SIZE, apduBuf+off, len);
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

    if(apdu.rx_len) {
        if (encrypted_data) {
            memcpy(encrypted_data, apdu.rx, apdu.rx_len);
        }

        if (encrypted_data_len) {
            *encrypted_data_len = apdu.rx_len;
        }
    }

end:
    
    return rv;
}

int tms_rsa_decrypt(unsigned char *data, unsigned int data_len,
                    unsigned char *decrypted_data, unsigned int *decrypted_data_len, 
                    unsigned short rsaID, tms_pad_type padding)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;
    int hash_len;

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

    if (data_len != RSA_MOD_LEN) {
        printf("[%s] %d :parameter data_len(%d) should be %d bytes.\n", __func__, __LINE__, data_len, RSA_MOD_LEN);
        rv = FAILED;
        goto end;
    }

    if (decrypted_data_len == NULL) {
        printf("[%s] %d :parameter decrypted_data_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (padding < PAD_PKCS1 || padding >= PAD_MAX) {
        printf("[%s] %d :parameter padding(%d) is invalid.\n", __func__, __LINE__, padding);
        rv = FAILED;
        goto end;
    }

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0x2C;
    buf[2] = 0x00;
    buf[3] = 0x00;

    //set apdu body
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    memcpy(apduBuf + dataLen, data, data_len);
    dataLen += data_len;

    while (off < dataLen) {

        if ((dataLen - off) > APDU_BODY_MAX_SIZE) {
            len = APDU_BODY_MAX_SIZE;
            if (off == 0x00) {
                buf[2] = PKG_FIRST;//first block
            } else {
                buf[2] = PKG_MID;//mid block
            }
        } else {
            len = (dataLen - off);
            buf[2] = PKG_LAST;//last block
        }

        buf[4] = (unsigned char)len;
        memcpy(buf + APDU_HEADER_SIZE, apduBuf+off, len);
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
    
    if(apdu.rx_len) {
        switch(padding) {
            case PAD_PKCS1:
                if (SUCCEED != rsa_pkcs1_check_padding_type2(apduBuf, &len, RSA_MOD_LEN, apdu.rx)) {
                    rv = FAILED;
                    goto end;
                }
                break;
        }

        if (decrypted_data) {
            if (*decrypted_data_len < len) {
                printf("[%s] %d :decrypted_data_len(%d) is less than %d bytes. \n", __func__, __LINE__, *decrypted_data_len, len);
                rv = FAILED;
                goto end;
            }
            memcpy(decrypted_data, apduBuf, len);
        }

        if (decrypted_data_len) {
            *decrypted_data_len = len;
        }
    }
    else {
        if (decrypted_data_len) {
            *decrypted_data_len = 0;
        }
    }

end:
    
    return rv;
}

int tms_rsa_signature(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int *sign_data_len,
                    unsigned short rsaID, tms_pad_type padding)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;
    int hash_len;

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

    if (sign_data_len == NULL) {
        printf("[%s] %d :parameter sign_data_len is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (sign_data == NULL) {//返回实际响应长度
        *sign_data_len = RSA_MOD_LEN;
        goto end;
    }
    else {//判断输出Buffer是否足够大
        if (*sign_data_len < RSA_MOD_LEN) {
            printf("[%s] %d :sign_data_len(%d) is less than %d bytes. \n", __func__, __LINE__, *sign_data_len, RSA_MOD_LEN);
            rv = FAILED;
            goto end;
        }
    }

    if (padding < PAD_PKCS1 || padding >= PAD_MAX) {
        printf("[%s] %d :parameter padding(%d) is invalid.\n", __func__, __LINE__, padding);
        rv = FAILED;
        goto end;
    }

    *sign_data_len = 0;

    hash_len = sizeof(RecvBuf);
    rv = tms_sha256(data, data_len, RecvBuf, &hash_len);
    if(rv != SUCCEED)    {
        printf("[%s] %d :tms_sha256 failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0x2C;
    buf[2] = 0x00;
    buf[3] = 0x00;

    //set apdu body
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    switch(padding) {
        case PAD_PKCS1:
            if (SUCCEED != rsa_pkcs1_add_padding_type1(RecvBuf, hash_len, RSA_MOD_LEN, apduBuf + dataLen)) {
                rv = FAILED;
                goto end;
            }
            dataLen += RSA_MOD_LEN;
            break;
    }

    while (off < dataLen) {

        if ((dataLen - off) > APDU_BODY_MAX_SIZE) {
            len = APDU_BODY_MAX_SIZE;
            if (off == 0x00) {
                buf[2] = PKG_FIRST;//first block
            } else {
                buf[2] = PKG_MID;//mid block
            }
        } else {
            len = (dataLen - off);
            buf[2] = PKG_LAST;//last block
        }

        buf[4] = (unsigned char)len;
        memcpy(buf + APDU_HEADER_SIZE, apduBuf+off, len);
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

int tms_rsa_verify(unsigned char *data, unsigned int data_len,
                    unsigned char *sign_data, unsigned int sign_data_len,
                    unsigned short rsaID, tms_pad_type padding)
{
    int rv = SUCCEED;
    TRANSMIT_DATA apdu;
    unsigned char hash[SHA256_HASH_SIZE];
    unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
    int dataLen = 0, off = 0, len = 0;
    int hash_len;

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

    if (sign_data == NULL) {
        printf("[%s] %d :parameter sign_data is null.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

    if (sign_data_len != RSA_MOD_LEN) {
        printf("[%s] %d :parameter sign_data_len(%d) should be %d bytes.\n", __func__, __LINE__, sign_data_len, RSA_MOD_LEN);
        rv = FAILED;
        goto end;
    }

    if (padding < PAD_PKCS1 || padding >= PAD_MAX) {
        printf("[%s] %d :parameter padding(%d) is invalid.\n", __func__, __LINE__, padding);
        rv = FAILED;
        goto end;
    }

    hash_len = sizeof(hash);
    rv = tms_sha256(data, data_len, hash, &hash_len);
    if(rv != SUCCEED)    {
        printf("[%s] %d :tms_sha256 failed %04X.\n", __func__, __LINE__, rv);
        rv = FAILED;
        goto end;
    }

    //set apdu header
    buf[0] = 0x80;
    buf[1] = 0x2A;
    buf[2] = 0x00;
    buf[3] = 0x00;

    //set apdu body
    apduBuf[dataLen++] = (unsigned char)(rsaID >> 8);
    apduBuf[dataLen++] = (unsigned char)(rsaID & 0x00FF);
    memcpy(apduBuf + dataLen, sign_data, sign_data_len);
    dataLen += sign_data_len;

    while (off < dataLen) {

        if ((dataLen - off) > APDU_BODY_MAX_SIZE) {
            len = APDU_BODY_MAX_SIZE;
            if (off == 0x00) {
                buf[2] = PKG_FIRST;//first block
            } else {
                buf[2] = PKG_MID;//mid block
            }
        } else {
            len = (dataLen - off);
            buf[2] = PKG_LAST;//last block
        }

        buf[4] = (unsigned char)len;
        memcpy(buf + APDU_HEADER_SIZE, apduBuf+off, len);
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

    switch(padding) {
        case PAD_PKCS1:
            if (SUCCEED != rsa_pkcs1_add_padding_type1(hash, hash_len, RSA_MOD_LEN, apduBuf)) {
                rv = FAILED;
                goto end;
            }
            break;
    }

    if (0 != memcmp(apduBuf, apdu.rx, apdu.rx_len)) {
        printf("[%s] %d :rsa verify failed %04X.\n", __func__, __LINE__);
        rv = FAILED;
        goto end;
    }

end:
    
    return rv;
}





