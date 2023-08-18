#include <string.h>
#include <stdio.h>
#include "types.h"
#include "se.h"
#include "common.h"

#define BUFF_MAX_SIZE   2048

void testSm2()
{
    unsigned char buf1[BUFF_MAX_SIZE];
    int len1;
    unsigned char buf2[BUFF_MAX_SIZE + 96];
    int len2;
    unsigned char buf3[BUFF_MAX_SIZE];
    int len3;
    unsigned char userid[16];
    

    unsigned char publicKey[SM2_PUBKEY_SIZE] = {
        0x0A,0x53,0x0A,0x5B,0xF0,0x1A,0x88,0xE9,
        0xDE,0x1D,0x38,0x5C,0x13,0x13,0xA1,0x23,
        0x4F,0xF2,0xE7,0xD6,0xC4,0x5D,0xA1,0xFE,
        0xE6,0xBF,0x19,0xC2,0xD8,0xF1,0xF2,0x2B,
        0x76,0xD3,0x6F,0xA6,0x85,0x76,0x1B,0x96,
        0xF8,0xA2,0x67,0x68,0xA7,0x86,0xEA,0x17,
        0xDD,0xF2,0x40,0x6A,0x08,0xFA,0x50,0x3F,
        0x0F,0x16,0x05,0x4B,0x78,0x8A,0xEA,0x24};
        
    unsigned char privateKey[SM2_PRIKEY_SIZE] = {
        0x2D,0x1B,0xA3,0xCE,0x19,0xC0,0x1F,0xAE,
        0x58,0x16,0x8E,0x69,0xCE,0x61,0x5F,0x71,
        0x6C,0x32,0xBA,0x43,0x44,0xAD,0xF2,0xC0,
        0x5E,0xD2,0x65,0x10,0x1B,0xDD,0xA2,0x40};
    
    if (SUCCEED != tms_initialize(NULL)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_sm2_genkeypair(0x0001)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_sm2_import_pubkey(0x002, publicKey, sizeof(publicKey))) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_sm2_import_prikey(0x002, privateKey, sizeof(privateKey))) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    len1 = sizeof(buf1);
    if (SUCCEED != tms_sm2_export_pubkey(0x001, buf1, &len1)) {
        goto err;
    }

    len1 = sizeof(buf1);
    if (SUCCEED != tms_sm2_export_pubkey(0x002, buf1, &len1)) {
        return;
    }

    if (len1 != SM2_PUBKEY_SIZE) {
        goto err;
    }

    if (memcmp(buf1, publicKey, len1)) {
        goto err;
    }

    len2 = sizeof(buf2);
    if (SUCCEED != tms_sm2_encrypt(buf1, sizeof(buf1), buf2, &len2, 0x0001)) {
        goto err;
    }
    len3 = sizeof(buf3);
    if (SUCCEED != tms_sm2_decrypt(buf2, len2, buf3, &len3, 0x0001)) {
        goto err;
    }
    if (memcmp(buf1, buf3, len1)) {
        goto err;
    }

    len2 = sizeof(buf2);
    if (SUCCEED != tms_sm2_encrypt(buf1, sizeof(buf1), buf2, &len2, 0x0002)) {
        goto err;
    }
    len3 = sizeof(buf3);
    if (SUCCEED != tms_sm2_decrypt(buf2, len2, buf3, &len3, 0x0002)) {
        goto err;
    }
    if (memcmp(buf1, buf3, len1)) {
        goto err;
    }

    len2 = sizeof(buf2);
    if (SUCCEED != tms_sm2_with_sm3_signature(buf1, sizeof(buf1), buf2, &len2, 0x0001, NULL)) {
        goto err;
    }
    if (SUCCEED != tms_sm2_with_sm3_verify(buf1, sizeof(buf1), buf2, len2, 0x0001, NULL)) {
        goto err;
    }


    len2 = sizeof(buf2);
    if (SUCCEED != tms_sm2_with_sm3_signature(buf1, sizeof(buf1), buf2, &len2, 0x0002, userid)) {
        goto err;
    }
    if (SUCCEED != tms_sm2_with_sm3_verify(buf1, sizeof(buf1), buf2, len2, 0x0002, userid)) {
        goto err;
    }

err:
    if (SUCCEED != tms_finalize()) {
        return;
    }

}

