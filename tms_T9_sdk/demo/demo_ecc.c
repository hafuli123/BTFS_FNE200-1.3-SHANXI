#include <string.h>
#include <stdio.h>
#include "types.h"
#include "se.h"
#include "common.h"

#define BUFF_MAX_SIZE   256
#define PUBKEY_SIZE   64
#define PRIKEY_SIZE   32

void main()
{
    unsigned char buf[BUFF_MAX_SIZE -11];
    int len;
    unsigned char buf1[BUFF_MAX_SIZE];
    int len1;
    unsigned char buf2[BUFF_MAX_SIZE];
    int len2;
    unsigned char buf3[BUFF_MAX_SIZE];
    int len3;
    
    unsigned char PublicKey[PUBKEY_SIZE] = {
0xB2,0x80,0xB2,0x32,0xA4,0xF8,0xD9,0x78,
0xD0,0x2A,0x6E,0xC4,0x30,0xCD,0x8C,0x97,
0xF5,0x34,0x09,0xF4,0x36,0xA4,0x83,0x1F,
0x98,0x3F,0x27,0x7E,0xCA,0x0D,0x0A,0x11,
0x28,0x18,0x91,0x7B,0xA3,0x06,0xE5,0x1E,
0x4A,0xF5,0x72,0x41,0xD3,0x4A,0x4A,0x41,
0x6A,0x8E,0x94,0xDF,0x45,0x29,0x35,0xF2,
0x03,0x3A,0xFB,0x6F,0x16,0xB1,0x37,0x52};

    unsigned char PrivateKey[PRIKEY_SIZE] = {
0x50,0xAC,0xF0,0x1D,0xC9,0x0B,0x58,0x7A,
0xDF,0xCE,0xE1,0x0D,0xEF,0xEB,0x5C,0x6E,
0x67,0xA7,0xB6,0x85,0x57,0x30,0x41,0x71,
0x51,0x37,0x16,0xD5,0xA4,0x3A,0x17,0x4E};

    if (SUCCEED != tms_initialize(NULL)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_ecc_genkeypair(0x0001, NIST_P256)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_ecc_import_pubkey(0x002, PublicKey, sizeof(PublicKey), NIST_P256)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_ecc_import_prikey(0x002, PrivateKey, sizeof(PrivateKey), NIST_P256)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    len1 = sizeof(buf1);
    if (SUCCEED != tms_ecc_export_pubkey(0x001, buf1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    len1 = sizeof(buf1);
    if (SUCCEED != tms_ecc_export_pubkey(0x002, buf1, &len1)) {
        printf("%d error.\n", __LINE__);
        return;
    }

    if (len1 != PUBKEY_SIZE) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(buf1, PublicKey, len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    len = sizeof(buf);
    if (SUCCEED != tms_get_rand(buf, 32)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(buf2);
    if (SUCCEED != tms_ecc_signature(buf, 32, buf2, &len2, 0x0001)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_ecc_verify(buf, 32, buf2, len2, 0x0001)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    len = sizeof(buf);
    if (SUCCEED != tms_get_rand(buf, 32)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(buf2);
    if (SUCCEED != tms_ecc_signature(buf, 32, buf2, &len2, 0x0002)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_ecc_verify(buf, 32, buf2, len2, 0x0002)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

err:
    if (SUCCEED != tms_finalize()) {
        printf("%d error.\n", __LINE__);
        return;
    }

}

