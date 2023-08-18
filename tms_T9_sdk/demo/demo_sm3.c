#include <string.h>
#include <stdio.h>
#include "types.h"
#include "se.h"
#include "common.h"

#define BUFF_MAX_SIZE1   8*1024
#define BUFF_MAX_SIZE2   8*1024-1

#define BUFF_MAX_SIZE   16*1024-1

void main()
{
    //0x11
    unsigned char Except1[] = {0xE4,0x2D,0x48,0xA7,0x97,0x0B,0x35,0xD7,0x2A,0xEA,0xA4,0x7A,0x44,0x8A,0x9D,0x8A,0xC5,0xC4,0x7B,0xC1,0x19,0x3B,0x1D,0xC0,0x40,0x9C,0x6C,0x30,0x14,0xF2,0xE8,0x0F};
    //0x1122334455667788
    unsigned char Except2[] = {0x62,0x51,0x9C,0x9B,0xB2,0xDC,0xDE,0x13,0xCF,0x6F,0x96,0x25,0x2F,0xA6,0x9A,0x6A,0xD4,0xF3,0xEC,0x43,0xA8,0x73,0x58,0xB5,0xE3,0x17,0x84,0xAA,0x5F,0x4E,0x43,0xC8};
    //16K-1 0x11
    unsigned char Except3[] = {0x48,0x38,0xCA,0xC2,0x28,0x47,0xE6,0xE0,0x26,0xB0,0x2E,0xCF,0x65,0x32,0x25,0xB3,0xA2,0xEB,0x3A,0x97,0x6D,0x1E,0x8C,0x48,0x6B,0x9C,0x55,0x0C,0xBB,0x1F,0xE7,0x92};

    unsigned char buf1[BUFF_MAX_SIZE];
    unsigned char hash[32];
    int len = sizeof(hash);

    
    if (SUCCEED != tms_initialize(NULL)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    /*tms_sm3 test*/

    buf1[0] = 0x11;
    len = sizeof(hash);
    if (SUCCEED != tms_sm3(buf1, 1, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(hash, Except1, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    buf1[0] = 0x11;
    buf1[1] = 0x22;
    buf1[2] = 0x33;
    buf1[3] = 0x44;
    buf1[4] = 0x55;
    buf1[5] = 0x66;
    buf1[6] = 0x77;
    buf1[7] = 0x88;
    len = sizeof(hash);
    if (SUCCEED != tms_sm3(buf1, 8, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(hash, Except2, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    memset(buf1, 0x11, BUFF_MAX_SIZE);
    len = sizeof(hash);
    if (SUCCEED != tms_sm3(buf1, BUFF_MAX_SIZE, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(hash, Except3, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }


    /*tms_sm3_init/tms_sm3_update/tms_sm3_final test*/

    buf1[0] = 0x11;
    len = sizeof(hash);
    if (SUCCEED != tms_sm3_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_final(buf1, 1, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(hash, Except1, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    buf1[0] = 0x11;
    buf1[1] = 0x22;
    buf1[2] = 0x33;
    buf1[3] = 0x44;
    buf1[4] = 0x55;
    buf1[5] = 0x66;
    buf1[6] = 0x77;
    buf1[7] = 0x88;
    len = sizeof(hash);
    if (SUCCEED != tms_sm3_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_final(buf1, 8, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(hash, Except2, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    buf1[0] = 0x11;
    buf1[1] = 0x22;
    buf1[2] = 0x33;
    buf1[3] = 0x44;
    buf1[4] = 0x55;
    buf1[5] = 0x66;
    buf1[6] = 0x77;
    buf1[7] = 0x88;
    len = sizeof(hash);
    if (SUCCEED != tms_sm3_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_update(buf1, 4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_final(buf1 + 4, 4, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(hash, Except2, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    memset(hash, 0, sizeof(hash));
    memset(buf1, 0x11, BUFF_MAX_SIZE);
    len = sizeof(hash);
    if (SUCCEED != tms_sm3_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_update(buf1, BUFF_MAX_SIZE1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_final(buf1 + BUFF_MAX_SIZE1, BUFF_MAX_SIZE2, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(hash, Except3, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    memset(hash, 0, sizeof(hash));
    memset(buf1, 0x11, BUFF_MAX_SIZE);
    len = sizeof(hash);
    if (SUCCEED != tms_sm3_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sm3_final(buf1, BUFF_MAX_SIZE, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(hash, Except3, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }


    
err:
    if (SUCCEED != tms_finalize()) {
        printf("%d error.\n", __LINE__);
        return;
    }

}
