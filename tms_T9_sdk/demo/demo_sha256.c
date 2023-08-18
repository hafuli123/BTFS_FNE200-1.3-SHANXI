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
    unsigned char Except1[] = {0x4A,0x64,0xA1,0x07,0xF0,0xCB,0x32,0x53,0x6E,0x5B,0xCE,0x6C,0x98,0xC3,0x93,0xDB,0x21,0xCC,0xA7,0xF4,0xEA,0x18,0x7B,0xA8,0xC4,0xDC,0xA8,0xB5,0x1D,0x4E,0xA8,0x0A};
    //0x1122334455667788
    unsigned char Except2[] = {0x1D,0xCE,0x66,0x04,0x59,0x1E,0xFB,0x43,0x9D,0x5E,0x87,0x41,0x8A,0x1D,0x00,0xDB,0xFD,0x01,0x43,0x27,0xD8,0xC4,0xDE,0xA8,0x62,0x81,0x57,0x14,0xB7,0x6A,0xE9,0xA5};
    //16K-1 0x11
    unsigned char Except3[] = {0x59,0x35,0x7A,0xB1,0x12,0xA6,0xA0,0x3B,0x14,0x98,0x53,0xE3,0x79,0xB1,0xA4,0xAA,0x59,0x42,0x24,0xC6,0xB1,0x26,0xB7,0x02,0xCE,0x05,0xD6,0xF4,0x4F,0xE1,0xB2,0x49};

    unsigned char buf1[BUFF_MAX_SIZE];
    unsigned char hash[32];
    int len = sizeof(hash);

    
    if (SUCCEED != tms_initialize(NULL)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    /*tms_sha256 test*/

    buf1[0] = 0x11;
    len = sizeof(hash);
    if (SUCCEED != tms_sha256(buf1, 1, hash, &len)) {
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
    if (SUCCEED != tms_sha256(buf1, 8, hash, &len)) {
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
    if (SUCCEED != tms_sha256(buf1, BUFF_MAX_SIZE, hash, &len)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(hash, Except3, len) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }


    /*tms_sha256_init/tms_sha256_update/tms_sha256_final test*/

    buf1[0] = 0x11;
    len = sizeof(hash);
    if (SUCCEED != tms_sha256_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_final(buf1, 1, hash, &len)) {
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
    if (SUCCEED != tms_sha256_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_update(buf1, 4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_final(buf1 + 4, 4, hash, &len)) {
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
    if (SUCCEED != tms_sha256_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_final(buf1, 8, hash, &len)) {
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
    if (SUCCEED != tms_sha256_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_update(buf1, BUFF_MAX_SIZE1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_final(buf1 + BUFF_MAX_SIZE1, BUFF_MAX_SIZE2, hash, &len)) {
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
    if (SUCCEED != tms_sha256_init()) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (SUCCEED != tms_sha256_final(buf1, BUFF_MAX_SIZE, hash, &len)) {
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

