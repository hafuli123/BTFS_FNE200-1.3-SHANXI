#include <string.h>
#include <stdio.h>
#include "types.h"
#include "se.h"
#include "common.h"

#define BUFF_MAX_SIZE   10240

void main()
{
    unsigned char in1[BUFF_MAX_SIZE];
    int len1;
    unsigned char out1[BUFF_MAX_SIZE];
    int len2;
    unsigned char in2[BUFF_MAX_SIZE];
    int len3;
    unsigned char out2[BUFF_MAX_SIZE];
    int len4;

    unsigned char sm4Key[16] = {
        0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
        0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    unsigned char iv[16] = {
        0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
        0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
        
            
    if (SUCCEED != tms_initialize(NULL)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_sm4_genkey(0x0001)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (SUCCEED != tms_sm4_import_key(sm4Key, sizeof(sm4Key), 0x0002)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    memset(in1, 0x11, sizeof(in1));
    memset(in2, 0x22, sizeof(in2));

   //sm4 ecb encrypt
    if (SUCCEED != tms_sm4_init(0x0001, NULL, 0, 0)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len1 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(in1, sizeof(in1), out1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(in2, sizeof(in2), out2, &len2)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    //sm4 ecb decrypt
    if (SUCCEED != tms_sm4_init(0x0001, NULL, 0, 1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len3 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(out1, len1, out1, &len3)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len4 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(out2, len2, out2, &len4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(out1, in1, sizeof(in1)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(out2, in2, sizeof(in2)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

   //sm4 cbc encrypt
    if (SUCCEED != tms_sm4_init(0x0001, iv, sizeof(iv), 0)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len1 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(in1, sizeof(in1), out1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(in2, sizeof(in2), out2, &len2)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    //sm4 cbc decrypt
    if (SUCCEED != tms_sm4_init(0x0001, iv, sizeof(iv), 1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len3 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(out1, len1, out1, &len3)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len4 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(out2, len2, out2, &len4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(out1, in1, sizeof(in1)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(out2, in2, sizeof(in2)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }


   //sm4 ecb encrypt
    if (SUCCEED != tms_sm4_init(0x0002, NULL, 0, 0)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len1 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(in1, sizeof(in1), out1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(in2, sizeof(in2), out2, &len2)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    //sm4 ecb decrypt
    if (SUCCEED != tms_sm4_init(0x0002, NULL, 0, 1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len3 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(out1, len1, out1, &len3)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len4 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(out2, len2, out2, &len4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(out1, in1, sizeof(in1)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(out2, in2, sizeof(in2)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

   //sm4 cbc encrypt
    if (SUCCEED != tms_sm4_init(0x0002, iv, sizeof(iv), 0)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len1 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(in1, sizeof(in1), out1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(in2, sizeof(in2), out2, &len2)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    //sm4 cbc decrypt
    if (SUCCEED != tms_sm4_init(0x0002, iv, sizeof(iv), 1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len3 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(out1, len1, out1, &len3)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len4 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(out2, len2, out2, &len4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(out1, in1, sizeof(in1)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(out2, in2, sizeof(in2)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }




   //sm4 ecb encrypt
    if (SUCCEED != tms_sm4_init_ext(sm4Key, sizeof(sm4Key), NULL, 0, 0)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len1 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(in1, sizeof(in1), out1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(in2, sizeof(in2), out2, &len2)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    //sm4 ecb decrypt
    if (SUCCEED != tms_sm4_init_ext(sm4Key, sizeof(sm4Key), NULL, 0, 1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len3 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(out1, len1, out1, &len3)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len4 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(out2, len2, out2, &len4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(out1, in1, sizeof(in1)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(out2, in2, sizeof(in2)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }

   //sm4 cbc encrypt
    if (SUCCEED != tms_sm4_init_ext(sm4Key, sizeof(sm4Key), iv, sizeof(iv), 0)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len1 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(in1, sizeof(in1), out1, &len1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len2 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(in2, sizeof(in2), out2, &len2)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    //sm4 cbc decrypt
    if (SUCCEED != tms_sm4_init_ext(sm4Key, sizeof(sm4Key), iv, sizeof(iv), 1)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len3 = sizeof(out1);
    if (SUCCEED != tms_sm4_update(out1, len1, out1, &len3)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    len4 = sizeof(out2);
    if (SUCCEED != tms_sm4_final(out2, len2, out2, &len4)) {
        printf("%d error.\n", __LINE__);
        goto err;
    }

    if (memcmp(out1, in1, sizeof(in1)) != 0)
    {
        printf("%d error.\n", __LINE__);
        goto err;
    }
    if (memcmp(out2, in2, sizeof(in2)) != 0)
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

