#include <string.h>
#include <stdio.h>
#include "types.h"
#include "se.h"

void main()
{
    unsigned char version[100];
    int ver_len = sizeof(version);
    unsigned char rand[1024];
    unsigned char id[16];
    unsigned char data[1024];
    unsigned char data_bak[1024];
    int i,len;

    memset(version, 0, sizeof(version));
    
    if (SUCCEED != tms_initialize(NULL)) {
        printf("tms_initialize failed\n");
        goto err;
    }
    
    if (SUCCEED != tms_get_version(version, &ver_len)) {
        printf("tms_get_version failed\n");
        goto err;
    }

    printf("version = %s\n", version);

    if (SUCCEED != tms_get_rand(rand, sizeof(rand))) {
        printf("tms_get_rand failed\n");
        goto err;
    }

    for(i = 0; i < sizeof(data); i++) {
        data[i] = i;
    }

    if (SUCCEED != tms_write_data(data, 0, sizeof(data))) {
        printf("tms_write_data failed\n");
        goto err;
    }

    if (SUCCEED != tms_read_data(data_bak, 0, sizeof(data_bak))) {
        printf("tms_read_data failed\n");
        goto err;
    }

    if (memcmp(data, data_bak, sizeof(data))) {
        printf("tms_write_data and tms_read_data dismatch\n");
        goto err;
    }

err:
    if (SUCCEED != tms_finalize()) {
        printf("tms_finalize failed\n");
        return;
    }

}
