
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include "driver.h"

#define TMC_SEND_MAX 261
#define TMC_RECE_MAX 265
#define TMC_I2C_MAX 4

static const char *default_i2c_Device = "/dev/i2c-1";

static uint8_t I2C_SLAVE_ADDR = 0x50; /* I2C slave address */
static int g_I2C_Fd = 0;

static TMS_DRIVER_OPS i2c_ops;
static TMS_DRIVER i2c_driver = {
    "I2C",
    &i2c_ops
};

static void print(unsigned char *s, unsigned char *buff, unsigned int len)
{
#ifdef APDU_PRINT
    int i;
    printf("%s [Len:%d]", s, len);
    for (i = 0; i < len; i++) {
        if (i % 8 == 0) {
            printf("\n\t");
        }
        printf("0x%02X ", buff[i]);
    }
    printf("\n");
#else
    (void *)s;
    (void *)buff;
    (void *)len;
#endif
}

static void pabort(const char *s)
{
    perror(s);
    abort();
}

static void delayT(int ns)
{
    struct timespec ts, ts1;

    ts.tv_nsec = ns;
    ts.tv_sec = 0;
    nanosleep(&ts, &ts1);
}

static unsigned char tmc_i2c_crc(unsigned char *InBuf, int len)
{
    unsigned char ret = 0;
    int i;

    for (i=0;i<len;i++) {
        ret = ret ^ InBuf[i];
    }
    return ret;
}


static int I2C_Write(uint8_t *TxBuf, int len)
{
    int ret;
    int fd = g_I2C_Fd;
    struct i2c_rdwr_ioctl_data work_queue;

    work_queue.nmsgs = 1;
    work_queue.msgs = (struct i2c_msg *)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
    if (!work_queue.msgs) {
        printf("[%s] work_queue.msgs alloc error\n", __func__);
        return -1;
    }

    (work_queue.msgs[0]).addr = I2C_SLAVE_ADDR;
    (work_queue.msgs[0]).flags = 0;//write
    (work_queue.msgs[0]).len = (unsigned short)len;
    (work_queue.msgs[0]).buf = TxBuf;

    ret = ioctl(fd, I2C_RDWR, (unsigned long)&work_queue);

    if (work_queue.msgs != NULL) {
        free(work_queue.msgs);
        work_queue.msgs = NULL;
    }

    return (ret < 0) ? -1 : 0;
}

static int I2C_Read(uint8_t *RxBuf, int len)
{
    int ret;
    int fd = g_I2C_Fd;
    struct i2c_rdwr_ioctl_data work_queue;

    work_queue.nmsgs = 1;
    work_queue.msgs = (struct i2c_msg *)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
    if (!work_queue.msgs) {
        printf("[%s] work_queue.msgs alloc error\n", __func__);
        return -1;
    }
    (work_queue.msgs[0]).addr = I2C_SLAVE_ADDR;
    (work_queue.msgs[0]).flags = 1;//read
    (work_queue.msgs[0]).len = (unsigned short)len;
    (work_queue.msgs[0]).buf = RxBuf;

    ret = ioctl(fd, I2C_RDWR, (unsigned long)&work_queue);

    if (work_queue.msgs != NULL) {
        free(work_queue.msgs);
        work_queue.msgs = NULL;
    }

    return (ret < 0) ? -1 : 0;
}

static int tmc_i2c_send(uint8_t *TxBuf, int len)
{
    int i,ret;
    
    if (!g_I2C_Fd) {
        printf("[%s] device is not exist.\n", __func__);
        return -1;
    }

    if (len > TMC_SEND_MAX) {
        printf("[%s] sending data is too long.\n", __func__);
        return -1;
    }

    int outlen = len + TMC_I2C_MAX + 1;
    uint8_t outBuf[outlen];

    memset(outBuf,0,outlen);

    outBuf[0] = (uint8_t)(I2C_SLAVE_ADDR << 1);
    outBuf[1] = 0xAA;
    outBuf[2] = len >> 8;
    outBuf[3] = len >> 0;
    memcpy(outBuf + 4, TxBuf, len);
    outBuf[outlen - 1] = tmc_i2c_crc(TxBuf, len);

    print("Send", outBuf, outlen);
    
    return I2C_Write(outBuf + 1, outlen - 1);
}

static int tmc_i2c_receive(uint8_t *RxBuf, int* len, int max_wait_time)
{
    int findAA = 0;
    int count = DELAY_1_SEC * max_wait_time;
    int inlen;
    uint8_t crc ;
    uint8_t inBuf[TMC_RECE_MAX];

    if (!g_I2C_Fd) {
        printf("[%s] device is not exist.\n", __func__);
        return -1;
    }

    memset(inBuf,0x00,TMC_RECE_MAX);

    delayT(200000);//200us

    while(count--)
    {
        I2C_Read(inBuf + 1, 3);
        if (inBuf[1] == 0xAA) {
            findAA = 1;
            break;
        }
        delayT(30000);//30us
    }

    if (findAA)
    {
        inlen = (inBuf[2] << 8) | (inBuf[3]);
        
        I2C_Read(inBuf + 4, inlen + 1);

        crc = tmc_i2c_crc(inBuf + 4, inlen);
        if (crc != inBuf[inlen + 4]) {
            printf("[%s] crc check failed.\n", __func__);
            return -1;
        }

        if (RxBuf) {
            memcpy(RxBuf, inBuf + 4, inlen);
        }
        if (len) {
            *len = inlen;
        }

        inBuf[0] = (uint8_t)((I2C_SLAVE_ADDR << 1) | 0x01);
        print("Recv", inBuf, inlen + 5);
        delayT(100000); //receive to send, delay 100us
        return 0;
    }
    else {
        printf("[%s] not receive tag 0xAA from I2C Slave.\n", __func__);
        return -1;
    }
}

int i2c_init(char *device)
{
    int fd;
    int ret = 0;
    char *pDev;

    if (g_I2C_Fd) {
        return 1;
    }

    if (device == NULL) {
        pDev = (char *)default_i2c_Device;
    }
    else {
        pDev = device;
    }

    fd = open(pDev, O_RDWR);
    if (fd < 0) {
        printf("[%s] open the device: %s failed.\n", __func__, pDev);
        abort();
    }
    else {
        if (flock(fd, LOCK_EX | LOCK_NB)) {
            printf("the device was locked.\n");
            sleep(3);
            while(flock(fd, LOCK_EX | LOCK_NB) == 0) {
                printf("wait for unlock.\n");
                sleep(3);
            }
        }
    }

    g_I2C_Fd = fd;

    ret = ioctl(fd,I2C_TENBIT,0);
    if (ret < 0) {
        pabort("set i2c bit error.\n");
    }

    ret = ioctl(fd, I2C_SLAVE, I2C_SLAVE_ADDR);
    if (ret < 0) {
        pabort("set i2c address error.\n");
    }

    printf("I2C Open Success\n");

    return ret;
}


int i2c_finish()
{
    if (!g_I2C_Fd) {
        printf("[%s] device is not exist.\n", __func__);
        return 1;
    }

    close(g_I2C_Fd);
    g_I2C_Fd = 0;

    return SUCCEED;
}

int i2c_reset()
{
    return SUCCEED;
}

int i2c_wakeup(void)
{
    unsigned char wkup = 0x55;

    if (!g_I2C_Fd) {
        printf("[%s] device is not exist.\n", __func__);
        return -1;
    }

    print("I2C Wakeup", &wkup, 1);

    I2C_Write(&wkup, 1);
    
    return 0;
}

int i2c_sleep(void)
{
    return SUCCEED;
}

int i2c_transmit(TRANSMIT_DATA *pData)
{
    int i,err,RETRY_COUNT=3;

    for (i = 0; i < RETRY_COUNT; i++) {
        err = tmc_i2c_send(pData->tx, pData->tx_len);        
        if (err) {
            continue;
        }
        
        err = tmc_i2c_receive(pData->rx, (int*)&pData->rx_len, pData->max_wait_time);
        if (err) {
            continue;
        }
        
        return SUCCEED;
    }
    return FAILED;
}

TMS_DRIVER * tms_get_i2c_driver(void)
{
    i2c_ops.init = i2c_init;
    i2c_ops.finish = i2c_finish;
    i2c_ops.reset = i2c_reset;
    i2c_ops.wakeup = i2c_wakeup;
    i2c_ops.sleep = i2c_sleep;
    i2c_ops.transmit = i2c_transmit;

    return &i2c_driver;
}

