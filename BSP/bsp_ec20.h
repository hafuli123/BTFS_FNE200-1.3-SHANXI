#ifndef __BSP_EC20_H
#define __BSP_EC20_H

#include "stdint.h"

void Chip_Ec20Init(void);
void Chip_Ec20Reset(void);
void Chip_Ec20Sleep(void);
void Chip_Ec20WakeUp(void);

uint8_t Chip_Ec20getVer(char* rbuf, uint8_t rLen);
uint8_t Chip_Ec20getIMEI(char* rbuf, uint8_t rLen);
uint8_t Chip_Ec20getIMSI(char* rbuf, uint8_t rLen);
uint8_t Chip_Ec20getICCID(char* rbuf, uint8_t rLen);
uint8_t Chip_Ec20getICCID(char* rbuf, uint8_t rLen);
uint8_t Chip_Ec20GetSignal(void);
uint8_t Chip_Ec20GetNetStatus(void);
uint8_t Chip_Ec20GetNetworkMode(void);
uint8_t Chip_Ec20PppdCall(char* ip);
uint8_t Chip_Ec20Mon(uint8_t *getCsq,uint8_t *getNetMode);

uint8_t Chip_Ec20TcpConnect(uint8_t socket_fd,char *domain,uint16_t port);
uint8_t Chip_Ec20TcpClose(uint8_t socket_fd);
uint8_t Chip_Ec20Tcp_Set_RecvCallBack(uint8_t socket_fd,void (*unpackCallBack)(uint8_t,uint8_t*,uint16_t len));
uint8_t Chip_Ec20TcpSta(uint8_t socket_fd);
uint8_t Chip_Ec20TcpSend(uint8_t socket_fd, uint8_t *buf, uint16_t len);
uint16_t Chip_Ec20TcpRecv(uint8_t socket_fd, uint8_t *buf, uint16_t len);

uint8_t Chip_Ec20Ftp_connect(char *hostname,uint16_t hostport,char *username,char *password);
uint8_t Chip_Ec20Ftp_disconnect(void);
uint32_t Chip_Ec20Ftp_GetFileSize(char *filePach,char* fileName);
uint32_t Chip_Ec20Ftp_ReadFile(char* fileName,uint32_t filePos,uint32_t getSize,uint8_t* buff);
uint8_t Chip_Ec20Http_Post_File_Start(char *url,char *username,char *password,char*fileName,uint32_t fileSize);
uint32_t Chip_Ec20Http_Post_File_Input(void* buff,uint32_t len);

#endif

