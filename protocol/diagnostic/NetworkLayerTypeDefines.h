#ifndef __NMLAYER_TYPE_DEFINES_H
#define __NMLAYER_TYPE_DEFINES_H

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

typedef unsigned          char byte;

typedef enum{
	N_OK,											//sender and receiver
	N_TIMEOUT_A,							//sender and receiver
	N_TIMEOUT_Bs,							//sender only
	N_TIMEOUT_Cr,							//receiver only
	N_WRONG_SN,								//receiver only
	N_INVALID_FS,							//sneder only
	N_UNEXP_PDU,							//receiver only
	N_WFT_OVRN,								//
	N_BUFFER_OVFLW,						//sender only
	N_ERROR,									//sender and receiver
}N_Result;

typedef enum{
	DIAGNOSTIC,
	REMOTE_DIAGNOSTICS,
}MType;

typedef enum{
	PHYSICAL,
	FUNCTIONAL,
}N_TAtype;

typedef enum{
	STmin,
	BS,
}Parameter;

typedef enum{
	N_CHANGE_OK,			//sender and receiver
	N_RX_ON,				//receiver only
	N_WRONG_PARAMETER,	//sender and receiver
	N_WRONG_VALUE,		//sender and receiver
}Result_ChangeParameter;

typedef enum{
	CTS,	 						//continue to send
	WT,								//wait
	OVFLW,						//overflow
}FS_Type;

typedef enum
{
	NWL_IDLE,						//空闲
	NWL_TRANSMITTING,		//传输
	NWL_RECIVING,				//接收
	NWL_WAIT,						//等待
}NWL_Status;

typedef enum{
	SF = 0,							//single frame
	FF = 1,							//first frame
	CF = 2,							//consecutive frame
	FC = 3,							//flow control
}N_PCIType;

typedef enum{
	TX_IDLE,
	TX_WAIT_FF_CONF,
	TX_WAIT_FC,
	TX_WAIT_CF_REQ,
	TX_WAIT_CF_CONF,
}TransmissionStep;

typedef enum{
	RX_IDLE,
	RX_WAIT_FC_REQ,
	RX_WAIT_FC_CONF,
	RX_WAIT_CF,
}RecivingStep;

typedef enum{
	HALF_DUPLEX,
	FULL_DUPLEX,
}DuplexMode;

typedef struct{
	uint16_t N_As;
	uint16_t N_Ar;
	uint16_t N_Bs;
	uint16_t N_Br;
	uint16_t N_Cs;
	uint16_t N_Cr;
	bool FF_ConfirmedOnSender;										//when transmission
	bool FC_RecivedOnSender;											//when transmission
	bool CF_RequestedOnSender;										//when transmission
	bool CF_ConfirmedOnSender;										//when transmission
	bool FC_RxBeforeFFOnSender;										//when transmission
	bool FC_RequestedOnReciver;										//when reciving
	bool FC_ConfirmedOnReciver;										//when reciving
	bool CF_RecivedOnReciver;											//when reciving
}TimePeriodParam;

typedef struct{
	MType Mtype;																	//信息类型
	uint8_t N_SA;																	//网络源地址
	uint8_t N_TA;																	//网络目标地址
	N_TAtype N_TAtype;														//网络目标地址类型
	uint8_t N_AE;																	//网络地址扩展
}AddressFormat;

typedef union{
	struct{
		MType Mtype;
		N_TAtype N_TAtype;
		uint8_t N_SA;
		uint8_t N_TA;
		uint8_t N_AE;
		uint32_t ID;
		uint8_t DLC;
		uint8_t RTR;
		uint8_t IDE;
		bool valid;
		uint8_t data7;
		uint8_t data6;
		uint8_t data5;
		uint8_t data4;
		uint8_t data3;
		uint8_t STmin;															//STmin ,data2
		uint8_t FF_DL_LOW;													//BS ,FF_DL_LOW ,data1
		uint8_t SF_DL:4;														//SF_DL ,FF_DL_HIGH ,TxParam.SN ,FS
		N_PCIType N_PciType:4;											//网络协议控制信息类型
	}N_PDU;																				//PDU
	struct{
		MType Mtype;																//信息类型
		N_TAtype N_TAtype;													//网络目标地址类型
		uint8_t N_SA;																//网络源地址
		uint8_t N_TA;																//网络目标地址
		uint8_t N_AE;																//网络地址扩展
		uint32_t ID;																//CAN ID
		uint8_t DLC;																//数据长度
		uint8_t RTR;																//是否远程帧
		uint8_t IDE;																//是否扩展帧
		bool valid;
		uint8_t data7;
		uint8_t data6;
		uint8_t data5;
		uint8_t data4;
		uint8_t data3;
		uint8_t data2;
		uint8_t data1;
		uint8_t data0;
	}CanData;																				//CAN
}NetworkFrame;

typedef struct{
	FS_Type FS_Type;																//数据帧状态类型
	uint8_t BlockSize;															//总数据包数
	uint8_t CompletedNumberInBlock;									//以处理数据包数
	uint8_t STmin;																	//传输间隔
	uint8_t SN;																			//流水号
	uint16_t TotalDataNumber;												//总数据数
	uint16_t CompletedDataNumber;										//已处理数据数
	uint16_t BuffSize;															//缓冲区大小
}CommuParam;

typedef enum{
	CONFIRM,																				//确认应答
	FF_INDICATION,																	//首帧接收
	INDICATION,																			//接收指示
}NetWorkNotificationType;

typedef struct{
	NetWorkNotificationType NotificationType;				//网络通知类型
	MType Mtype;																		//信息类型
	uint8_t N_SA;																		//网络源地址
	uint8_t N_TA;																		//网络目标地址
	N_TAtype N_TAtype;															//网络目标地址类型
	uint8_t N_AE;																		//网络地址扩展
	uint8_t *MessageData;														//网络数据帧
	uint16_t length;																//网络数据长度
	N_Result N_Resut;																//结果
	bool valid;
}NetworkNotification;

typedef uint8_t (*SendCANFun)(uint32_t ID, uint8_t *array, uint8_t length, uint8_t priority, uint8_t rtr, uint8_t ide);
typedef uint32_t (*GetMsTickCountFun)(void);

typedef struct{
	uint32_t TimerCounter;
	bool valid;
}DiagTimer;

#endif /* __NMLAYER_TYPE_DEFINES_H */
