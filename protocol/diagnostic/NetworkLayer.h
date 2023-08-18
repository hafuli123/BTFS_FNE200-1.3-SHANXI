#ifndef _NETWORKLAYER_H
#define _NETWORKLAYER_H

#include "NetworkLayerTypeDefines.h"

#define MAX_BUFF_NUMBER													3
#define MAX_INDICATION_NUMBER										3
#define MAX_DOWNLOADING_BUF											100

typedef struct
{
 NetworkNotification IndicationList[MAX_INDICATION_NUMBER];		//指示列表
 uint8_t IndicationInIndex;																		//指示存入位置
 uint8_t IndicationOutIndex;																	//指示读取位置
	//接收CAN数据环形存入取出
 NetworkFrame RxFrameBuff[MAX_BUFF_NUMBER];										//rx帧
 uint8_t RxInIndex;																						//rx存入位置
 uint8_t RxOutIndex;																					//rx读取位置
	//发送CAN数据唤醒存入取出
 NetworkFrame TxFrameBuff[MAX_BUFF_NUMBER];										//tx帧
 uint8_t TxInIndex;																						//tx存入位置
 uint8_t TxOutIndex;																					//tx读取位置
 //网络控制
 N_Result m_N_Result;																				//网络结果
 NWL_Status m_NetworkStatus;																//网络状态，空闲，接收，传输，等待
 DuplexMode m_DuplexMode;																		//双工模式 全双工（发送时可切换到接收模式） 半双工
 TimePeriodParam m_TimePeriod;															//时间段控制
 TransmissionStep m_TxStep;																	//发送状态机
 RecivingStep m_RxStep;																			//接收状态机
 AddressFormat m_AddressFormat;															//地址格式
 uint8_t* CFDataPionter;																		//发送数据连续帧指针
 CommuParam TxParam;																				//发送参数
 CommuParam RxParam;																				//接收参数
 uint8_t NetworkDataBufRx[MAX_DOWNLOADING_BUF];							//接收数据的缓存
 uint8_t FrameFillData;																			//CAN帧填充数据
 //CAN数据
 uint32_t m_PyhReqID;																				//物理ID
 uint32_t m_FunReqID;																				//功能ID
 uint32_t m_ResponseID;																			//响应ID
 uint8_t RxDataBuff[7];																			//RX数据缓存
 //定时器
 DiagTimer SendTimer;																				//发送定时器
 DiagTimer ReciveTimer;																			//接收定时器
 DiagTimer CFRequestTimer;																	//连续帧请求定时器
 SendCANFun NetworkSend;																		//CAN发送函数
}TP_T;

//初始化参数
extern void NetworkLayer_InitParam(TP_T* pTp,uint32_t PyhReqID,uint32_t FunReqID, uint32_t ResponseID,SendCANFun sendFun,GetMsTickCountFun GetMsTickCntFun);
//参数设置
extern void NetworkLayer_SetParam(TP_T* pTp,uint8_t TimeAs, uint8_t TimeBs, uint8_t TimeCr, uint8_t TimeAr, uint8_t TimeBr, uint8_t TimeCs, 	uint8_t Bs, uint8_t m_STmin, DuplexMode nDuplex ,  MType Mtype , uint8_t N_SA , uint8_t N_TA , N_TAtype N_TAtype , uint8_t N_AE , uint8_t FillData);
//网络层处理
extern void NetworkLayer_Proc(TP_T* pTp);
//数据发送
extern void N_USData_request(TP_T* pTp,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, uint8_t* MessageData, uint16_t Length);
//数据接收
extern void NetworkLayer_RxFrame(TP_T* pTp,uint32_t ID,uint8_t* data,uint8_t IDE,uint8_t DLC,uint8_t RTR);
//指示是否为空
extern bool IsIndicationListEmpty(TP_T* pTp);
//拉取指示
extern NetworkNotification PullIndication(TP_T* pTp);
//改为参数请求
extern void N_ChangeParameter_request(TP_T* pTp,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, Parameter Parameter, uint8_t Parameter_Value);
extern void DiagTimer_Set(DiagTimer *STimer, uint32_t TimeLength);
extern bool DiagTimer_HasExpired(DiagTimer *STimer);
#endif
