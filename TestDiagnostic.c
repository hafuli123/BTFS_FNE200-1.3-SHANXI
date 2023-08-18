
#include "NetworkLayer.h"

#define SERVICE_NUMBER 				3

typedef enum{
	LEVEL_ZERO = 7,
	LEVEL_ONE = 1,
	LEVEL_TWO = 2,
	LEVEL_THREE = 4,
	LEVEL_FOUR = 8,
	LEVEL_UNSUPPORT = 0,
}SecurityLevel;

typedef struct{
	bool support;
	ServiceName serviceName;
	uint8_t PHYDefaultSession_Security:4;							//security suppport in default session physical address
	uint8_t PHYProgramSeesion_Security:4;							//security suppport in program session physical address
	uint8_t PHYExtendedSession_Security:4;						//security suppport in extened session physical address
	uint8_t FUNDefaultSession_Security:4;							//security suppport in default session function address
	uint8_t FUNProgramSeesion_Security:4;							//security suppport in program session function address
	uint8_t FUNExtendedSession_Security:4;						//security suppport in extened session function address
	ServiceHandler serviceHandle;
}SessionService;

static uint8_t CurrentService;																														//��ǰ����

SessionService ServiceList[SERVICE_NUMBER] = {
	{FALSE, SESSION_CONTROL,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			Service10Handle},//0X10
	{FALSE, CLEAR_DTC_INFO,				LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			Service14Handle},//0X14
	{FALSE, READ_DTC_INFO,				LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service19Handle},//0X19
};


/* 
���ܣ���ʼ��
������	pTp					�ṹָ��
			PyhReqID		����ID
			FunReqID		����ID
			ResponseID	��ӦID
			sendFun			���ͺ���
*/
void Diagnostic_Init(TP_T* pTp,uint32_t PyhReqID,uint32_t FunReqID, uint32_t ResponseID,SendCANFun sendFun)
{
	NetworkLayer_InitParam(pTp, PyhReqID, FunReqID, ResponseID, sendFun);
}

/*
���ܣ���������ö�ʱ���� 
������	TimeAs��N_As				CAN ���Ĵ���ʱ�� sender->receiver
			TimeBs��N_Bs				(N_Ar + N_Br) ��ʱ���
			TimeCr��N_Cr				��������֡��ʱ��
			TimeAr��N_Ar				CAN ���Ĵ���ʱ�� receiver->sender
			TimeBr��N_Br				������һ������֡��ʱ��
			TimeCs��N_Cs				��������֡��ʱ��
			DuplexMode					ȫ˫��/��˫����FULL_DUPLEX / HALF_DUPLEX
			STmin								������
			BlockSize						�����ݰ���	
			Mtype								��Ϣ���ͣ�DIAGNOSTIC / REMOTE_DIAGNOSTICS
			N_SA								����Դ��ַ
			N_TA								����Ŀ���ַ
			N_TAtype						����Ŀ���ַ���ͣ�PHYSICAL / FUNCTIONAL
			N_AE								�����ַ��չ
			FrameFillData				CAN֡�������
*/
void Diagnostic_SetNLParam(TP_T* pTp,uint8_t TimeAs, uint8_t TimeBs, uint8_t TimeCr, uint8_t TimeAr, uint8_t TimeBr, uint8_t TimeCs, 	uint8_t Bs, uint8_t m_STmin, DuplexMode nDuplex ,  MType Mtype , uint8_t N_SA , uint8_t N_TA , N_TAtype N_TAtype , uint8_t N_AE , uint8_t FillData)
{
	NetworkLayer_SetParam(pTp, TimeAs, TimeBs, TimeCr, TimeAr, TimeBr, TimeCs, Bs, m_STmin, nDuplex, Mtype, N_SA, N_TA, N_TAtype, N_AE, FillData);
}

/*
���ܣ�����Ӧ������
������serviceName			�񶨷���ID
			RejectCode			����Ӧ��
*/
void ServiceNegReponse(TP_T* pTp,uint8_t serviceName,uint8_t RejectCode)
{
//	DiagnosticBuffTX[0] = 0x7F;										//�̶�ֵ��
//	DiagnosticBuffTX[1] = serviceName;
//	DiagnosticBuffTX[2] = RejectCode;
//	N_USData_request(pTp, DIAGNOSTIC, N_Sa, N_Ta, PHYSICAL, 0, DiagnosticBuffTX, 3);
}

void Diagnostic_ServiceHandle(uint8_t N_SA , uint8_t N_TA , uint8_t N_TAtype , uint16_t length , uint8_t *MessageData)
{
	bool ValidSid;
	uint16_t ServiceIndex;
	ValidSid = FALSE;
	ServiceIndex = 0;
	CurrentService = MessageData[0];	
	//���ҷ���
	while((ServiceIndex < SERVICE_NUMBER) && (!ValidSid))
	{
		if(ServiceList[ServiceIndex].serviceName == CurrentService)
		{
			if(ServiceList[ServiceIndex].support == TRUE)
			{
				ValidSid = TRUE;
			}
			else//found service but service not enable by application layer
			{
				ValidSid = FALSE;
				break;
			}
		}
		else
		{
			ServiceIndex++;
		}
	}
	if(ValidSid == TRUE)
	{
		//����Ѱַ
		if(N_TAtype == PHYSICAL)
		{
			suppressResponse = FALSE;
			//Ĭ�ϻỰ
			if(ECU_DEFAULT_SESSION == m_CurrSessionType)
			{
				//��֧�ֵİ�ȫ�ȼ�
				if(ServiceList[ServiceIndex].PHYDefaultSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].PHYDefaultSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			//��չ�Ự
			else if(ECU_EXTENED_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].PHYExtendedSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].PHYExtendedSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			//��̻Ự
			else if(ECU_PAOGRAM_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].PHYProgramSeesion_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].PHYProgramSeesion_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			//��Ӧ�̻Ự��������������
			else if(ECU_FACTORY_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].serviceName== SESSION_CONTROL
					|| ServiceList[ServiceIndex].serviceName== SECURITY_ACCESS
					|| ServiceList[ServiceIndex].serviceName== READ_DATA_BY_ID
					|| ServiceList[ServiceIndex].serviceName== WRITE_DATA_BY_ID
					|| ServiceList[ServiceIndex].serviceName== RESET_ECU)
				{
					ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
				}
				else
				{
					m_NRC = SNSIAS;
				}
			}
		}
		//����Ѱַ
		else if(N_TAtype == FUNCTIONAL)
		{
			if(ECU_DEFAULT_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].FUNDefaultSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].FUNDefaultSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			else if(ECU_EXTENED_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].FUNExtendedSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].FUNExtendedSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			else if(ECU_PAOGRAM_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].FUNProgramSeesion_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].FUNProgramSeesion_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			else if(ECU_FACTORY_SESSION == m_CurrSessionType)
			{
				m_NRC = SNS;
			}
		}
	}
	else
	{
		//��Ч����
		if(N_TAtype == PHYSICAL)//����Ѱַ��ЧSED����Ӧ
		{
			m_NRC = SNS;
			suppressResponse = FALSE;
		}
		else
		{
			suppressResponse = TRUE;
		}
	}
}

/*
���ܣ�UDS ��������
������

*/
void Diagnostic_MainProc(TP_T* pTp)
{
	uint32_t rxId = 0;
	if(!IsIndicationListEmpty(pTp))
	{
		NetworkNotification temp = PullIndication(pTp);
		rxId = ((temp.N_SA << 8) + temp.N_TA);
		
		if(temp.NotificationType == INDICATION)
		{
			uint8_t RequestEquipment = 0xFF;	
			if((rxId & 0xFFFF) == (pTp->m_PyhReqID & 0xFFFF) || (rxId & 0xFFFF) == ((pTp->m_FunReqID & 0xFFFF)))
			{
				RequestEquipment = 0;
				pTp->m_AddressFormat.N_TA = (uint8_t)(pTp->m_ResponseID);
				pTp->m_AddressFormat.N_SA = (uint8_t)(pTp->m_ResponseID >> 8);
			}	
			if(RequestEquipment == 0)
			{
				if(temp.N_Resut == N_OK ||temp.N_Resut == N_UNEXP_PDU)
				{
					Diagnostic_ServiceHandle(temp.N_SA,temp.N_TA,temp.N_TAtype,temp.length, temp.MessageData);
					if((temp.N_TAtype == FUNCTIONAL) && ((m_NRC == SNS) || (m_NRC == SFNS) || (m_NRC == SFNSIAS) || (m_NRC == ROOR)) && (ResponsePending == FALSE))
					{
						//printf("res supress,pending =  %d\r\n",ResponsePending);
						/* suppress negative response message */
					}
					else if (suppressResponse == TRUE)
					{
						//printf("res supress bit is TRUE\r\n");
						/* suppress positive response message */
					}
					else
					{
						if(m_NRC == PR)
						{
							N_USData_request(DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , ResponseLength);
						}
						else
						{
							ServiceNegReponse(CurrentService,m_NRC);
						}
					}
					if(m_CurrSessionType  != ECU_DEFAULT_SESSION)
					{
						DiagTimer_Set(&S3serverTimer, 5000);
					}
				}
			}
		}
		else if(temp.NotificationType == CONFIRM)
		{
			if((rxId & 0xFFFF) == (EcuID & 0xFFFF))
			{
				if(WaitConfirmBeforeJump == TRUE)
				{
					
				}
				else if(WaitConfirmBeforeErase == TRUE)
				{
					WaitConfirmBeforeErase = FALSE;
					#ifdef BOOTLOADER
					//Printf("31 service will erase app\r\n");
					//Update_EraseFlash();
					DiagnosticBuffTX[0] = 0x71;
					DiagnosticBuffTX[1] = 0x01;
					DiagnosticBuffTX[2] = 0xFF;
					DiagnosticBuffTX[3] = 0x00;
					DiagnosticBuffTX[4] = 0x00;
					N_USData_request(DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 5);
					#endif
				}
				else if(WaitConfimBeforeReset == TRUE)
				{
					WaitConfimBeforeReset = FALSE;
					if(ResetCallBackFun != NULL)
					{
						ResetCallBackFun(m_EcuResetType);
					}
				}
			}
		}
		else if(temp.NotificationType == FF_INDICATION)
		{
			//Printf("RX FF ind\r\n");
		}
	}
}







