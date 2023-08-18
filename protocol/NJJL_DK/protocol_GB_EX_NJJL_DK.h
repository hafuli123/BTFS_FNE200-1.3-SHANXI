//����ͨ�����ļ�
#ifndef __YZT_DEF_H
#define __YZT_DEF_H

#include "fun_can.h"
#include "bsp_rtc.h"
#include "stdint.h"
#include "cmsis_os2.h"

enum YZT_GB_CODE
{
	CMD_LOCKCAR = 0x83,				 //����ָ��	
	CMD_CARDOORRSP = 0xC0,		 //����Ӧ��
	CMD_UPAPP = 0xC4,					 //��������Ӧ��
	CMD_LOCKCARRRSP = 0xC5,		 //����Ӧ��(��������ϱ�)
};

//�Զ��� 80����
typedef struct _selfData80{
	uint8_t sCarState;																		//����״̬
	uint8_t sRunModel;																		//��������ģʽ
	uint16_t sRunMileage;																		//�������
	uint8_t sPowerAvgConsumption;													//�ٹ���ƽ�����
	uint16_t sVCUTorsion;																	//VCUŤ������
	
	uint16_t sPhaseCurr;																	//�����
	
	uint8_t sHighDevState;																//��ѹ����ʹ��״̬
	uint8_t sVCUTouchControlCmd;													//VCU�Ӵ�������ָ��
	uint8_t sVCUTouchCloseState;													//VCU�Ӵ����պ�״̬
	uint8_t sVCUFault;																		//VCU����
	uint8_t sMCUFault;																		//MCU����
	uint8_t sLifeSign;																		//Life�ź�
	uint8_t sVCUVersionInfo;															//VCU�汾��Ϣ
	uint16_t sCarAcceleratedSpeed;													//�������ٶ�
	uint8_t sCarState1;																	//����״̬1
	uint8_t sCarFaultState1;														//��������״̬1��
	
}SelfData80;

//�Զ��� 81����
typedef struct _selfData81{
	uint8_t sCHGLinkState;																//�������״̬
	uint8_t sBatCoolState;																//�����ȴ״̬
	uint8_t sHeatState;																		//����״̬
	uint8_t sBalanceState;																//����״̬
	uint8_t sBMSFaultShutHighPower;												//BMS�쳣״̬�������жϸ�ѹ
	uint8_t sBMSFaultLevel;																//BMS���ϵȼ�(������ع���״̬)
	
	uint8_t sToucherLinkFault;														//�Ӵ���ճ������
	uint8_t sBranchPreAlarm;															//֧·ѹ���
	uint8_t sFireLimitFaultAlarm;													//���ּ��޹��ϱ���
	uint8_t sBatProtectOut;																//��طŵ籣�������ڳ���С��10A�ŵ�2Сʱ�ж��ܸ���
	uint8_t sSingleBatOffLinState;												//�����ѹ�ɼ�����״̬
	uint8_t sSingleTemOffLinState;												//�¶Ȳɼ�����״̬
	
	uint8_t sBatCoolSystemFault; 													//�����ȴϵͳ����
	uint8_t sHeatFaultAlarmState;													//���ȹ��ϱ���״̬
	uint8_t sBalanceAlarmState;														//���ⱨ��״̬
	uint8_t sPreCHGAlarm;																	//Ԥ��籨��
	uint8_t sCHGDevInfoAlarm;															//�����ͨ�ű���
	uint8_t sBMSControlOffLinAlarm;												//BMS�ӿص��߱��������ĳ�����ݶ�ʧ��
	
	uint8_t sCHGCurrentAlarm;															//����������
	uint8_t sSOCDiffenceAlarm;														//SOC���챨��
	uint8_t sBatLowTemAlarm;															//��ص��±���
	uint8_t sBMSCommunicationFault;												//BMSͨѶ���ϣ����canӲ�����ϣ�
	uint8_t sBatSystemOtherFault;													//���ϵͳ��������
	
	uint8_t sPoleColumnHighTemAlarm;											//�������±���
	uint8_t sCHGGunHighTemAlarm;													//���ǹ���±���
	uint8_t sOutCurrentAlarm;															//�ŵ��������

	uint8_t sSOCHighAlarm;																//SOC�߱���
	uint8_t sInsulationAlarmState;												//��Ե��ⱨ��״̬
	
	uint8_t sSingleLowVlotAlarm;													//����Ƿѹ����
	uint8_t sSingleDifVlotAlarm;													//�����ѹ���챨��
	uint8_t sTemDiffAlarm1;																//�¶Ȳ��챨��1
	uint8_t sSOCLowAlarm1;																//SOC�ͱ���1
	
	uint8_t sBatHighTemAlarm1;														//��ظ��±���1
	uint8_t sSingleOverVlotAlarm;													//�����ѹ����
	uint8_t sBatsOverVlotAlarm;														//������ѹ����
	uint8_t sBatsLowVlotAlarm;														//�����Ƿѹ����
	
	uint8_t sBatFaultNum;																	//������ع�����
	uint16_t sBatsCount;																	//������ܴ���
	uint8_t sBatsTemsCount;																//������¶ȵ���
	
	uint8_t sMaxLongInCurrent;														//�����ó�����������5min��
	uint8_t sMaxShortInCurrent;														//�����ö�ʱ��������30s��
	uint8_t sMaxLongOutCurrent;														//�����ó����ŵ������5min��
	uint8_t sMaxShortOutCurrent;													//�����ö�ʱ�ŵ������30s��
	
	uint8_t sBMSTouchControlCMD;													//BMS�Ӵ�����������
	uint8_t sBMSTouchControlCloseState;										//BMS�Ӵ����պ�״̬
	
	uint16_t sCHGCounts;																	//������
	uint32_t sBMSAddUpOutPower;															//������ۼ��������
	uint32_t sBMSAddUpChgPower;															//������ۼƳ�磨�����ƶ�����������
	
	uint16_t sInsulationCheckAllVolt;											//��Ե�������ѹ
	
	uint8_t sCHGPlusesTem1;																//�����1���¶�
	uint8_t sCHGMinusTem1;																//���ǹ1���¶�
	uint8_t sCHGPlusesTem2;																//�����2���¶�
	uint8_t sCHGMinusTem2;																//���ǹ2���¶�
	
	uint8_t sBatsProductDate_Month;												//������������ڣ��£�
	uint8_t sBatsProductDate_Year;												//������������ڣ��꣩
	uint8_t sBatsProducer;																//���������������
	uint8_t sBMSLifeSignal;																//BMS life�ź�
	uint32_t sBMSSoftwareVersion;														//BMS����汾
}SelfData81;

//�Զ��� 82����
typedef struct _selfData82{
	uint8_t sFourInOne_State;															//4��1״̬
	uint8_t sFourInOne_FaultNum;													//4��1������
	uint8_t sFourInOne_BMSToucherState;										//4��1 - BMS�Ӵ���״̬����
	uint8_t sFourInOne_VCUToucherState;										//4��1 - VCU�Ӵ���״̬����
	uint8_t sFourInOne_BMSToucherFaultState;							//4��1 - BMS�Ӵ�������״̬
	uint8_t sFourInOne_VCUToucherFaultState;							//4��1 - VCU�Ӵ�������״̬
	
	uint16_t sHighOilPump_OutVolt;														//��ѹ�ͱ������ѹ							
	uint8_t sHighOilPump_DCACOutCur;												//��ѹ�ͱ�DC/AC�������
	uint8_t sHighOilPump_MoterTem;												//��ѹ�ͱõ���¶�
	uint8_t sHighOilPump_DCACStateAndFault;								//��ѹ�ͱ�DC/AC״̬������
	uint8_t sHighOilPump_ConverterFaultNum;								//��ѹ�ͱñ�Ƶ��������
	uint8_t sHighOilPump_motorSpeed;											//��ѹ�ͱ�ת��
	uint8_t sHighOilPump_DCACLifeSignal;									//��ѹ�ͱ�DC/AC life�ź�
	
	uint8_t sDCDC_RealTimeOutCur;														//DC/DCʵʱ�������
	uint8_t sDCDCTem;																			//DC/DC�����¶�
	uint8_t sDCDCWorkState;																//DCDC����״̬
	uint8_t sDCDCLifeSignal;															//DCDC Life�ź�
	
	uint16_t sAirPump_DCACOutVolt;												//����DC/AC�����ѹ
	uint8_t sAirPump_DCACOutCur;												//����DC/AC�������
	uint8_t sAirPump_DCACStateAndFault;										//����DC/AC״̬������
	uint8_t sAirPump_Tem;																	//�����¶�
	uint8_t sAirPump_ConverterFaultNum;										//���ñ�Ƶ��������
	uint8_t sAirPump_motorSpeed;													//����ת��
	uint8_t sAirPump_DCACLifeSignal;											//����DC/AC Life�ź�
	
	uint16_t sLowOilPump_OutVolt;													//��ѹ�ͱ������ѹ							
	uint8_t sLowOilPump_DCACOutCur;												//��ѹ�ͱ�DC/AC�������
	uint8_t sLowOilPump_DCACStateAndFault;								//��ѹ�ͱ�DC/AC״̬������
	uint8_t sLowOilPump_ConverterFaultNum;								//��ѹ�ͱñ�Ƶ��������
	uint8_t sLowOilPump_motorSpeed;												//��ѹ�ͱ�ת��
	uint8_t sLowOilPump_DCACLifeSignal;										//��ѹ�ͱ�DC/AC life�ź�
	
}SelfData82;

//�Զ��� 83����
typedef struct _selfData83{
	uint8_t sLowBatVolt;																	//��ѹ��ص�ѹ
	uint8_t sFrontBrakeAirPressure;												//ǰ�ƶ�����Ͳ��ѹ
	uint8_t sRearBrakeAirPressure;												//���ƶ�����Ͳ��ѹ
	
	uint32_t sAllMileage;																		//�����
	
	uint8_t sCarState1;																		//����״̬1
	uint8_t sCarState2;																		//����״̬2
	uint8_t sCarState3;																		//����״̬3
	
	uint8_t sInstrumentAlarmState1;												//�Ǳ���״̬1
	uint8_t sInstrumentAlarmState2;												//�Ǳ���״̬2
	uint8_t sInstrumentSoftwareVersion;										//�Ǳ����汾
}SelfData83;

//�Զ��� 84����
typedef struct _selfData84{
	uint8_t sBMSCoolWorkMode;															//BMS��ȴ������ģʽ
	uint8_t sBMSsetoutWaterTem;														//BMSˮ������ˮ�ڣ������ˮ�ڣ��趨�¶�
	uint8_t sBatsHighestTem;															//���������¶�
	uint8_t sBatsLowestTem;																//���������¶�
	uint8_t sBatsQueLifeValue;														//���������lifeֵ
	uint8_t sHotContrlMode;																//�ȹ���ϵͳ����ģʽ����
	uint8_t sInWaterTem;																	//��ˮ�¶�
	uint8_t sOutWaterTem;																	//��ˮ�¶�
	uint8_t sCompressorPower;															//ѹ�������ر���
	uint8_t sHotContrlFaultNum;														//�ȹ���ϵͳ������
	uint8_t sHotContrlLifeValue;													//�ȹ�����lifeֵ
}SelfData84;

//�Զ��� 85����
typedef struct _selfData85{
	uint8_t sAirConditionerOpenCMD;												//�յ����ػ�����/״̬
	uint8_t sAirConditionerSetTem;												//�յ��趨�¶�
	uint8_t sAirConditionerRunStall;											//�յ�������е�λ
	uint8_t sInCarTem;																		//���ڻ����¶�
	uint8_t sAirConditionerRunMode;												//�յ���������ģʽ
	uint8_t sOutCarTem;																		//���⻷���¶�
	uint8_t sAirToucherContrlAndState;										//�յ��Ӵ��������Լ�״̬
	uint16_t sAirSystemVolt;															//�յ�ϵͳĸ�ߵ�ѹ
	uint8_t sAirSystem_PartsRunState;											//�յ�ϵͳһ��������״̬
	uint8_t sAirSystem_SystemRunState;										//�յ�ϵͳһϵͳ����״̬
	uint8_t sAirRunTargetHz;															//ѹ����Ŀ��Ƶ��
	uint8_t sAirRunHz;																		//ѹ��������Ƶ��
	uint8_t sAirFaultNum;																	//�յ�������
	uint8_t sAirLife;																			//�յ�life
}SelfData85;

//�Զ��� 86����
typedef struct _selfData86{
	uint8_t sTirePosition;															//��̥λ��
	uint8_t sTirePressure;																//��̥ѹ��
	uint16_t sTireTem;																			//��̥�¶�
	uint8_t sTireState;																	//״̬����̥״̬��
	uint8_t sPressureValveCheck;												//ѹ�������
}SelfData86;

//�Զ��� 87����
typedef struct _selfData87{
	uint8_t sFlameArrester_SystemState;										//�����ϵͳ״̬
	uint8_t sBatsNum;																			//������
	uint8_t sSensorBoxState;															//�����ڴ���������״̬
	uint8_t sSensorBoxFault;															//�����ڹ���״̬�ȼ�
	uint8_t sSensorBoxTem;																//�������¶�
	uint8_t sSensorBoxStartState;													//���������������״̬
	uint8_t sSensorBoxLife;																//LIFE
}SelfData87;

//�Զ��� 88����
typedef struct _selfData88{
	uint8_t sAheadCarWarning;															//ǰ����ײ����
	uint16_t sAheadCarDistance;														//ǰ������
	uint16_t sAheadCar_RelativeSpeed;											//����ٶ�
	uint8_t sLaneWarning;																	//����ƫ�뾯��
	uint8_t sLaneDirection;																//����ƫ�뷽��
	uint8_t sActivateCollisionWarning;										//������ײԤ������
	uint8_t sActivateEmergencyBraking;										//���������ƶ�����
	uint8_t	sABESystemState;															//AEBϵͳ״̬
}SelfData88;

typedef struct{
	/*********************Զ���������****************/
	uint8_t carDoorCtrCode;						//�������ƴ���  1:����ָ�� 2:����ָ�� ����:�޲���
	uint8_t isReturnLockState;				//0xA5:����Ӧ�� 0xAA����Чָ��
	uint16_t carDoorCtrCount;					//Զ��������ˮ�� 
	uint8_t isHeartLock;							//�������Ƿ���		0 �رգ�1 ����
	/*Զ���������ܲ���*/
	uint16_t carDoorCtrRspCode;				//Զ������Ӧ�� 0x00:�ս��յ�ƽָ̨�� 0xFF:�ȵ�
	uint8_t remoteLockState;					//Զ������״̬
	uint8_t heartLockState;						//������״̬
	uint8_t randomKey;								//�����Կ		
}REMOTELOCK_PARA;


extern SelfData80* pSelfData80;					
extern SelfData81* pSelfData81;
extern SelfData82* pSelfData82;
extern SelfData83* pSelfData83;
extern SelfData84* pSelfData84;
extern SelfData85* pSelfData85;
extern SelfData86* pSelfData86;
extern SelfData87* pSelfData87;
extern SelfData88* pSelfData88;
extern REMOTELOCK_PARA gRemoteLockPara;		/* Զ���������� */
extern uint8_t sendLockCMDSign;

extern void unpackYZTcan(uint8_t ch,CAN_msg *msg);

#endif

