#ifndef __SHWX_GBEX_H
#define __SHWX_GBEX_H

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"
#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_rtc.h"
#include "bsp_ec20.h"
#include "algo_verify.h"
#include "protocol_GB.h"

#include "fun_mon.h"
#include "bsp_gps.h"
#include "bsp_storage.h"
#include "bsp_io.h"
#include "bsp_sys.h"
#include "bsp_power.h"
#include "protocol_GB32960.h"
#include "bsp_storage.h"
#include "rl_fs.h"
#include "bsp_uart_fifo.h"

//�Զ��� 80����
typedef struct _selfData80{

	uint8_t vcucheckfault;//VCU�Լ����
    uint8_t Prechargingfault;//Ԥ�����
    uint8_t Gearfault;       //��λ����
    uint8_t Brakepedalfault; //�ƶ�̤�����
    uint8_t Acceleratorpedalfault; //����̤�����
    uint8_t Bmsoffline;             //BMS�ڵ��ܵ����
    uint8_t Instrumentoffline;      //�Ǳ�ڵ��ܵ����
    uint8_t Maincontactorfault;     //���Ӵ�������
    uint8_t Batterylowtemalarm;     //��ص��±���
    uint8_t Batteryrechargefault;   //��ػس�������ޱ���
    uint8_t Batterydischargecurrentfault;//��طŵ�������ޱ���
    uint8_t Heatingcontactorfault;   //�������Ӵ�������
    uint8_t Mainnegativecontactorfault; //�����Ӵ�������
    uint8_t Dcchargingcontactorfault; //ֱ�����Ӵ���
    uint8_t Mainpositivecontactor;      //�����Ӵ�������
    uint8_t BMSinternalCANcommunicationfault; //BMS�ڲ�CANͨѶ����
    uint8_t BMS24Vpowersupplyfault;         //BMS��24V�������
    uint8_t batteryinterlockfault;          //��ػ�������
    uint8_t Chargingoverheatedfault;        //����������
    uint8_t Motoroverspeedfault;           //������ٱ���
    uint8_t Leakagealarm;                      //©�籨��
    uint8_t Wadingalarm;                   //��ˮ����
}SelfData80;

//�Զ��� 81���� ��������
typedef struct _selfData81{
	uint8_t AirPressure1;	//��ѹ 1
	uint8_t AirPressure2;	//��ѹ 2
	float BatteryVoltage;	//���ص�ѹ
	uint8_t FrontDoorStatus:2;	//ǰ��״̬
	uint8_t MiddleDoorStatus:2;	//����״̬
	uint8_t RearDoorStatus:2;	//����״̬
	uint8_t RearHatchStatus:2;	//�����״̬
	uint8_t HandBrakeSignal:2;	//��ɲ�ź�
	uint8_t FootBrakeSignal:2;	//��ɲ�ź�
	uint8_t KeyLocation:3;	//Կ��λ��
	uint8_t LeftTurnSignal:2;	//��ת���
	uint8_t RightTurnSignal:2;	//��ת���
	uint8_t PositionLight:2;	//λ�õ�
	uint8_t NearLightLamp:2;	//�����
	uint8_t HighBeamLamp:2;	//Զ���
	uint8_t FrontFogLight:2;	//ǰ���
	uint8_t RearFogLamp:2;	//�����
	uint8_t Wiper:2;	//���
	
}SelfData81;

//�Զ��� 82���� ��������������
typedef struct _selfData82{
	uint8_t PrechargeControlSignal:2;	//Ԥ������ź�
	uint8_t MainContactorPositiveControlSignal:2;	//���Ӵ����������ź�
	uint8_t MainContactorFeedbackSignal:2;	//���Ӵ��������ź�
	uint8_t MainContactorNegativeControlSignal:2;	//���Ӵ����������ź�
	uint8_t MainContactorNegativeFeedbackSignal:2;	//���Ӵ����������ź�
	uint8_t EmergencyPoweroffRequest:2;	//�����µ�����
	uint8_t FaultCode;	//������
	uint8_t WarningCode;	//������
	uint8_t DriveSystemFaultCode;	//����ϵͳ������
	
}SelfData82;

//�Զ��� 83���� �����������
typedef struct _selfData83{
    uint16_t RangeDriving;	//�������
	float SOH;	//SOH
	float batteryMaximumAllowableDischargeCurrent;	//����������ŵ����
	float batteryMaximumAllowableChargeCurrent;	//���������������
	float BatteryVoltageInsideMainRelay;	//��ص�ѹ�����̵����ڲࣩ
	float BatteryVoltageOutsideMainRelay;	//��ص�ѹ�����̵�����ࣩ
	float AccumulatedChargeCapacity;	//�ۻ�������
	float AccumulatedDischargeCapacity;	// �ۻ��ŵ����
	float MotorFeedbackQuantity;	//�����������
	
    uint16_t batteryPackPositiveInsulation;	//��ذ�������Եֵ
	uint16_t batteryPackNegativeInsulation;	//��ذ�������Եֵ
	uint8_t EmergencyPoweroffRequest:2;	//�����µ�����
	uint8_t BatteryEquilibriumState:2;	// ��ؾ���״̬
	
}SelfData83;

typedef struct selfData83
{
    float SOH;	//SOH
}data83;

//�Զ��� 84���� DCDC ����
typedef struct _selfData84{
	float DCDC_OutputCurrent;	//DC/DC �������
	float DCDC_OutputVoltage;	//DC/DC �����ѹ
	float DCDC_InputVoltage;	//DC/DC �����ѹ
	uint8_t DCDC_HeatSinkTemperature;	//DC/DC ɢ�����¶�
	uint8_t DCDC_EnableSignal:1;	//DC/DC ʹ���ź�
	
}SelfData84;

//�Զ��� 85���� ���� DC/AC ����
typedef struct _selfData85{
	float AirPumpDCAC_U_PhaseOutputCurrent;	//���� DC/AC U ���������
	float AirPumpDCAC_V_PhaseOutputCurrent;	//���� DC/AC V ���������
	float AirPumpDCAC_W_PhaseOutputCurrent;	//����DC/AC W ���������
	uint8_t AirPump_DCAC_HeatSinkTemperature;	//���� DC/AC ɢ�����¶�
	uint8_t AirPump_DCAC_EnableSignal:1;	//���� DC/AC ʹ���ź�
	
}SelfData85;

//�Զ��� 86���� �ͱ� DC/AC ����
typedef struct _selfData86{
	float OilPumpDCAC_U_PhaseOutputCurrent;	//�ͱ� DC/AC U ���������
	float OilPumpDCAC_V_PhaseOutputCurrent;	//�ͱ� DC/AC V ���������
	float OilPumpDCAC_W_PhaseOutputCurrent;	//�ͱ�DC/AC W ���������
	uint8_t OilPump_DCAC_HeatSinkTemperature;	//�ͱ� DC/AC ɢ�����¶�
	uint8_t OilPump_DCAC_EnableSignal:1;	//�ͱ� DC/AC ʹ���ź�
	
}SelfData86;

//�Զ��� 87���� �յ�����
typedef struct _selfData87{
	uint8_t AirConditionerLowSpeed:1;	//�յ�����
	uint8_t AirConditioningMediumSpeed:1;	//�յ�����
	uint8_t AirConditionerHighSpeed:1;	//�յ�����
	uint8_t AirConditioningHeating:1;	//�յ�����
	uint8_t AirConditioningRefrigerationDefrosting1:1;	//�յ����� 1 ��˪
	uint8_t AirConditioningFreshAir:1;	//�յ��·�
	uint8_t AirConditioningSterilization:1;	//�յ�ɱ��
	uint8_t AirConditioningRefrigerationDefrosting2:1;	//�յ����� 2 ��˪
	uint8_t AirConditioningRefrigeration2:1;	// �յ����� 2
	uint8_t AirConditioningRefrigeration1:1;	//�յ����� 1
	float InsideCarTemperature;	//�����¶�
	float OutsideCarTemperature;	//�����¶�
	
}SelfData87;

//�Զ��� 88���� ����������
typedef struct _selfData88{
	uint8_t LubricationPressureState:2;	//��ѹ��״̬
	uint8_t LubricatingOilLevelState:2;	//����λ״̬
	uint8_t LubricatingMotorState:2;	//�󻬵��״̬
	uint8_t LubricationSystemStatus:2;	// ��ϵͳ״̬
	
}SelfData88;

//�Զ��� 89���� ����������
typedef struct _selfData89{
	uint8_t EngineWaterTemperature;	//������ˮ��
	uint8_t EngineOilPressure;	//����������ѹ��
	float RemainingOilAmount;	//ʣ������
	uint8_t UreaLevel;	//����Һλ
	
}SelfData89;

//�Զ��� 8A���� ̥ѹ�������
typedef struct _selfData8A{
	uint8_t LeftFrontTirePressure;	//��ǰ��̥ѹ��
	float LeftFrontTireTemperature;	//��ǰ��̥�¶�
	uint8_t RightFrontTirePressure;	//��ǰ��̥ѹ��
	float RightFrontTireTemperature;	// ��ǰ��̥�¶�
	uint8_t LeftRear1TirePressure;	//��� 1 ��̥ѹ��
	float LeftRear1TireTemperature;	//��� 1 ��̥�¶�
	uint8_t LeftRear2TirePressure;	//��� 2 ��̥ѹ��
	float LeftRear2TireTemperature;	//��� 2 ��̥�¶�
	uint8_t RightRear1TirePressure;	//  �Һ� 1 ��̥ѹ��
	float RightRear1TireTemperature;	//�Һ� 1 ��̥�¶�
	uint8_t RightRear2TirePressure;	// �Һ� 2 ��̥ѹ��
	float RightRear2TireTemperature;	//�Һ� 2 ��̥�¶�
	
}SelfData8A;

//�Զ��� 8B���� �����ȴϵͳ TMS ����
typedef struct _selfData8B{
	uint8_t TMS_OperatingStatus:2;	//TMS ����״̬
	uint8_t TMS_HighVoltageRelayStatus:2;	//TMS ��ѹ�̵���״̬
	uint8_t WaterOutletTemperature;	//��ˮ�¶�
	uint8_t WaterInletTemperature;	// ��ˮ�¶�
	uint8_t TMS_FaultCode;	//TMS ������
	
}SelfData8B;

//�Զ��� 8C���� �������
typedef struct _selfData8C{
	float DCDC_InstantaneousPower;	//DC/DC ˲ʱ����
	float SteeringOilPumpInstantaneousPower;	//ת���ͱ�˲ʱ����
	float AirPumpInstantaneousPower;	// ����˲ʱ����
	float DriveSystemInstantaneousPower;	// ����ϵͳ˲ʱ����
	float DriveSystemRemainingPower;	//����ϵͳʣ�๦��
	float DCDC_Power_Consumption;	//DC/DC ���
	float PowerConsumptionSteeringOilPump;	//ת���ͱõ��
	float AirPumpPowerConsumption;	// ���õ��
	float DriveSystemPowerConsumption;	// ����ϵͳ���
	float RangeDriving;	// ��ʻ���
	
}SelfData8C;

//�Զ��� 8D���� ���˪����
typedef struct _selfData8D{
	uint8_t DefrosterHighPressureContactorStatus:2;	//��˪���ڸ�ѹ�Ӵ���״̬
	uint8_t FanStatus:2;	//���״̬
	uint8_t DefrostEnable:2;		//���˪ʹ��
	uint8_t HeatingBodyTemperature;	//�������¶�
	uint8_t DefrosterEnclosureTemperature;	// ��˪������¶�
	uint8_t DefrosterOutletTemperature;	//��˪�������¶�
	float HeatingBodyCurrent;	//���������
	float DefrosterWorkingPower;	//��˪����������
	
}SelfData8D;

//�Զ��� 8E���� ���Ӳ���������
typedef struct _selfData8E{
	uint8_t EDC_TurnSign;	//EDC ת���־
	uint8_t EDC_WorkingMode;	//EDC ����ģʽ
	
}SelfData8E;

//�Զ��� 8F���� ������������
typedef struct _selfData8F{
	uint16_t ChargeCumulativeNumber;	//����ۼƴ���
	uint16_t OilPumpDCAC_LineVoltage;	//�ͱ� DCAC �ߵ�ѹ
	uint16_t AirPumpDCAC_LineVoltage;	//���� DCAC �ߵ�ѹ
	uint8_t AirConditioningInstantaneousPower;	//�յ�˲ʱ����
	uint8_t LeftFrontExplosionproofTireDeviceStatus:1;	//��ǰ����̥װ��״̬
	uint8_t RightFrontExplosionproofTireDeviceStatus:1;	//��ǰ����̥װ��״̬
	uint8_t LeftRear1ExplosionproofTireDeviceStatus:1;	//��� 1 ����̥װ��״̬
	uint8_t LeftRear2ExplosionproofTireDeviceStatus:1;	//��� 2 ����̥װ��״̬
	uint8_t RightRear1ExplosionproofTireDeviceStatus:1;	//�Һ� 1 ����̥װ��״̬
	uint8_t RightRear2ExplosionproofTireDeviceStatus:1;	//�Һ� 2 ����̥װ��״̬
	float DrontAxleLeftBrakeShoeResidualAmount;	//ǰ�����ƶ���Ƭʣ����
	float DrontAxleRightBrakeShoeResidualAmount;	//ǰ�����ƶ���Ƭʣ����
	float RearAxle1LeftBrakeShoeRemainingAmount;	//���� 1 ���ƶ���Ƭʣ����
	float RearAxle1RightBrakeShoeRemainingAmount;	//���� 1 ���ƶ���Ƭʣ����
	uint8_t TotalBatteryPacks;	//��ذ�����
	uint8_t AirConditioningSettingTemperature;	//�յ��趨�¶�
	
}SelfData8F;

//�Զ��� 90���� ����������ϵͳ����
typedef struct _selfData90{
	uint8_t DetectorNum;	//̽������ţ������ţ�
	uint8_t FireLevel;	//�𾯼���
	uint8_t SystemStatus;	//ϵͳ״̬
	uint8_t SubvalveControlCommandStatus;	//�ӷ���������״̬
	
}SelfData90;

//�Զ��� 91���� ADAS ����
typedef struct _selfData91{
	uint8_t LaneDepartureWarningStatus:2;	//����ƫ��Ԥ��״̬
	uint8_t ForwardCollisionWarningStatus:2;	//ǰײԤ��״̬
	uint8_t AEB_WorkingStatusAlert:2;	//AEB ����״̬��ʾ
	uint8_t VehicleWarningSign:2;	//����Ԥ����־
	float FrontVehicleRelativeSpeed;	//��ǰ������ٶ�
	float FrontVehicleRelativeDistance;	//��ǰ������
	uint8_t AEBS_FaultStatus:2;	//AEBS ����״̬
	uint8_t PrimaryControllerFailure:2;	//������������
	uint8_t CommunicationFailure:2;	//ͨѶ���ϣ��복��ͨѶ��
	uint8_t PedestrianWarningSigns:2;	//����Ԥ����־
	float TimeIntervalValue;	//ʱ����ֵ
	uint8_t LaneDepartureSystemWorkingStatus:3;	//����ƫ��ϵͳ����״̬
	uint8_t LaneKeepingSystemWorkingStatus:3;	//��������ϵͳ����״̬
	uint8_t ImageSensorFault:2;	//ͼ�񴫸�������
	uint8_t AdaptiveCruiseSystemOperatingState:3;	//����ӦѲ��ϵͳ����״̬
	uint8_t ForwardCollisionWarningSystemOperatingStatus:3;	//ǰ����ײԤ��ϵͳ����״̬
	uint8_t ImageSensorCommunicationFaulty:2;	//ͼ�񴫸���ͨѶ����
	uint8_t CollisionMitigationSystemWorkingStatus:3;	//��ײ����ϵͳ����״̬
	uint8_t ThrottleAntimisstepSystemWorkingStatus:3;	//���ŷ����ϵͳ����״̬
	uint8_t AuxiliaryControllerFailure:2;	//��������������
	uint8_t ImageSensorWorkingStatus:3;	//ͼ�񴫸�������״̬
	uint8_t Millimeter_waveRadarOperatingStatus:3;	//���ײ��״﹤��״̬
	uint8_t AuxiliaryControllerCommunicationFaulty:2;	//����������ͨѶ����
	uint8_t Millimeter_waveRadarFaulty:2;	//���ײ��״����
	uint8_t Millimeter_waveRadarCommunicationFaulty:2;	//���ײ��״�ͨѶ����
	uint16_t CSVU_self_CheckStatus;	//CSVU �Լ�״̬
	
}SelfData91;

extern SelfData80* pSelfData80;					
extern SelfData81* pSelfData81;
extern SelfData82* pSelfData82;
extern SelfData83 pSelfData83;
extern SelfData84 pSelfData84;
extern SelfData85 pSelfData85;
extern SelfData86 pSelfData86;
extern SelfData87 pSelfData87;
extern SelfData88 pSelfData88;
extern SelfData89 pSelfData89;					
extern SelfData8A pSelfData8A;
extern SelfData8B pSelfData8B;
extern SelfData8C pSelfData8C;
extern SelfData8D pSelfData8D;
extern SelfData8E pSelfData8E;
extern SelfData8F pSelfData8F;
extern SelfData90 pSelfData90;
extern SelfData91 pSelfData91;


uint16_t extSHDBReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen);

uint16_t extHyFuelCellReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen);


#endif
