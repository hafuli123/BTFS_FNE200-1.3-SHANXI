/***************************************************  电池安全预警算法算例函数接口头文件定义 *******************************************************/
/* 算例函数包含初始化函数initialFunction、算例1函数battsafetyFunction1、算例2函数battsafetyFunction2、算例3函数battsafetyFunction3 */
/* 函数接口结构体YJ_MESSAGE定义：
	(1) 函数输入：UTC秒级时间戳、电池包串联单体数量、电池包充电状态、电池包总电压、电池包SOC、电池包电流、电池包单体电压数组。
	(2) 算例推送的计算结果 */
/* 函数调用规程：
	(1) 初始化函数initialFunction仅在终端上电时调用一次，用于初始化函数接口的结构体，正常返回0，错误返回非0；
	(2) 初始化成功后，读取Flash缓存数据，Flash结构体特殊说明如下：
	      1）结构体Flash_DATA1用于算例2和算例3，终端每次下电时需要存储结构体Flash_DATA1内所有的数据，然后终端每次上电时需要通过结构体Flash_DATA1把存储的数据读取出来;
	      2） 结构体Flash_DATA2用于算例2；Flash_DATA2读写的另外一个触发条件就是车辆充电状态（0为放电/1为充电）的转变，读写规则参见附件PDF；
	(3) Flash读取完成后，依次调用算例1~算例3，调用逻辑为：先调用算例1函数battsafetyFunction1，算例1函数计算完成后调用算例2函数battsafetyFunction2，算例2函数计算完成后调用算例3函数battsafetyFunction3；
	(4) 算例函数1~3调用计算完成后，统一上报算例的计算结果；
	(5) 算例函数的调用时间周期为5秒。  */
/**********************************************   Edison Wei,  March 2022 in Shenzhen      **************************************************************/

#include "stdint.h"

// 库文件版本号，用于擦除flash数据：前后两次值不一致定义为擦除Flash；上次程序为0，当前程序为0
#define Cache_Version	0 										

// 程序固化定义参量，不可修改
#define MAX_CELL_CNT 104      							
#define MAX_Fault_CELL_CNT 5   						
#define MAX_diffocv_CNT 40 								
#define Char_CNT 5 		
#define Panduan_base_CNT 5 
#define Panduan_yujing_error_TimeCNT 40
#define UTC_CNT_PanduanBase 3
#define Max_Data_Save_CNT 250

// 算例接口结构体
typedef struct {
	uint32_t UTC_TIME;														// 函数输入：UTC时间戳，分辨率：秒
	uint8_t Nbat_s;																// 函数输入：电池包串联单体数量，有效值：1~MAX_CELL_CNT
	uint8_t Charge_state;													// 函数输入：充电状态标志位，有效值0或1：1表示充电中，0表示为非充电状态
	uint8_t SOC;																	// 函数输入：电池包SOC，单位为%，有效值：0~110
	float Current;																// 函数输入：电池包电流，单位为A，放电为正
	float PackVoltage;															// 函数输入：电池包电压，单位为V
	float* cellV;																	// 函数输入：单体电压数组，单位为V，维数不超过MAX_CELL_CNT
	uint8_t SWD_VERSION;													// 算例推出：算法版本号，有效值：100~250，即1.0.0~2.5.0
	uint8_t life1;   																// 算例推出：过程参数，有效值：0~250
	uint8_t life2;   																// 算例推出：过程参数，有效值：0~250
	uint8_t life3;   																// 算例推出：过程参数，有效值：0~250
	uint8_t fault_OCVID;   													// 算例推出：过程参数，有效值：0~100
	uint16_t mean_OCV;														// 算例推出：过程参数，有效值：0~5000
	uint16_t OCV_interval_CNT;											// 算例推出：过程参数，有效值：0~65500
	uint8_t fault_state1;   													// 算例推出：结果参数，有效值：0或者1，0表示正常,1表示异常
	uint8_t fault_state2;   													// 算例推出：结果参数，有效值：0或者1，0表示正常,1表示异常
	uint8_t fault_state3;   													// 算例推出：结果参数，有效值：0或者1，0表示正常,1表示异常
	uint8_t fault_state4;   													// 算例推出：结果参数，有效值：0或者1，0表示正常,1表示异常
	uint8_t fault_state5;   													// 算例推出：结果参数，有效值：0或者1，0表示正常,1表示异常
	uint8_t fault_cell_number;											// 算例推出：结果参数，有效值：0~5
	uint8_t fault_cell_ID[MAX_Fault_CELL_CNT];			// 算例推出：结果参数，1×5的数组，数组中每一个元素的有效值：0~250
	uint8_t fault_cell_Type[MAX_Fault_CELL_CNT];		// 算例推出：结果参数，1×5的数组，数组中每一个元素的有效值：0~250
	uint8_t PF_Total[MAX_CELL_CNT];							// 算例推出：结果参数，1×104的数组，数组中每一个元素的有效值：0~250
}YJ_MESSAGE;

// 存储于flash的结构体：基于上下电进行读写
typedef struct {
	uint8_t Cache_flag;																																			// 算例Flash变量
	uint8_t Cache_Flag_SOC_Highin;																													// 算例Flash变量
	uint8_t Cache_Flag_SOC_Middlehold;																											// 算例Flash变量
	uint8_t Cache_Flag_SOC_Lowout;																													// 算例Flash变量
	uint8_t Cache_Flag_SOC_RangeOK;																												// 算例Flash变量
	uint8_t Cache_New_charge_flag;																													// 算例Flash变量
	uint8_t Cache_EKF_TimeCNT;																														// 算例Flash变量
	uint8_t Cache_ocv_interval_jump_flag;																										// 算例Flash变量
	uint8_t Cache_OCV_ID_k;																																// 算例Flash变量
	uint8_t Cache_Panduan_yujing_error_TimeCNT_Disch;																			// 算例Flash变量
	uint8_t Cache_Panduan_yujing_error_TimeCNT_Char; 																			// 算例Flash变量
	uint8_t CC_Charge_Flag;																																	// 算例Flash变量
	uint16_t Cache_ocv_ID_CNT;																															// 算例Flash变量
	uint16_t Cache_Charge_CNT;																														// 算例Flash变量
	uint8_t Cache_EarlyCharge_CNT;																													// 算例Flash变量
	int16_t Cache_mean_OCV_k0;																														// 算例Flash变量
	uint8_t Cache_Fault_CNT;																																// 算例Flash变量
	uint8_t FaultCell_CNT_New;																															// 算例Flash变量
	uint8_t FaultCellOk_CNT_New;																														// 算例Flash变量
	uint8_t Disch_FaultCellOk_CNT_Last;																											// 算例Flash变量
	uint8_t Char_FaultCellOk_CNT_Last;																												// 算例Flash变量
	uint8_t FaultCell_CNT_Last;																															// 算例Flash变量
	uint8_t FaultOK_CNT;																																		// 算例Flash变量
	uint8_t Fault_OCV_ID;																																		// 算例Flash变量
	uint8_t Fault_Yes_Flag;																																	// 算例Flash变量
	uint8_t Disch_Fault_Yes_Flag;																														// 算例Flash变量
	uint8_t Char_Fault_Yes_Flag;																															// 算例Flash变量
	uint32_t dOCV_SOC_CNT;																																// 算例Flash变量
	float Cache_panduan_yujing_disch[MAX_CELL_CNT];																				// 算例Flash多维数组
	float Cache_X_celiang_M[MAX_CELL_CNT + 1][2];																					// 算例Flash多维数组
	float Cache_P_celiang_M[MAX_CELL_CNT + 1][4];																					// 算例Flash多维数组
	float Cache_R_charge[MAX_CELL_CNT + 1][Char_CNT];																			// 算例Flash多维数组
	uint8_t Cache_FaultCell_ID_SameProcess[MAX_Fault_CELL_CNT][4];													// 算例Flash多维数组
	uint8_t Cache_Disch_FaultCell_ID_DiffProcess_Last[MAX_Fault_CELL_CNT];										// 算例Flash多维数组
	uint8_t Cache_Char_FaultCell_ID_DiffProcess_Last[MAX_Fault_CELL_CNT];											// 算例Flash多维数组
	uint8_t Cache_FaultCell_ID_DiffProcess_New[MAX_Fault_CELL_CNT];													// 算例Flash多维数组
	float Cache_dOCV_SOC[MAX_CELL_CNT];																									// 算例Flash多维数组
	uint8_t Suanli2_ThresholdFlag_Disch[MAX_CELL_CNT];																			// 算例Flash多维数组
	uint8_t Suanli2_ThresholdUpdata_Disch[MAX_CELL_CNT][3];																// 算例Flash多维数组
	uint8_t Suanli2_ThresholdFlag_Char[MAX_CELL_CNT];																			// 算例Flash多维数组
	uint8_t Suanli2_ThresholdUpdata_Char[MAX_CELL_CNT][3];																    // 算例Flash多维数组
	uint8_t Cache_dSOC_Cycles[MAX_CELL_CNT][Panduan_yujing_error_TimeCNT];								// 算例Flash多维数组
	uint8_t Cache_dQ_Cycles[MAX_CELL_CNT][Panduan_yujing_error_TimeCNT];									// 算例Flash多维数组
	float Cache_dSOC_Cal_Time[Panduan_yujing_error_TimeCNT];																// 算例Flash多维变量
	float Cache_dQ_Cal_Time[Panduan_yujing_error_TimeCNT];																	// 算例Flash多维变量
	uint32_t Cache_dSOC_Cal_Time_Last;																											// 算例Flash多维变量
	uint32_t Cache_dQ_Cal_Time_Last;																												// 算例Flash多维变量
	uint16_t Cache_dSOC_Cycles_CNT;																												// 算例Flash变量
	uint16_t Cache_dQ_Cycles_CNT;																													// 算例Flash变量
}Flash_DATA1;

// 存储于flash的结构体：基于上下电和充放电状态转变进行读写
typedef struct {
	int16_t Cache_panduan_base[MAX_CELL_CNT][MAX_diffocv_CNT][Panduan_base_CNT];				// 算例2Flash多维数组
	int16_t Cache_panduan_yujing_error_Time[MAX_CELL_CNT][Panduan_yujing_error_TimeCNT];		// 算例2Flash多维数组
	uint32_t Cache_UTCtimeOF_panduan_base[MAX_diffocv_CNT][UTC_CNT_PanduanBase];				// 算例2Flash多维数组
	uint8_t Cache_panduan_base_CNT[MAX_diffocv_CNT];																	// 算例2Flash多维数组
}Flash_DATA2;

/*********   终端调用用户自定义函数    *************/
//extern const char* SWD_VERSION;				//  算法版本号

// 初始化函数，只在上电时调用一次；正常返回0，错误返回非0  
extern int initialFunction(Flash_DATA1* Cache_CAL_DATA, Flash_DATA2* Cache_Base_DATA , YJ_MESSAGE* Cal_Suanli);						// 算例初始化函数

// 算例1函数
extern void battsafetyFunction1(YJ_MESSAGE* Cal_Suanli1);																																	// 算例1函数

// 算例2函数
extern void battsafetyFunction2(Flash_DATA1* Cache_CAL_DATA, Flash_DATA2* Cache_Base_DATA, YJ_MESSAGE* Cal_Suanli2);		// 算例2函数

// 算例3函数
extern void battsafetyFunction3(Flash_DATA1* Cache_CAL_DATA, YJ_MESSAGE* Cal_Suanli3);																	// 算例3函数

