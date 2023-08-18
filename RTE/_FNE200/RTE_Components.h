
/*
 * Auto generated Run-Time-Environment Component Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'YZT-BOX' 
 * Target:  'YZT-BOX' 
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H


/*
 * Define the Device Header File: 
 */
#define CMSIS_device_header "stm32f4xx.h"

#define RTE_CMSIS_RTOS2                 /* CMSIS-RTOS2 */
        #define RTE_CMSIS_RTOS2_RTX5            /* CMSIS-RTOS2 Keil RTX5 */
#define RTE_Compiler_IO_File            /* Compiler I/O: File */
          #define RTE_Compiler_IO_File_FS         /* Compiler I/O: File (File System) */
#define RTE_DEVICE_FRAMEWORK_CLASSIC
#define RTE_DEVICE_HAL_COMMON
#define RTE_DEVICE_HAL_CORTEX
#define RTE_DEVICE_HAL_DMA
#define RTE_DEVICE_HAL_GPIO
#define RTE_DEVICE_HAL_PWR
#define RTE_DEVICE_HAL_RCC
#define RTE_DEVICE_HAL_SRAM
#define RTE_DEVICE_STARTUP_STM32F4XX    /* Device Startup for STM32F4 */
#define RTE_Drivers_MCI0                /* Driver MCI0 */
#define RTE_Drivers_USBD0               /* Driver USBD0 */
#define RTE_FileSystem_Core             /* File System Core */
#define RTE_FileSystem_LFN              /* File System with Long Filename support */
#define RTE_FileSystem_Release          /* File System Release Version */
#define RTE_FileSystem_Drive_MC_0       /* File System Memory Card Drive 0 */
#define RTE_USB_Core                    /* USB Core */
#define RTE_USB_Core_Release            /* USB Core Release Version */
#define RTE_USB_Device_0                /* USB Device 0 */
#define RTE_USB_Device_MSC_0            /* USB Device MSC instance 0 */

#define RTE_USB_OTG_FS                  1
#define RTE_USB_OTG_FS_DEVICE           1
#define RTE_OTG_FS_VBUS_SENSING_PIN     1

#endif /* RTE_COMPONENTS_H */
