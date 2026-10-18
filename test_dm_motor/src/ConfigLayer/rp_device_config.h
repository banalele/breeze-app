#ifndef __RP_DEVICE_CONFIG_H
#define __RP_DEVICE_CONFIG_H

/* Includes ------------------------------------------------------------------*/



/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* 设备层 --------------------------------------------------------------------*/

/**
 *	@brief	设备工作状态(通用)
 *	@class	device
 */
typedef enum {
	DEV_OFFLINE,
	DEV_ONLINE,
	
} dev_work_state_t;

/**
 * @brief 未初始化：DEV_RESET_NO 初始化完成:DEV_RESET_OK
 *
 */
typedef enum DEV_RESET_STATE
{
	DEV_RESET_NO,
	DEV_RESET_ING,
	DEV_RESET_OK,
} Dev_Reset_State_e;

#endif
