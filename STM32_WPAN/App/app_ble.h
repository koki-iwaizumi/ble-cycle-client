/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    App/app_ble.h
 * @author  MCD Application Team
 * @brief   Header for ble application
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLE_H
#define APP_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hci_tl.h"
#include "tl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

typedef enum {
	APP_BLE_IDLE,
	APP_BLE_FAST_ADV,
	APP_BLE_LP_ADV,
	APP_BLE_SCAN,
	APP_BLE_LP_CONNECTING,
	APP_BLE_CONNECTED_SERVER,
	APP_BLE_CONNECTED_CLIENT,

	APP_BLE_DISCOVER_SERVICES,
	APP_BLE_DISCOVER_CHARACS,
	APP_BLE_DISCOVER_WRITE_DESC,
	APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC,
	APP_BLE_ENABLE_NOTIFICATION_DESC,
	APP_BLE_DISABLE_NOTIFICATION_DESC
} APP_BLE_ConnStatus_t;

typedef enum {
	BLE_IDLE, BLE_SCANNING, BLE_SCAN, BLE_CONNECTING, BLE_CONNECTED,
} APP_BLE_Status;

typedef enum {
	BLE_GATT_IDLE,
	BLE_GATT_DISCOVER_CHARACS,
	BLE_GATT_DISCOVER_NOTIFICATION_CHAR_DESC,
	BLE_GATTA_ENABLE_NOTIFICATION_DESC
} APP_BLE_GATT_Status;

typedef struct {
	uint8_t addr_type;  // アドレスの種類 (パブリック or ランダム)
	tBDAddr bd_addr;    // デバイスの Bluetooth アドレス (6 バイト)
	APP_BLE_Status status;
	APP_BLE_GATT_Status gatt_status;
	uint16_t connection_handle;
	uint16_t service_handle;
	uint16_t service_end_handle;
	uint16_t profile_handle;
	uint16_t notify_count; // 一定期間notifyした回数
} SensorDevice_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void APP_BLE_Init(void);
APP_BLE_ConnStatus_t APP_BLE_Get_Client_Connection_Status(
		uint16_t Connection_Handle);

/* USER CODE BEGIN EF */
uint8_t is_duplicate_devices(uint8_t *addr);
void Connect_To_Next_Sensor(void);
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*APP_BLE_H */
