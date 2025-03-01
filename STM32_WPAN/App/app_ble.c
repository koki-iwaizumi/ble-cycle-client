#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"
#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"
#include "p2p_client_app.h"
#include "hw.h"
#include "stm32wbxx_hal.h"

typedef struct _tSecurityParams {
	uint8_t ioCapability;
	uint8_t mitm_mode;
	uint8_t bonding_mode;
	uint8_t Use_Fixed_Pin;
	uint8_t encryptionKeySizeMin;
	uint8_t encryptionKeySizeMax;
	uint32_t Fixed_Pin;
	uint8_t initiateSecurity;
} tSecurityParams;

typedef struct _tBLEProfileGlobalContext {
	tSecurityParams bleSecurityParam;
	uint16_t gapServiceHandle;
	uint16_t devNameCharHandle;
	uint16_t appearanceCharHandle;
	uint16_t connectionHandle;
	uint8_t advtServUUIDlen;
	uint8_t advtServUUID[100];
} BleGlobalContext_t;

typedef struct {
	BleGlobalContext_t BleApplicationContext_legacy;
	APP_BLE_ConnStatus_t Device_Connection_Status;
	uint8_t SwitchOffGPIO_timer_Id;
	uint8_t DeviceServerFound;
} BleApplicationContext_t;

SensorDevice_t speedSensor = { .addr_type = 1, .bd_addr = { 0xCB, 0x3B, 0x6F,
		0x4C, 0xDE, 0xA5 }, .status = BLE_IDLE };

SensorDevice_t powerMeterSensor = { .addr_type = 1, .bd_addr = { 0xF4, 0x27,
		0x14, 0x35, 0xEA, 0x2F }, .status = BLE_IDLE };

#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define BD_ADDR_SIZE_LOCAL    6

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

tBDAddr SERVER_REMOTE_BDADDR;
uint8_t SERVER_REMOTE_ADDR_TYPE;

P2PC_APP_ConnHandle_Not_evt_t handleNotification;

static BleApplicationContext_t BleApplicationContext;

static void BLE_UserEvtRx(void *pPayload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static void Scan_Request(void);
static void Connect_Request(void);
static void Reconnect_Request(void);
static void Notify_Check(void);

extern TIM_HandleTypeDef htim2; // 使用するタイマー

void APP_BLE_Init(void) {
	SHCI_CmdStatus_t status;
	SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet = { { { 0, 0, 0 } }, /**< Header unused */
	{ 0, /** pBleBufferAddress not used */
	0, /** BleBufferSize not used */
	CFG_BLE_NUM_GATT_ATTRIBUTES,
	CFG_BLE_NUM_GATT_SERVICES,
	CFG_BLE_ATT_VALUE_ARRAY_SIZE,
	CFG_BLE_NUM_LINK,
	CFG_BLE_DATA_LENGTH_EXTENSION,
	CFG_BLE_PREPARE_WRITE_LIST_SIZE,
	CFG_BLE_MBLOCK_COUNT,
	CFG_BLE_MAX_ATT_MTU,
	CFG_BLE_PERIPHERAL_SCA,
	CFG_BLE_CENTRAL_SCA,
	CFG_BLE_LS_SOURCE,
	CFG_BLE_MAX_CONN_EVENT_LENGTH,
	CFG_BLE_HSE_STARTUP_TIME,
	CFG_BLE_VITERBI_MODE,
	CFG_BLE_OPTIONS, 0,
	CFG_BLE_MAX_COC_INITIATOR_NBR,
	CFG_BLE_MIN_TX_POWER,
	CFG_BLE_MAX_TX_POWER,
	CFG_BLE_RX_MODEL_CONFIG,
	CFG_BLE_MAX_ADV_SET_NBR,
	CFG_BLE_MAX_ADV_DATA_LEN,
	CFG_BLE_TX_PATH_COMPENS,
	CFG_BLE_RX_PATH_COMPENS,
	CFG_BLE_CORE_VERSION,
	CFG_BLE_OPTIONS_EXT } };

	Ble_Tl_Init();
	UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);
	UTIL_SEQ_RegTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU,
			hci_user_evt_proc);

	status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
	if (status != SHCI_Success) {
		APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status)
		;
		Error_Handler();
	} else {
		APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r")
		;
	}

	Ble_Hci_Gap_Gatt_Init();
	SVCCTL_Init();

	UTIL_SEQ_RegTask(1 << CFG_TASK_START_SCAN_ID, UTIL_SEQ_RFU, Scan_Request);
	UTIL_SEQ_RegTask(1 << CFG_TASK_Notify_Check_ID, UTIL_SEQ_RFU, Notify_Check);

	BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

	P2PC_APP_Init();

	// UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *pckt) {
	hci_event_pckt *event_pckt;
	evt_le_meta_event *meta_evt;
	hci_le_connection_complete_event_rp0 *connection_complete_event;
	evt_blecore_aci *blecore_evt;
	hci_le_advertising_report_event_rp0 *le_advertising_event;
	event_pckt = (hci_event_pckt*) ((hci_uart_pckt*) pckt)->data;
	hci_disconnection_complete_event_rp0 *cc = (void*) event_pckt->data;
	uint8_t result;
	uint8_t event_type, event_data_size;
	int k = 0;
	uint8_t adtype, adlength;

	switch (event_pckt->evt) {
	case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE: {
		handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
		blecore_evt = (evt_blecore_aci*) event_pckt->data;
		switch (blecore_evt->ecode) {
		case ACI_GAP_PROC_COMPLETE_VSEVT_CODE: {
			aci_gap_proc_complete_event_rp0 *gap_evt_proc_complete =
					(void*) blecore_evt->data;
			if (gap_evt_proc_complete->Procedure_Code
					== GAP_GENERAL_DISCOVERY_PROC
					&& gap_evt_proc_complete->Status == 0x00) {
				APP_DBG_MSG("Scan_Result: speedSensor.status=%d, powerMeterSensor.status=%d\n\r", speedSensor.status, powerMeterSensor.status)
				;

				if (speedSensor.status == BLE_SCANNING)
					speedSensor.status = BLE_IDLE;
				if (powerMeterSensor.status == BLE_SCANNING)
					powerMeterSensor.status = BLE_IDLE;

				Connect_Request();
			}
		}
			break;
		default:
			break;
		}
	}
		break;

	case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
		if (cc->Connection_Handle
				== BleApplicationContext.BleApplicationContext_legacy.connectionHandle) {
			BleApplicationContext.BleApplicationContext_legacy.connectionHandle =
					0;
			BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
			APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH SERVER \n\r")
			;
			handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
			handleNotification.ConnectionHandle =
					BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
		}

		if (cc->Connection_Handle == speedSensor.connection_handle) {
			APP_DBG_MSG("disconnect: speedSensor \n\r")
			;
			speedSensor.status = BLE_IDLE;
			speedSensor.gatt_status = BLE_GATT_IDLE;
			Reconnect_Request();
		} else if (cc->Connection_Handle
				== powerMeterSensor.connection_handle) {
			APP_DBG_MSG("disconnect: powerMeterSensor \n\r")
			;
			powerMeterSensor.status = BLE_IDLE;
			powerMeterSensor.gatt_status = BLE_GATT_IDLE;
			Reconnect_Request();
		}

		break;

	case HCI_LE_META_EVT_CODE: {
		meta_evt = (evt_le_meta_event*) event_pckt->data;

		switch (meta_evt->subevent) {
		case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
			connection_complete_event =
					(hci_le_connection_complete_event_rp0*) meta_evt->data;

			if (connection_complete_event->Status != 0x00)
				break;

			APP_DBG_MSG("Connected to device Peer_Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
					connection_complete_event->Peer_Address[0], connection_complete_event->Peer_Address[1], connection_complete_event->Peer_Address[2],
					connection_complete_event->Peer_Address[3], connection_complete_event->Peer_Address[4], connection_complete_event->Peer_Address[5])

			if (memcmp(speedSensor.bd_addr,
					connection_complete_event->Peer_Address, sizeof(tBDAddr))
					== 0) {
				APP_DBG_MSG("Connected to speedSensor\r\n")
				;
				speedSensor.status = BLE_CONNECTED;
				speedSensor.gatt_status = BLE_GATT_IDLE;
				speedSensor.connection_handle =
						connection_complete_event->Connection_Handle;
			} else if (memcmp(powerMeterSensor.bd_addr,
					connection_complete_event->Peer_Address, sizeof(tBDAddr))
					== 0) {
				APP_DBG_MSG("Connected to powerMeterSensor\r\n")
				;
				powerMeterSensor.status = BLE_CONNECTED;
				powerMeterSensor.gatt_status = BLE_GATT_IDLE;
				powerMeterSensor.connection_handle =
						connection_complete_event->Connection_Handle;
			}

			Connect_Request();

			BleApplicationContext.BleApplicationContext_legacy.connectionHandle =
					connection_complete_event->Connection_Handle;

			result =
					aci_gatt_disc_all_primary_services(
							BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
			if (result == BLE_STATUS_SUCCESS) {
				APP_DBG_MSG("aci_gatt_disc_all_primary_services Success \r\n\r")
				;
			} else {
				APP_DBG_MSG("aci_gatt_disc_all_primary_services Failed : 0x%02X\r\n\r", result)
				;
			}
			;
			break;

		case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE: {
			uint8_t *adv_report_data;

			le_advertising_event =
					(hci_le_advertising_report_event_rp0*) meta_evt->data;
			event_type = le_advertising_event->Advertising_Report[0].Event_Type;
			event_data_size =
					le_advertising_event->Advertising_Report[0].Length_Data;
			adv_report_data =
					(uint8_t*) (&le_advertising_event->Advertising_Report[0].Length_Data)
							+ 1;

			uint8_t addr_type =
					le_advertising_event->Advertising_Report[0].Address_Type;
			uint8_t *addr = le_advertising_event->Advertising_Report[0].Address;

			k = 0;

			while (k < event_data_size) {
				adlength = adv_report_data[k];
				adtype = adv_report_data[k + 1];
				switch (adtype) {
				case AD_TYPE_COMPLETE_LOCAL_NAME:
				case AD_TYPE_SHORTENED_LOCAL_NAME:
					char local_name[32] = { 0 }; // 最大31バイト (BLEの制約)
					uint8_t name_length = adlength - 1; // AD Typeの1バイトを除く

					if (name_length > sizeof(local_name) - 1) {
						name_length = sizeof(local_name) - 1; // バッファオーバーフロー防止
					}

					memcpy(local_name, &adv_report_data[k + 2], name_length); // ADデータをコピー
					local_name[name_length] = '\0'; // NULL終端

					APP_DBG_MSG("New device found: Addr=%02X:%02X:%02X:%02X:%02X:%02X type=%d  Name=%s\r\n", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], addr_type, local_name)
					;

					if (strcmp(local_name, "XOSS_VOR_S5546") == 0) {
						speedSensor.addr_type = addr_type;
						memcpy(speedSensor.bd_addr, addr, 6);
						speedSensor.status = BLE_SCAN;
					} else if (strcmp(local_name, "ASSIOMA46734L") == 0) {
						powerMeterSensor.addr_type = addr_type;
						memcpy(powerMeterSensor.bd_addr, addr, 6);
						powerMeterSensor.status = BLE_SCAN;
					}
					break;

				default:
					break;
				}
				k += adlength + 1;
			}
		}
			break;
		default:
			break;
		}
	}
		break;
	default:
		break;
	}

	return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Client_Connection_Status(
		uint16_t Connection_Handle) {
	if (BleApplicationContext.BleApplicationContext_legacy.connectionHandle
			== Connection_Handle) {
		return BleApplicationContext.Device_Connection_Status;
	}
	return APP_BLE_IDLE;
}

static void Ble_Tl_Init(void) {
	HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

	Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*) &BleCmdBuffer;
	Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
	hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

	return;
}

static void Ble_Hci_Gap_Gatt_Init(void) {
	uint8_t role;
	uint16_t gap_service_handle, gap_dev_name_char_handle,
			gap_appearance_char_handle;

	uint16_t a_appearance[1] = { BLE_CFG_GAP_APPEARANCE };
	tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

	APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n")
	;

	ret = hci_reset();
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: hci_reset command\n")
		;
	}

	uint8_t random_address[6] = { 0xC0, 0xFF, 0xEE, 0xAA, 0xBB, 0xCC }; // 任意のランダムアドレス

	ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, 6,
			random_address);
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("Failed to set random address. Error code: 0x%02X\r\n", ret)
		;
	} else {
		APP_DBG_MSG("Random address set successfully.\r\n")
		;
	}

	ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN,
			(uint8_t*) a_BLE_CfgIrValue);
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n")
		;
	}

	ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN,
			(uint8_t*) a_BLE_CfgErValue);
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n")
		;
	}

	ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n")
		;
	}

	ret = aci_gatt_init();
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: aci_gatt_init command\n")
		;
	}

	role = 0;
	role |= GAP_CENTRAL_ROLE;

	if (role > 0) {
		const char *name = "P2PCLI";

		ret = aci_gap_init(role,
		CFG_PRIVACY,
		APPBLE_GAP_DEVICE_NAME_LENGTH, &gap_service_handle,
				&gap_dev_name_char_handle, &gap_appearance_char_handle);

		if (ret != BLE_STATUS_SUCCESS) {
			APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret)
			;
		} else {
			APP_DBG_MSG("  Success: aci_gap_init command\n")
			;
		}

		ret = aci_gatt_update_char_value(gap_service_handle,
				gap_dev_name_char_handle, 0, strlen(name), (uint8_t*) name);
		if (ret != BLE_STATUS_SUCCESS) {
			BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
		} else {
			BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
		}
	}

	ret = aci_gatt_update_char_value(gap_service_handle,
			gap_appearance_char_handle, 0, 2, (uint8_t*) &a_appearance);
	if (ret != BLE_STATUS_SUCCESS) {
		BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
	} else {
		BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
	}

	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability =
	CFG_IO_CAPABILITY;
	ret =
			aci_gap_set_io_capability(
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n")
		;
	}

	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode =
	CFG_MITM_PROTECTION;
	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin =
	CFG_ENCRYPTION_KEY_SIZE_MIN;
	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax =
	CFG_ENCRYPTION_KEY_SIZE_MAX;
	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin =
	CFG_USED_FIXED_PIN;
	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin =
	CFG_FIXED_PIN;
	BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode =
	CFG_BONDING_MODE;

	ret =
			aci_gap_set_authentication_requirement(
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
					CFG_SC_SUPPORT,
					CFG_KEYPRESS_NOTIFICATION_SUPPORT,
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
					BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
					CFG_IDENTITY_ADDRESS);
	if (ret != BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret)
		;
	} else {
		APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n")
		;
	}

	if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode) {
		ret = aci_gap_configure_whitelist();
		if (ret != BLE_STATUS_SUCCESS) {
			APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret)
			;
		} else {
			APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n")
			;
		}
	}
	APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r")
	;
}

static void Scan_Request(void) {
	APP_DBG_MSG("Scan_Request: speedSensor.status=%d, powerMeterSensor.status=%d\n\r", speedSensor.status, powerMeterSensor.status)
	;
	if ((speedSensor.status != BLE_IDLE && powerMeterSensor.status != BLE_IDLE)
			|| speedSensor.status == BLE_SCANNING
			|| powerMeterSensor.status == BLE_SCANNING)
		return;

	tBleStatus result = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L,
	CFG_BLE_ADDRESS_TYPE, 1);
	if (result == BLE_STATUS_SUCCESS) {
		APP_DBG_MSG(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r")
		;
		if (speedSensor.status == BLE_IDLE)
			speedSensor.status = BLE_SCANNING;
		if (powerMeterSensor.status == BLE_IDLE)
			powerMeterSensor.status = BLE_SCANNING;
	} else {
		APP_DBG_MSG("-- DISCOVERY (SCAN), Failed \r\n\r")
		;
	}
}

static void Connect_Request(void) {
	APP_DBG_MSG("Connect_Request: speedSensor.status=%d, powerMeterSensor.status=%d\n\r", speedSensor.status, powerMeterSensor.status)

	tBleStatus result;

	if (speedSensor.status == BLE_SCAN) {
		APP_DBG_MSG("aci_gap_create_connection start: speedSensor\r\n")
		;
		result = aci_gap_create_connection(
		SCAN_P, SCAN_L, speedSensor.addr_type, speedSensor.bd_addr,
		CFG_BLE_ADDRESS_TYPE,
		CONN_P1, CONN_P2, 0, SUPERV_TIMEOUT,
		CONN_L1, CONN_L2);

		if (result == BLE_STATUS_SUCCESS) {
			APP_DBG_MSG("speedSensor aci_gap_create_connection successfully.\n")
			;
			speedSensor.status = BLE_CONNECTING;
		} else {
			APP_DBG_MSG("speedSensor aci_gap_create_connection Failed: 0x%02X\n",
					result)
			;
			speedSensor.status = BLE_IDLE;
		}
	} else if (powerMeterSensor.status == BLE_SCAN) {
		APP_DBG_MSG("aci_gap_create_connection start: powerMeterSensor\r\n")
		;
		result = aci_gap_create_connection(
		SCAN_P, SCAN_L, powerMeterSensor.addr_type, powerMeterSensor.bd_addr,
		CFG_BLE_ADDRESS_TYPE,
		CONN_P1, CONN_P2, 0, SUPERV_TIMEOUT,
		CONN_L1, CONN_L2);

		if (result == BLE_STATUS_SUCCESS) {
			APP_DBG_MSG(
					"powerMeterSensor aci_gap_create_connection successfully.\n")
			;
			powerMeterSensor.status = BLE_CONNECTING;
		} else {
			APP_DBG_MSG(
					"powerMeterSensor aci_gap_create_connection Failed: 0x%02X\n",
					result)
			;
			powerMeterSensor.status = BLE_IDLE;
		}
	}
}

static void Reconnect_Request(void) {
	APP_DBG_MSG("Reconnect_Request: speedSensor.status=%d, powerMeterSensor.status=%d\n\r", speedSensor.status, powerMeterSensor.status)

	if (speedSensor.status != BLE_IDLE && powerMeterSensor.status != BLE_IDLE)
		return;

	Peer_Entry_t peerList[2];

	peerList[0].Peer_Address_Type = speedSensor.addr_type;
	memcpy(peerList[0].Peer_Address, speedSensor.bd_addr, sizeof(tBDAddr));

	peerList[1].Peer_Address_Type = powerMeterSensor.addr_type;
	memcpy(peerList[1].Peer_Address, powerMeterSensor.bd_addr, sizeof(tBDAddr));

	tBleStatus result = aci_gap_start_auto_connection_establish_proc(
	SCAN_P,  // Scan Interval (10ms〜10.24s) (例: 1.024秒)
			SCAN_L,  // Scan Window (スキャン時間, 例: 0.512秒)
			CFG_BLE_ADDRESS_TYPE,  // 自デバイスのアドレスタイプ (PUBLIC_ADDR or RANDOM_ADDR)
			CONN_P1,  // 最小接続間隔 (例: 30ms)
			CONN_P2,  // 最大接続間隔 (例: 50ms)
			0,       // 接続遅延 (Latency = 0)
			SUPERV_TIMEOUT,  // 監視タイムアウト (例: 5秒)
			CONN_L1,  // 最小CE長
			CONN_L2,  // 最大CE長
			2,  // 接続対象デバイスの数
			peerList      // 接続デバイスリスト
			);

	if (result == BLE_STATUS_SUCCESS) {
		APP_DBG_MSG("Auto connection process started successfully.\n")
		;
	} else {
		APP_DBG_MSG("Failed to start auto connection: 0x%02X\n", result)
		;
	}
}

static void Notify_Check(void) {
	if (speedSensor.gatt_status == BLE_GATTA_ENABLE_NOTIFICATION_DESC
			&& speedSensor.notify_count == 0) {
		APP_DBG_MSG("speedSensor terminate\r\n")
		;
		aci_gap_terminate(speedSensor.connection_handle, 0x13); // 0x13 = Remote User Terminated
	}

	if (powerMeterSensor.gatt_status == BLE_GATTA_ENABLE_NOTIFICATION_DESC
			&& powerMeterSensor.notify_count == 0) {
		APP_DBG_MSG("powerMeterSensor terminate\r\n")
		;
		aci_gap_terminate(powerMeterSensor.connection_handle, 0x13); // 0x13 = Remote User Terminated
	}

	speedSensor.notify_count = 0;
	powerMeterSensor.notify_count = 0;
}

void hci_notify_asynch_evt(void *pdata) {
	UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
	return;
}

void hci_cmd_resp_release(uint32_t flag) {
	UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
	return;
}

void hci_cmd_resp_wait(uint32_t timeout) {
	UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
	return;
}

static void BLE_UserEvtRx(void *pPayload) {
	SVCCTL_UserEvtFlowStatus_t svctl_return_status;
	tHCI_UserEvtRxParam *pParam;

	pParam = (tHCI_UserEvtRxParam*) pPayload;

	svctl_return_status = SVCCTL_UserEvtRx((void*) &(pParam->pckt->evtserial));
	if (svctl_return_status != SVCCTL_UserEvtFlowDisable) {
		pParam->status = HCI_TL_UserEventFlow_Enable;
	} else {
		pParam->status = HCI_TL_UserEventFlow_Disable;
	}

	return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t status) {
	uint32_t task_id_list;
	switch (status) {
	case HCI_TL_CmdBusy:
		task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
		UTIL_SEQ_PauseTask(task_id_list);
		break;

	case HCI_TL_CmdAvailable:
		task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
		UTIL_SEQ_ResumeTask(task_id_list);
		break;

	default:
		break;
	}
	return;
}

void SVCCTL_ResumeUserEventFlow(void) {
	hci_resume_flow();
	return;
}

