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

typedef struct _tSecurityParams
{
  uint8_t ioCapability;
  uint8_t mitm_mode;
  uint8_t bonding_mode;
  uint8_t Use_Fixed_Pin;
  uint8_t encryptionKeySizeMin;
  uint8_t encryptionKeySizeMax;
  uint32_t Fixed_Pin;
  uint8_t initiateSecurity;
} tSecurityParams;

typedef struct _tBLEProfileGlobalContext
{
  tSecurityParams bleSecurityParam;
  uint16_t gapServiceHandle;
  uint16_t devNameCharHandle;
  uint16_t appearanceCharHandle;
  uint16_t connectionHandle;
  uint8_t advtServUUIDlen;
  uint8_t advtServUUID[100];
} BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;
  uint8_t SwitchOffGPIO_timer_Id;
  uint8_t DeviceServerFound;
} BleApplicationContext_t;

#define MAX_DEVICES 8
typedef struct {
    uint8_t addr_type;  // アドレスの種類 (パブリック or ランダム)
    tBDAddr bd_addr;    // デバイスの Bluetooth アドレス (6 バイト)
    uint8_t connected;  // 接続状態 (0: 未接続, 1: 接続済み)
} SensorDevice_t;
SensorDevice_t SensorDevices[MAX_DEVICES];  // センサーのリスト
uint8_t num_sensors_found = 0;              // 見つかったデバイスの数
uint8_t current_connection_index = 0;       // 現在接続中のデバイスインデックス


#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define BD_ADDR_SIZE_LOCAL    6

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

tBDAddr SERVER_REMOTE_BDADDR;
uint8_t SERVER_REMOTE_ADDR_TYPE;

P2PC_APP_ConnHandle_Not_evt_t handleNotification;

static BleApplicationContext_t BleApplicationContext;

static void BLE_UserEvtRx(void * pPayload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static void Scan_Request(void);
static void Connect_Request(void);

void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
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
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG,
     CFG_BLE_MAX_ADV_SET_NBR,
     CFG_BLE_MAX_ADV_DATA_LEN,
     CFG_BLE_TX_PATH_COMPENS,
     CFG_BLE_RX_PATH_COMPENS,
     CFG_BLE_CORE_VERSION,
     CFG_BLE_OPTIONS_EXT
    }
  };

  Ble_Tl_Init();
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);
  UTIL_SEQ_RegTask(1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
    Error_Handler();
  }
  else
  {
    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
  }

  Ble_Hci_Gap_Gatt_Init();
  SVCCTL_Init();

  UTIL_SEQ_RegTask(1<<CFG_TASK_START_SCAN_ID, UTIL_SEQ_RFU, Scan_Request);
  UTIL_SEQ_RegTask(1<<CFG_TASK_CONN_DEV_1_ID, UTIL_SEQ_RFU, Connect_Request);

  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

  P2PC_APP_Init();

  UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *pckt)
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  hci_le_connection_complete_event_rp0 * connection_complete_event;
  evt_blecore_aci *blecore_evt;
  hci_le_advertising_report_event_rp0 * le_advertising_event;
  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;
  hci_disconnection_complete_event_rp0 *cc = (void *) event_pckt->data;
  uint8_t result;
  uint8_t event_type, event_data_size;
  int k = 0;
  uint8_t adtype, adlength;

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      {
        handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
        blecore_evt = (evt_blecore_aci*) event_pckt->data;
        switch (blecore_evt->ecode)
        {
          case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
            {
              aci_gap_proc_complete_event_rp0 *gap_evt_proc_complete = (void*) blecore_evt->data;
              if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC && gap_evt_proc_complete->Status == 0x00)
              {
                APP_DBG_MSG("-- GAP GENERAL DISCOVERY PROCEDURE_COMPLETED: %d\n\r", num_sensors_found);
                if (num_sensors_found > 0)
				{
					current_connection_index = 0;
					Connect_To_Next_Sensor();
				}
              }
            }
            break;
          default:
            break;
        }
      }
      break;

    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
      {
        if (cc->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
        {
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
          BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
          APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH SERVER \n\r");
          handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
          handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          P2PC_APP_Notification(&handleNotification);
        }
      }
      break;

    case HCI_LE_META_EVT_CODE:
      {
        meta_evt = (evt_le_meta_event*) event_pckt->data;

        switch (meta_evt->subevent)
        {
          case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        	connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;
			if (connection_complete_event->Status != 0x00) break;

			APP_DBG_MSG("Connected to device %d\r\n", current_connection_index + 1);

			SensorDevices[current_connection_index].connected = 1;
			current_connection_index++;
			if (current_connection_index < num_sensors_found) Connect_To_Next_Sensor();

			BleApplicationContext.BleApplicationContext_legacy.connectionHandle = connection_complete_event->Connection_Handle;
			APP_DBG_MSG("Set connection handle: 0x%04X\n\r", BleApplicationContext.BleApplicationContext_legacy.connectionHandle);

            result = aci_gatt_disc_all_primary_services(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
            if (result != BLE_STATUS_SUCCESS) APP_DBG_MSG("aci_gatt_disc_all_primary_services Failed \r\n\r");
            break;

          case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
            {
            	uint8_t *adv_report_data;

            	le_advertising_event = (hci_le_advertising_report_event_rp0 *) meta_evt->data;
            	event_type = le_advertising_event->Advertising_Report[0].Event_Type;
            	event_data_size = le_advertising_event->Advertising_Report[0].Length_Data;
            	adv_report_data = (uint8_t*)(&le_advertising_event->Advertising_Report[0].Length_Data) + 1;

                uint8_t addr_type = le_advertising_event->Advertising_Report[0].Address_Type;
                uint8_t *addr = le_advertising_event->Advertising_Report[0].Address;

				// すでにリストに存在するかチェック（重複除外）
				if (is_duplicate_devices(addr) || num_sensors_found == MAX_DEVICES) break;

            	k = 0;

				while(k < event_data_size)
				{
				  adlength = adv_report_data[k];
				  adtype = adv_report_data[k + 1];
				  switch (adtype)
				  {
					  case AD_TYPE_COMPLETE_LOCAL_NAME:
					  case AD_TYPE_SHORTENED_LOCAL_NAME:
						  char local_name[32] = {0}; // 最大31バイト (BLEの制約)
						  uint8_t name_length = adlength - 1; // AD Typeの1バイトを除く

						  if (name_length > sizeof(local_name) - 1)
						 {
							 name_length = sizeof(local_name) - 1; // バッファオーバーフロー防止
						 }

						 memcpy(local_name, &adv_report_data[k + 2], name_length); // ADデータをコピー
						 local_name[name_length] = '\0'; // NULL終端

						 if (strcmp(local_name, "XOSS_VOR_S5546") != 0 && strcmp(local_name, "ASSIOMA46734L") != 0) break;
						 // if (strcmp(local_name, "ASSIOMA46734L") != 0) break;

						 // 接続対象のデバイスとして登録
						 SensorDevices[num_sensors_found].addr_type = addr_type;
						 memcpy(SensorDevices[num_sensors_found].bd_addr, addr, 6);
						 SensorDevices[num_sensors_found].connected = 0;

						 APP_DBG_MSG("New device %d found: Addr = %02X:%02X:%02X:%02X:%02X:%02X Name=%s\r\n", num_sensors_found + 1, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], local_name);

						 num_sensors_found++;

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

APP_BLE_ConnStatus_t APP_BLE_Get_Client_Connection_Status(uint16_t Connection_Handle)
{
  if (BleApplicationContext.BleApplicationContext_legacy.connectionHandle == Connection_Handle)
  {
    return BleApplicationContext.Device_Connection_Status;
  }
  return APP_BLE_IDLE;
}

static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;

  uint16_t a_appearance[1] = { BLE_CFG_GAP_APPEARANCE };
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n");

  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_reset command\n");
  }

  uint8_t random_address[6] = {0xC0, 0xFF, 0xEE, 0xAA, 0xBB, 0xCC}; // 任意のランダムアドレス

  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, 6, random_address);
  if (ret != BLE_STATUS_SUCCESS) {
      APP_DBG_MSG("Failed to set random address. Error code: 0x%02X\r\n", ret);
  } else {
      APP_DBG_MSG("Random address set successfully.\r\n");
  }

  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)a_BLE_CfgIrValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n");
  }

  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)a_BLE_CfgErValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n");
  }

  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
  }

  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_init command\n");
  }

  role = 0;
  role |= GAP_CENTRAL_ROLE;

  if (role > 0)
  {
    const char *name = "P2PCLI";

    ret = aci_gap_init(role,
                       CFG_PRIVACY,
                       APPBLE_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);

    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_init command\n");
    }

    ret = aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  ret = aci_gatt_update_char_value(gap_service_handle,
                                   gap_appearance_char_handle,
                                   0,
                                   2,
                                   (uint8_t *)&a_appearance);
  if (ret != BLE_STATUS_SUCCESS)
  {
    BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
  }
  else
  {
    BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
  }

  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  ret = aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
  }

  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;

  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                               CFG_IDENTITY_ADDRESS);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n");
  }

  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_whitelist();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n");
    }
  }
  APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r");
}

static void Scan_Request(void)
{
  tBleStatus result;
  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_CLIENT)
  {
    result = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L, CFG_BLE_ADDRESS_TYPE, 1);
    if (result == BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r");
    }
    else
    {
      APP_DBG_MSG("-- BLE_App_Start_Limited_Disc_Req, Failed \r\n\r");
    }
  }
  return;
}

static void Connect_Request(void)
{
  tBleStatus result;

  APP_DBG_MSG("\r\n\r** CREATE CONNECTION TO SERVER **  \r\n\r");

  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_CLIENT)
  {
    result = aci_gap_create_connection(SCAN_P,
                                       SCAN_L,
                                       SERVER_REMOTE_ADDR_TYPE, SERVER_REMOTE_BDADDR,
                                       CFG_BLE_ADDRESS_TYPE,
                                       CONN_P1,
                                       CONN_P2,
                                       0,
                                       SUPERV_TIMEOUT,
                                       CONN_L1,
                                       CONN_L2);

    if (result == BLE_STATUS_SUCCESS)
    {
      BleApplicationContext.Device_Connection_Status = APP_BLE_LP_CONNECTING;

    }
    else
    {
      BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

    }
  }
  return;
}

void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

static void BLE_UserEvtRx(void * pPayload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t status)
{
  uint32_t task_id_list;
  switch (status)
  {
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

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();
  return;
}

// すでにリストにあるデバイスかどうかを確認
uint8_t is_duplicate_devices(uint8_t *addr) {
    for (uint8_t i = 0; i < num_sensors_found; i++) {
        if (memcmp(SensorDevices[i].bd_addr, addr, 6) == 0) {
            return 1; // 重複あり
        }
    }
    return 0; // 重複なし
}

void Connect_To_Next_Sensor(void)
{
    if (current_connection_index < num_sensors_found)
    {
        SensorDevice_t *sensor = &SensorDevices[current_connection_index];

        APP_DBG_MSG("Connecting to device %d...\r\n", current_connection_index + 1);

        aci_gap_create_connection(
            SCAN_P, SCAN_L,
            sensor->addr_type, sensor->bd_addr,
            CFG_BLE_ADDRESS_TYPE,
            CONN_P1, CONN_P2,
            0, SUPERV_TIMEOUT,
            CONN_L1, CONN_L2
        );
    }
}
