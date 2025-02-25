#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_client_app.h"
#include "stm32_seq.h"
#include "app_ble.h"

typedef enum {
	P2P_START_TIMER_EVT, P2P_STOP_TIMER_EVT, P2P_NOTIFICATION_INFO_RECEIVED_EVT,
} P2P_Client_Opcode_Notification_evt_t;

typedef struct {
	uint8_t *pPayload;
	uint8_t Length;
} P2P_Client_Data_t;

typedef struct {
	P2P_Client_Opcode_Notification_evt_t P2P_Client_Evt_Opcode;
	P2P_Client_Data_t DataTransfered;
} P2P_Client_App_Notification_evt_t;

typedef struct {
	APP_BLE_ConnStatus_t state;
	uint16_t connHandle;
	uint16_t P2PServiceHandle;
	uint16_t P2PServiceEndHandle;
	uint16_t P2PWriteToServerCharHdle;
	uint16_t P2PWriteToServerDescHandle;
	uint16_t P2PNotificationCharHdle;
	uint16_t P2PNotificationDescHandle;
	uint16_t CSC_Handle;  // CSC Measurement の Characteristic Handle
	uint16_t CPM_Handle;  // Cycling Power Measurement の Characteristic Handle
} P2P_ClientContext_t;

/* USER CODE BEGIN PTD */
// 過去のデータを保存する変数
static uint32_t prevWheelRevolutions = 0;
static uint16_t prevWheelEventTime = 0;
static uint8_t prevZeroWheelRevCount = 0;

static uint16_t prevCrankRevolutions = 0;
static uint16_t prevCrankEventTime = 0;
static uint8_t prevZeroCrankRevCount = 0;

#define MIN_TIME_DIFF 50
#define MAX_SPEED_CHANGE_RATIO 4
#define WHEEL_CIRCUMFERENCE 2.130  // 700x28c のホイール円周 (m)
#define WHEEL_ZERO_COUNT 2  // 何連続で0.0を計測したら本当に0.0km/hと見なすか
#define CRANK_ZERO_COUNT 2  // 何連続で0.0を計測したら本当に0.0km/hと見なすか

#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))

static P2P_ClientContext_t aP2PClientContext[BLE_CFG_CLT_MAX_NBR_CB];

static SVCCTL_EvtAckStatus_t Event_Handler(void *Event);

void P2PC_APP_Init(void) {
	uint8_t index = 0;

	for (index = 0; index < BLE_CFG_CLT_MAX_NBR_CB; index++) {
		aP2PClientContext[index].state = APP_BLE_IDLE;
	}

	SVCCTL_RegisterCltHandler(Event_Handler);

	APP_DBG_MSG("-- P2P CLIENT INITIALIZED \n")
	;

	return;
}

void P2PC_APP_Notification(P2PC_APP_ConnHandle_Not_evt_t *pNotification) {
	switch (pNotification->P2P_Evt_Opcode) {
	case PEER_CONN_HANDLE_EVT:
		break;

	case PEER_DISCON_HANDLE_EVT:
		break;

	default:
		break;
	}
	return;
}

static SVCCTL_EvtAckStatus_t Event_Handler(void *Event) {
	SVCCTL_EvtAckStatus_t return_value;
	hci_event_pckt *event_pckt;
	evt_blecore_aci *blecore_evt;

	return_value = SVCCTL_EvtNotAck;
	event_pckt = (hci_event_pckt*) (((hci_uart_pckt*) Event)->data);

	switch (event_pckt->evt) {
	case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE: {
		blecore_evt = (evt_blecore_aci*) event_pckt->data;
		switch (blecore_evt->ecode) {

		case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE: {
			aci_att_read_by_group_type_resp_event_rp0 *pr =
					(void*) blecore_evt->data;
			uint8_t numServ, i, idx;
			uint16_t uuid, handle;

			uint8_t index;
			handle = pr->Connection_Handle;
			index = 0;
			while ((index < BLE_CFG_CLT_MAX_NBR_CB)
					&& (aP2PClientContext[index].state != APP_BLE_IDLE)) {
				APP_BLE_ConnStatus_t status;

				APP_DBG_MSG("Checking Connection Status: Handle = 0x%04X\n", aP2PClientContext[index].connHandle)
				;
				status = APP_BLE_Get_Client_Connection_Status(
						aP2PClientContext[index].connHandle);

				if ((aP2PClientContext[index].state == APP_BLE_CONNECTED_CLIENT)
						&& (status == APP_BLE_IDLE)) {
					/* Handle disconnected */
					aP2PClientContext[index].state = APP_BLE_IDLE;
					aP2PClientContext[index].connHandle = 0xFFFF;
					break;
				}
				index++;
			}

			if (index < BLE_CFG_CLT_MAX_NBR_CB) {
				aP2PClientContext[index].connHandle = handle;

				numServ = (pr->Data_Length) / pr->Attribute_Data_Length;

				APP_DBG_MSG("-- GATT: ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE: Attribute_Data_Length=%d Data_Length=%d index=%d numServ=%d\n\r", pr->Attribute_Data_Length, pr->Data_Length, index, numServ)
				;

				if (pr->Attribute_Data_Length != 6)
					break;

				idx = 4;
				for (i = 0; i < numServ; i++) {
					uuid = UNPACK_2_BYTE_PARAMETER(
							&pr->Attribute_Data_List[idx]);

					APP_DBG_MSG("-- GATT: ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE: Found UUID: 0x%04X\n\r", uuid)
					;

					// 0x1816 -> Cycling Speed and Cadence (CSC) サービス
					// 0x1818 -> Cycling Power Service (CPS)
					if (uuid == 0x1816 || uuid == 0x1818) {
						APP_DBG_MSG("-- GATT : SERVICE_UUID FOUND: uuid=0x%04X\n", uuid)
						;

						aP2PClientContext[index].P2PServiceHandle =
								UNPACK_2_BYTE_PARAMETER(
										&pr->Attribute_Data_List[idx - 4]);
						aP2PClientContext[index].P2PServiceEndHandle =
								UNPACK_2_BYTE_PARAMETER(
										&pr->Attribute_Data_List[idx - 2]);
						aP2PClientContext[index].state =
								APP_BLE_DISCOVER_CHARACS;
					}
					idx += 6;
				}
			}
		}
			break;

		case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE: {

			aci_att_read_by_type_resp_event_rp0 *pr = (void*) blecore_evt->data;
			uint8_t idx;
			uint16_t uuid, handle;
			uint8_t index;

			index = 0;
			while ((index < BLE_CFG_CLT_MAX_NBR_CB)
					&& (aP2PClientContext[index].connHandle
							!= pr->Connection_Handle))
				index++;

			if (index < BLE_CFG_CLT_MAX_NBR_CB) {

				APP_DBG_MSG("-- GATT: ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE: Handle_Value_Pair_Length: %d index: %d\n\r", pr->Handle_Value_Pair_Length, index)
				;

				idx = 5;
				if (pr->Handle_Value_Pair_Length == 7) {
					pr->Data_Length -= 1;
					while (pr->Data_Length > 0) {
						uuid = UNPACK_2_BYTE_PARAMETER(
								&pr->Handle_Value_Pair_Data[idx]);
						handle = UNPACK_2_BYTE_PARAMETER(
								&pr->Handle_Value_Pair_Data[idx - 2]);

						// APP_DBG_MSG("-- GATT : NOTIFICATION_CHAR_UUID - uuid=0x%04x\n", uuid);

						// 0x2A5B -> CSC Measurement (Speed & Cadence)
						// 0x2A63 -> Cycling Power Measurement
						if (uuid == 0x2A5B || uuid == 0x2A63) {
							APP_DBG_MSG("-- GATT : NOTIFICATION_CHAR_UUID FOUND - uuid=0x%04x\n", uuid)
							;

							aP2PClientContext[index].state =
									APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC;
							aP2PClientContext[index].P2PNotificationCharHdle =
									handle;
						}

						if (uuid == 0x2A5B) // CSC Measurement (Speed & Cadence)
								{
							APP_DBG_MSG("-- GATT : Found CSC Measurement Characteristic\n")
							;
							aP2PClientContext[index].CSC_Handle = handle;
						} else if (uuid == 0x2A63)  // Cycling Power Measurement
								{
							APP_DBG_MSG("-- GATT : Found Cycling Power Measurement Characteristic\n")
							;
							aP2PClientContext[index].CPM_Handle = handle;
						}

						pr->Data_Length -= 7;
						idx += 7;
					}
				}
			}
		}
			break;

		case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE: {
			aci_att_find_info_resp_event_rp0 *pr = (void*) blecore_evt->data;

			uint8_t numDesc, idx, i;
			uint16_t uuid, handle;
			uint8_t index;

			index = 0;
			while ((index < BLE_CFG_CLT_MAX_NBR_CB)
					&& (aP2PClientContext[index].connHandle
							!= pr->Connection_Handle))

				index++;

			if (index < BLE_CFG_CLT_MAX_NBR_CB) {

				numDesc = (pr->Event_Data_Length) / 4;
				idx = 0;
				if (pr->Format == UUID_TYPE_16) {
					for (i = 0; i < numDesc; i++) {
						handle = UNPACK_2_BYTE_PARAMETER(
								&pr->Handle_UUID_Pair[idx]);
						uuid = UNPACK_2_BYTE_PARAMETER(
								&pr->Handle_UUID_Pair[idx + 2]);

						if (uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID) {
							APP_DBG_MSG("-- GATT : CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID- index: %d\n", index)
							;

							if (aP2PClientContext[index].state
									== APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC) {

								aP2PClientContext[index].P2PNotificationDescHandle =
										handle;
								aP2PClientContext[index].state =
										APP_BLE_ENABLE_NOTIFICATION_DESC;

								uint8_t enable_notification[] = { 0x01, 0x00 };
								tBleStatus status = aci_gatt_write_char_desc(
										aP2PClientContext[index].connHandle,
										handle, 2, enable_notification);

								if (status == BLE_STATUS_SUCCESS) {
									APP_DBG_MSG("-- GATT: CSC Measurement Notification Enabled!\n\r")
									;
								} else {
									APP_DBG_MSG("-- GATT: Failed to Enable Notification: 0x%02X\n\r", status)
									;
								}
							}
						}
						idx += 4;
					}
				}
			}
		}
			break;

		case ACI_GATT_NOTIFICATION_VSEVT_CODE: {
			aci_gatt_notification_event_rp0 *pr = (void*) blecore_evt->data;
			uint8_t index;

			index = 0;
			while ((index < BLE_CFG_CLT_MAX_NBR_CB)
					&& (aP2PClientContext[index].connHandle
							!= pr->Connection_Handle))
				index++;

			if (index >= BLE_CFG_CLT_MAX_NBR_CB)
				break;

			// APP_DBG_MSG("-- GATT : ACI_GATT_NOTIFICATION_VSEVT_CODE - index: %d, Length: %d\n", index, pr->Attribute_Value_Length);

			// 受信データの Attribute Handle でどのキャラクタリスティックか判別
			if (pr->Attribute_Handle == aP2PClientContext[index].CSC_Handle) // CSC Measurement (Speed & Cadence)
					{
				uint8_t flags = pr->Attribute_Value[0];
				uint8_t offset = 1;

				if (flags & 0x01)  // Bit 0: Wheel Revolution Data Present
						{
					// Cumulative Wheel Revolutions (4バイト)
					uint32_t wheelRevolutions = (pr->Attribute_Value[offset]
							| (pr->Attribute_Value[offset + 1] << 8)
							| (pr->Attribute_Value[offset + 2] << 16)
							| (pr->Attribute_Value[offset + 3] << 24));
					offset += 4;

					// Last Wheel Event Time (2バイト)
					uint16_t wheelEventTime = (pr->Attribute_Value[offset]
							| (pr->Attribute_Value[offset + 1] << 8));
					offset += 2;

					uint32_t wheelRevDiff = wheelRevolutions
							- prevWheelRevolutions;
					uint16_t timeDiff =
							(wheelEventTime >= prevWheelEventTime) ?
									(wheelEventTime - prevWheelEventTime) :
									((65536 + wheelEventTime)
											- prevWheelEventTime); // 16bit のオーバーフロー対応

					float timeSeconds = timeDiff / 1024.0;  // CSC は 1/1024 秒単位
					float speed_mps = (wheelRevDiff * WHEEL_CIRCUMFERENCE)
							/ timeSeconds;
					float speed_kmh = speed_mps * 3.6;

					if (wheelRevDiff == 0) {
						prevZeroWheelRevCount++;
					} else {
						prevZeroWheelRevCount = 0;
					}

					if ((prevWheelRevolutions == 0 && prevWheelEventTime == 0)
							|| prevZeroWheelRevCount > WHEEL_ZERO_COUNT) {
						// APP_DBG_MSG("0 km/h");
						speed_kmh = 0.0;
					} else if (0
							< prevZeroWheelRevCount&& prevZeroWheelRevCount <= WHEEL_ZERO_COUNT) {
						// APP_DBG_MSG("ignore");
						speed_kmh = -1.0;
					}

					if (speed_kmh >= 0) {
						APP_DBG_MSG("Speed: %.2f km/h", speed_kmh)
						;
						//	        			APP_DBG_MSG("Speed: %.2f km/h, wheelRevolution=%u, prevWheelRevolutions=%u, wheelRevDiff=%u, wheelEventTime=%u, prevWheelEventTime=%u, timeDiff=%u, prevZeroWheelRevCount=%d\n",
						//	        					speed_kmh ,wheelRevolutions, prevWheelRevolutions, wheelRevDiff, wheelEventTime, prevWheelEventTime, timeDiff, prevZeroWheelRevCount);
					}

					// 現在のデータを保存
					prevWheelRevolutions = wheelRevolutions;
					prevWheelEventTime = wheelEventTime;
				}
			}

			if (pr->Attribute_Handle == aP2PClientContext[index].CPM_Handle) // Cycling Power Measurement
					{
				// **パワーの取得**
				int16_t power = (int16_t) (pr->Attribute_Value[2]
						| (pr->Attribute_Value[3] << 8));

				// **ケイデンスの取得**
				uint16_t cumulativeCrankRevolutions = pr->Attribute_Value[5]
						| (pr->Attribute_Value[6] << 8);
				uint16_t lastCrankEventTime = pr->Attribute_Value[7]
						| (pr->Attribute_Value[8] << 8);
				uint16_t crankDiff = cumulativeCrankRevolutions
						- prevCrankRevolutions;
				uint16_t timeDiff =
						(lastCrankEventTime >= prevCrankEventTime) ?
								(lastCrankEventTime - prevCrankEventTime) :
								((65536 + lastCrankEventTime)
										- prevCrankEventTime); // 16bit のオーバーフロー対策
				float timeSeconds = timeDiff / 1024.0;  // 1/1024 秒単位
				int16_t cadence = (crankDiff / timeSeconds) * 60; // 回転数/分 (RPM)

				if (crankDiff == 0) {
					prevZeroCrankRevCount++;
				} else {
					prevZeroCrankRevCount = 0;
				}

				if ((prevCrankRevolutions == 0 && prevCrankEventTime == 0)
						|| prevZeroCrankRevCount > CRANK_ZERO_COUNT) {
					cadence = 0;
				} else if (0
						< prevZeroCrankRevCount&& prevZeroCrankRevCount <= CRANK_ZERO_COUNT) {
					cadence = -1;
				}

				if (cadence >= 0) {
					APP_DBG_MSG("Power: %d W, Cadence: %d rpm\n", power, cadence)
					;
				}

//				APP_DBG_MSG("Power: %d W, Cadence: %d rpm, crank: %d, prevCrank: %d, cDiff: %d, time: %d, prevTime: %d, tDiff: %d, seconds: %.4f s\n",
//						power, cadence, cumulativeCrankRevolutions, prevCrankRevolutions, crankDiff, lastCrankEventTime, prevCrankEventTime, timeDiff, timeSeconds);

//				APP_DBG_MSG("Flags: 0x%02X%02X, Power: 0x%02X%02X, PowerBalance: 0x%02X, Crank: 0x%02X%02X, CrankTime: 0x%02X%02X\n",
//						pr->Attribute_Value[0], pr->Attribute_Value[1],
//						pr->Attribute_Value[2], pr->Attribute_Value[3],
//						pr->Attribute_Value[4],
//						pr->Attribute_Value[5], pr->Attribute_Value[6],
//						pr->Attribute_Value[7], pr->Attribute_Value[8]
//				);

				prevCrankRevolutions = cumulativeCrankRevolutions;
				prevCrankEventTime = lastCrankEventTime;
			}
		}
			break;/* end ACI_GATT_NOTIFICATION_VSEVT_CODE */

		case ACI_GATT_PROC_COMPLETE_VSEVT_CODE: {
			aci_gatt_proc_complete_event_rp0 *pr = (void*) blecore_evt->data;
			APP_DBG_MSG("-- GATT : ACI_GATT_PROC_COMPLETE_VSEVT_CODE \n")
			;

			uint8_t index;

			index = 0;
			while ((index < BLE_CFG_CLT_MAX_NBR_CB)
					&& (aP2PClientContext[index].connHandle
							!= pr->Connection_Handle)) {
				index++;
			}

			if (index < BLE_CFG_CLT_MAX_NBR_CB) {
				APP_DBG_MSG("-- GATT : ACI_GATT_PROC_COMPLETE_VSEVT_CODE (Service discovery complete) index: %d\n\r", index)
				;

				if (aP2PClientContext[index].state
						== APP_BLE_DISCOVER_CHARACS) {
					// サービスの探索が終わったら、キャラクタリスティックの探索を開始
					tBleStatus status = aci_gatt_disc_all_char_of_service(
							aP2PClientContext[index].connHandle,
							aP2PClientContext[index].P2PServiceHandle,
							aP2PClientContext[index].P2PServiceEndHandle);

					if (status == BLE_STATUS_SUCCESS) {
						APP_DBG_MSG("-- GATT : Start discovering characteristics\n\r")
						;
					} else {
						APP_DBG_MSG("-- GATT : Failed to start characteristic discovery: 0x%02X\n\r", status)
						;
					}
				}

				if (aP2PClientContext[index].state
						== APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC) {
					APP_DBG_MSG("-- GATT : APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC (Service discovery complete) index: %d\n\r", index)
					;

					tBleStatus status = aci_gatt_disc_all_char_desc(
							aP2PClientContext[index].connHandle,
							aP2PClientContext[index].P2PNotificationCharHdle,
							aP2PClientContext[index].P2PNotificationCharHdle
									+ 2);

					if (status == BLE_STATUS_SUCCESS) {
						APP_DBG_MSG("-- GATT: Start aci_gatt_disc_all_char_desc\n\r")
						;
					} else {
						APP_DBG_MSG("-- GATT: Failed to start aci_gatt_disc_all_char_desc: 0x%02X\n\r", status)
						;
					}
				}
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

	return (return_value);
}

uint8_t P2P_Client_APP_Get_State(void) {
	return aP2PClientContext[0].state;
}
