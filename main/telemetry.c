#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
//#include "nimble/ble.h"
#include "nimble/nimble_port.h"
//#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "telemetry"

static ble_uuid128_t svc_uuid;
static ble_uuid128_t chr_uuid;

static uint8_t chr_val[9] = {0, 1, 2, 3, 0, 1, 2, 3, 4};
static uint16_t chr_val_handle;

static uint16_t chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool conn_handle_inited = false;
static bool notify_status = false;

static uint8_t own_addr_type;
static uint8_t own_addr[6] = {0};

static uint8_t esp_uri[] = {0x17 /* HTTPS */,
                            '/',
                            '/',
                            'e',
                            's',
                            'p',
                            'r',
                            'e',
                            's',
                            's',
                            'i',
                            'f',
                            '.',
                            'c',
                            'o',
                            'm'};

static bool has_time = false;

void ble_store_config_init(void);

static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);

void start_advertising(void);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &chr_uuid.u,
                    .access_cb = chr_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE,
                    .val_handle = &chr_val_handle,
                },
                {
                    0, // END
                }},
    },
    {
        0, // END
    },
};

static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (attr_handle != chr_val_handle) {
    return BLE_ATT_ERR_INVALID_HANDLE;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    return os_mbuf_append(ctxt->om, chr_val, sizeof(chr_val)) == 0
      ? 0
      : BLE_ATT_ERR_INSUFFICIENT_RES;
  } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (OS_MBUF_PKTLEN(ctxt->om) != 8) {
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (!has_time) {
      has_time = true;
      uint64_t millis = 0;
      uint16_t unused = 0;
      int ret = ble_hs_mbuf_to_flat(ctxt->om, &millis, 8, &unused);
      if (ret != 0) {
        return BLE_ATT_ERR_UNLIKELY;
      }
      struct timeval tv = {
        .tv_sec = millis / 1000,
        .tv_usec = (millis % 1000) * 1000,
      };
      settimeofday(&tv, NULL);
      MODLOG_DFLT(INFO, "Time received %lld (%llx)", millis, millis);
    }
    return 0;
  }
  // Unsupported operation.
  return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

void reset_sub(void) {
  chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  conn_handle_inited = false;
  notify_status = false;
}

int gap_event_handler(struct ble_gap_event *evt, void *arg) {
  if (evt->type == BLE_GAP_EVENT_DISCONNECT ||
      evt->type == BLE_GAP_EVENT_ADV_COMPLETE ||
      (evt->type == BLE_GAP_EVENT_CONNECT && evt->connect.status != 0)) {
    start_advertising();
  } else if (evt->type == BLE_GAP_EVENT_CONNECT && evt->connect.status == 0) {
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(evt->connect.conn_handle, &desc) == 0) {
      ESP_LOGI(TAG, "Connected: interval %d / %d / %d.", desc.conn_itvl,
               desc.conn_latency, desc.supervision_timeout);
    }
  } else if (evt->type == BLE_GAP_EVENT_SUBSCRIBE &&
             evt->subscribe.attr_handle == chr_val_handle) {
    chr_conn_handle = evt->subscribe.conn_handle;
    conn_handle_inited = true;
    notify_status = evt->subscribe.cur_notify;
  }
  return 0;
}

void start_advertising(void) {
  //const char *name;
  struct ble_hs_adv_fields adv_fields = {0};
  struct ble_hs_adv_fields rsp_fields = {0};
  struct ble_gap_adv_params adv_params = {0};

  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  // name = ble_svc_gap_device_name();
  // adv_fields.name = (uint8_t *)name;
  // adv_fields.name_len = strlen(name);
  // adv_fields.name_is_complete = 1;

  adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  adv_fields.tx_pwr_lvl_is_present = 1;

  adv_fields.appearance = 0x0200; // BLE_GAP_APPEARANCE_GENERIC_TAG
  adv_fields.appearance_is_present = 1;

  adv_fields.le_role = 0x00; // BLE_GAP_LE_ROLE_PERIPHERAL
  adv_fields.le_role_is_present = 1;

  adv_fields.uuids128 = &svc_uuid;
  adv_fields.num_uuids128 = 1;
  adv_fields.uuids128_is_complete = 1;

  if (ble_gap_adv_set_fields(&adv_fields) != 0) {
    ESP_LOGE(TAG, "Failed to set advertising data.");
    return;
  }

  rsp_fields.device_addr = own_addr;
  rsp_fields.device_addr_type = own_addr_type;
  rsp_fields.device_addr_is_present = 1;

  rsp_fields.uri = esp_uri;
  rsp_fields.uri_len = sizeof(esp_uri);

  rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
  rsp_fields.adv_itvl_is_present = 1;

  if (ble_gap_adv_rsp_set_fields(&rsp_fields) != 0) {
    ESP_LOGE(TAG, "Failed to set scan response data.");
    return;
  }

  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
  adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

  if (ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                        gap_event_handler, NULL) != 0) {
    ESP_LOGE(TAG, "Failed to start advertising.");
    return;
  }

  ESP_LOGI(TAG, "Started advertising.");
}

void adv_init(void) {
  if (ble_hs_util_ensure_addr(0) != 0) {
    ESP_LOGE(TAG, "Device does not have any available BT address.");
    return;
  }

  if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
    ESP_LOGE(TAG, "Failed to infer address type.");
    return;
  }

  if (ble_hs_id_copy_addr(own_addr_type, own_addr, NULL) != 0) {
    ESP_LOGE(TAG, "Failed to copy device address.");
    return;
  }

  start_advertising();
}

static void on_stack_sync(void) { adv_init(); }

static void nimble_host_task(void *param) {
  /* Task entry log */
  ESP_LOGI(TAG, "nimble host task has been started!");

  /* This function won't return until nimble_port_stop() is executed */
  nimble_port_run();

  /* Clean up at exit */
  vTaskDelete(NULL);
}

void telemetry_init() {
  ble_uuid_any_t uuid;
  ble_uuid_from_str(&uuid, "4155b086-4973-4e88-8364-e381ba328355");
  svc_uuid = uuid.u128;
  ble_uuid_from_str(&uuid, "2a4ca9c6-8d05-41e8-b241-c238ee79a2bf");
  chr_uuid = uuid.u128;

  esp_err_t ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
    return;
  }

  ble_svc_gap_init();

  if (ble_svc_gap_device_name_set("Shiftlogger") != 0) {
    ESP_LOGE(TAG, "Failed to set BLE device name.");
    return;
  }

  ble_svc_gatt_init();

  if (ble_gatts_count_cfg(gatt_svr_svcs) != 0 ||
      ble_gatts_add_svcs(gatt_svr_svcs) != 0) {
    ESP_LOGE(TAG, "Failed to setup BLE service.");
    return;
  }

  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();

  int rc =
      xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
  if (rc != pdPASS) {
    ESP_LOGE(TAG, "failed to create NimBLE host task");
    return;
  }
}

void telemetry_update(uint32_t event_ms, uint8_t* data) {
  if (!notify_status) {
    return;
  }

  uint8_t buf[sizeof(chr_val)] = {0};
  *((uint32_t*)buf) = __builtin_bswap32(event_ms);
  memcpy(&buf[4], data, sizeof(chr_val) - sizeof(event_ms));

  struct os_mbuf *om;
  om = ble_hs_mbuf_from_flat(buf, sizeof(chr_val));
  if (ble_gattc_notify_custom(chr_conn_handle, chr_val_handle, om) != 0) {
    ESP_LOGW(TAG, "Notification failed.");
  }
}
