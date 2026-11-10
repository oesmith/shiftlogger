#include "esp_log.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include <sys/time.h>
#include <time.h>
// #include "nimble/ble.h"
#include "nimble/nimble_port.h"
// #include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "telemetry"

#define SVC_UUID 0x1ff8
#define MAIN_CHR_UUID 0x1
#define FILTER_CHR_UUID 0x2

static ble_uuid16_t svc_uuid = BLE_UUID16_INIT(SVC_UUID);

static uint8_t main_chr_val[9] = {1, 0, 0, 0, 0, 0, 0, 0, 0};
static uint16_t main_chr_val_handle;

static uint16_t filter_chr_val_handle;

static uint16_t main_chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool conn_handle_inited = false;
static bool notify_status = false;
static uint16_t notify_interval = 0;

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

void ble_store_config_init(void);

static int main_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
static int filter_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);

void start_advertising(void);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SVC_UUID),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID16_DECLARE(MAIN_CHR_UUID),
                    .access_cb = main_chr_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &main_chr_val_handle,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(FILTER_CHR_UUID),
                    .access_cb = filter_chr_access,
                    .flags = BLE_GATT_CHR_F_WRITE,
                    .val_handle = &filter_chr_val_handle,
                },
                {
                    0, // END
                }},
    },
    {
        0, // END
    },
};

static int main_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (attr_handle != main_chr_val_handle) {
    return BLE_ATT_ERR_INVALID_HANDLE;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    return os_mbuf_append(ctxt->om, main_chr_val, sizeof(main_chr_val)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  // Unsupported operation.
  return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

static int filter_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (attr_handle != filter_chr_val_handle) {
    return BLE_ATT_ERR_INVALID_HANDLE;
  }
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    // Unsupported operation.
    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
  }
  uint8_t buf[8] = {0};
  uint16_t actual_size = 0;
  int ret = ble_hs_mbuf_to_flat(ctxt->om, &buf, 8, &actual_size);
  if (ret != 0) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  switch (buf[0]) {
  case 0: // DENY ALL
    notify_interval = 0;
    ESP_LOGI(TAG, "DENY ALL");
    break;
  case 1: // ALLOW ALL
    notify_interval = __builtin_bswap16(*(uint16_t *)(&buf[1]));
    ESP_LOGI(TAG, "ALLOW ALL %u", notify_interval);
    break;
  case 2: // ALLOW
    notify_interval = __builtin_bswap16(*(uint16_t *)(&buf[1]));
    uint32_t pid = __builtin_bswap32(*(uint32_t *)(&buf[3]));
    ESP_LOGI(TAG, "ALLOW %u %u", notify_interval, pid);
    break;
  default:
    break;
  }
  return 0;
}

void reset_sub(void) {
  main_chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  conn_handle_inited = false;
  notify_status = false;
  notify_interval = 0;
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
             evt->subscribe.attr_handle == main_chr_val_handle) {
    main_chr_conn_handle = evt->subscribe.conn_handle;
    conn_handle_inited = true;
    notify_status = evt->subscribe.cur_notify;
  }
  return 0;
}

void start_advertising(void) {
  struct ble_hs_adv_fields adv_fields = {0};
  struct ble_hs_adv_fields rsp_fields = {0};
  struct ble_gap_adv_params adv_params = {0};

  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  const char *name = ble_svc_gap_device_name();
  adv_fields.name = (uint8_t *)name;
  adv_fields.name_len = strlen(name);
  adv_fields.name_is_complete = 1;

  adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  adv_fields.tx_pwr_lvl_is_present = 1;

  adv_fields.appearance = 0x0200; // BLE_GAP_APPEARANCE_GENERIC_TAG
  adv_fields.appearance_is_present = 1;

  adv_fields.le_role = 0x00; // BLE_GAP_LE_ROLE_PERIPHERAL
  adv_fields.le_role_is_present = 1;

  adv_fields.uuids16 = &svc_uuid;
  adv_fields.num_uuids16 = 1;
  adv_fields.uuids16_is_complete = 1;

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

void telemetry_update(uint8_t *data) {
  memcpy(&main_chr_val[4], data, sizeof(main_chr_val) - 4);

  if (!notify_status) {
    return;
  }

  // TODO update interval?

  struct os_mbuf *om =
      ble_hs_mbuf_from_flat(main_chr_val, sizeof(main_chr_val));
  if (ble_gattc_notify_custom(main_chr_conn_handle, main_chr_val_handle, om) !=
      0) {
    ESP_LOGW(TAG, "Notification failed.");
  }
}

bool telemetry_active() {
  return notify_status;
}
