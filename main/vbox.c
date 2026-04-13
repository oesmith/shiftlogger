#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TAG "vbox"

const char* const DEVICE_NAME = "shiftlogger";

const char* const VBOX_NAME = "VBSport 07010985";
esp_bd_addr_t VBOX_ID = {0x0, 0x7, 0x80, 0x76, 0xc2, 0x17};

void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  switch (event) {
    case ESP_SPP_INIT_EVT:
      ESP_LOGI(TAG, "SPP initialized");
      esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 0, DEVICE_NAME);
      return;
    case ESP_SPP_UNINIT_EVT:
      ESP_LOGI(TAG, "SPP deinitialized");
      return;
    case ESP_SPP_DISCOVERY_COMP_EVT:
      ESP_LOGI(TAG, "SPP discovery: %d %d", param->disc_comp.status,
               param->disc_comp.scn_num);
      if (param->disc_comp.status == ESP_SPP_SUCCESS &&
          param->disc_comp.scn_num > 0) {
        esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER,
                        param->disc_comp.scn[0], VBOX_ID);
      }
      return;
    case ESP_SPP_CL_INIT_EVT:
      ESP_LOGI(TAG, "SPP CL init: %d", param->cl_init.status);
      return;
    case ESP_SPP_OPEN_EVT:
      ESP_LOGI(TAG, "SPP open: %d", param->open.status);
      return;
    default:
      ESP_LOGI(TAG, "SPP event: %d", event);
      return;
  }
}

void vbox_init() {
  // Initialize NVS flash -- it is used to store calibration data for the BT
  // hardware.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Release memory used for the BTLE stack, as we're only using BT classic.
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  const uint8_t* addr = esp_bt_dev_get_address();
  if (addr == NULL) {
    ESP_LOGE(TAG, "BT address unavailable.");
    return;
  }
  ESP_LOGI(TAG, "BT address: %02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1],
           addr[2], addr[3], addr[4], addr[5]);

  ESP_ERROR_CHECK(esp_bt_gap_set_device_name(DEVICE_NAME));

  ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                           ESP_BT_GENERAL_DISCOVERABLE));

  esp_spp_register_callback(spp_callback);

  esp_spp_cfg_t spp_cfg = BT_SPP_DEFAULT_CONFIG();
  spp_cfg.mode = ESP_SPP_MODE_CB;
  ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

  ESP_ERROR_CHECK(esp_spp_start_discovery(VBOX_ID));
}