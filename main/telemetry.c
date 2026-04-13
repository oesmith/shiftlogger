#include <stdio.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define TAG "telemetry"

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_SCK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_5

void telemetry_init() {
  esp_err_t ret;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024,
  };

  sdmmc_card_t* card;
  const char mount_point[] = MOUNT_POINT;

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.unaligned_multi_block_rw_max_chunk_size = 8;

  spi_bus_config_t bus_cfg = {
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = PIN_NUM_MISO,
      .sclk_io_num = PIN_NUM_SCK,
      .quadhd_io_num = -1,
      .quadwp_io_num = -1,
      .max_transfer_sz = 4000,
  };

  if ((ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA)) !=
      ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
    return;
  }

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_NUM_CS;
  slot_config.host_id = host.slot;

  if ((ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config,
                                     &mount_config, &card)) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount card: %s", esp_err_to_name(ret));
    return;
  }

  sdmmc_card_print_info(stdout, card);

  FILE* f = fopen(MOUNT_POINT "/config.txt", "r");
  if (f == NULL) {
    ESP_LOGE(TAG, "Failed to open config file.");
    return;
  }
  char line[256];
  while (fgets(line, 256, f) != NULL) {
    char* pos = strchr(line, '\n');
    if (pos != NULL) {
      *pos = '\0';
    }
    ESP_LOGI(TAG, "[CFG] %s", line);
  }
  fclose(f);
}