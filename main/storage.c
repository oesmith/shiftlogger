#include "storage.h"

#include <sys/time.h>
#include <time.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define TAG "storage"

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MOSI 40
#define PIN_NUM_MISO 41
#define PIN_NUM_SCLK 39
#define PIN_NUM_CS 45

#define SHUTDOWN_TIMEOUT pdMS_TO_TICKS(500)

static bool has_sdcard = false;
static bool is_recording = false;
static TickType_t last_valid_ts = 0;
static FILE *file = NULL;

void storage_init(void) {
  esp_vfs_fat_sdmmc_mount_config_t mount_config =
      VFS_FAT_MOUNT_DEFAULT_CONFIG();
  const char mount_point[] = MOUNT_POINT;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  spi_bus_config_t bus_cfg = {
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = PIN_NUM_MISO,
      .sclk_io_num = PIN_NUM_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_NUM_CS;
  slot_config.host_id = host.slot;

  sdmmc_card_t *card;

  if (esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config,
                              &card) != ESP_OK) {
    ESP_LOGW(TAG, "SD card missing");
    return;
  }

  has_sdcard = true;
  sdmmc_card_print_info(stdout, card);
}

int millis_since_midnight(struct tm *tm, int usec) {
  return tm->tm_hour * 60 * 60 * 1000 // Hours
         + tm->tm_min * 60 * 1000     // Minutes
         + tm->tm_sec * 1000          // Seconds
         + (usec / 1000);             // Milliseconds
}

void format_millis(int ms, char *out, size_t len) {
  snprintf(out, len, "%02d:%02d:%02d.%02d",
           ms / (60 * 60 * 1000),   // Hours
           (ms / (60 * 1000)) % 60, // Minutes
           (ms / 1000) % 60,        // Seconds
           (ms / 10) % 100);        // Centi-seconds
}

void storage_update(TickType_t ts, bool has_power, uint16_t rpm, float temp_c,
                    uint8_t throttle) {
  TickType_t now_ts = xTaskGetTickCount();

  struct timeval tv;
  gettimeofday(&tv, NULL);

  struct tm tm;
  gmtime_r(&tv.tv_sec, &tm);

  bool is_valid = has_power && rpm > 0;
  if (!is_recording && is_valid) {
    // Start recording
    char filename[64];
    strftime(filename, sizeof(filename), MOUNT_POINT "/log-%Y%m%d-%H%M%S.csv",
             &tm);

    file = fopen(filename, "w");
    if (file == NULL) {
      ESP_LOGE(TAG, "Failed to open log file %s", filename);
      return;
    }

    ESP_LOGI(TAG, "Started logging to %s", filename);

    is_recording = true;

    fprintf(file, "Time,RPM,Throttle,Water temperature\n");
  }

  if (!is_recording) {
    return;
  }

  if (is_valid) {
    last_valid_ts = ts;
  }

  int now_ms = millis_since_midnight(&tm, tv.tv_usec);
  int event_ms = now_ms - (now_ts - ts);

  char hhmmss[16];
  format_millis(event_ms, hhmmss, sizeof(hhmmss));

  fprintf(file, "%s,%d,%d,%.1f\n", hhmmss, rpm, throttle, temp_c);

  if ((ts - last_valid_ts) > SHUTDOWN_TIMEOUT) {
    is_recording = false;
    fclose(file);
    file = NULL;
    ESP_LOGI(TAG, "Stopped logging");
  }
}

bool storage_has_card(void) { return has_sdcard; }

bool storage_is_recording(void) { return is_recording; }
