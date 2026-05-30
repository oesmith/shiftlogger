#include "shiftlight.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "led_strip.h"

#define TAG "shiftlight"

// Resjacan
#define STRIP_GPIO 2
// Homebrew
// #define STRIP_GPIO 13

// Change dependent on LED strip supply voltage.
// 5v   = 0x44
// 3.3v = 0x66
#define BRI 0x66

#define RED {.r = BRI, .g = 0, .b = 0}
#define GREEN {.r = 0, .g = BRI, .b = 0}
#define ORANGE {.r = BRI, .g = BRI, .b = 0}
#define BLUE {.r = 0, .g = 0, .b = BRI}

#define COLOUR(c) c.r, c.g, c.b

#define SLOW_FLASH(ts) (((pdTICKS_TO_MS(ts) / 300) % 2) == 0)
#define FAST_FLASH(ts) (((pdTICKS_TO_MS(ts) / 100) % 2) == 0)

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} colour_t;

static led_strip_handle_t led_strip;

static TickType_t last_update = 0;

#define LED_INTERVAL pdMS_TO_TICKS(25)

#define STARTUP_TIME pdMS_TO_TICKS(1000)
#define STARTUP_SWEEP_SPEED pdMS_TO_TICKS(100)

#define WARM_TEMP 75.0f

#define RPM_COUNT 8

// Note: Sigma 125 has soft cut at 6800 RPM / hard cut at 6900 RPM.
const uint16_t RPM_THRESHOLDS_WARM[] = {
    4100, 4400, 4700, 5000, 5300, 5600, 5900, 6200, 6500,
};

// Reduce thresholds by 1500rpm when cold.
const uint16_t RPM_THRESHOLDS_COLD[] = {
    2600, 2900, 3200, 3500, 3800, 4100, 4400, 4700, 5000,
};

const colour_t RPM_COLOURS_WARM[] = {
    GREEN, GREEN, GREEN, ORANGE, ORANGE, RED, RED, RED,
};

const colour_t RPM_COLOURS_COLD[] = {
    BLUE, BLUE, BLUE, BLUE, BLUE, BLUE, BLUE, BLUE,
};

const colour_t unresponsive_colour = RED;
const colour_t engine_off_colour = ORANGE;

void shiftlight_init() {
  led_strip_config_t led_strip_config = {
      .strip_gpio_num = STRIP_GPIO,
      .max_leds = RPM_COUNT,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
  };

  led_strip_rmt_config_t led_strip_rmt_config = {
      .resolution_hz = 10 * 1000 * 1000,  // 10 MHz
      .flags.with_dma = false,
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config,
                                           &led_strip_rmt_config, &led_strip));

  led_strip_clear(led_strip);
}

void shiftlight_update(TickType_t ts, uint16_t rpm, float temp_c) {
  if ((ts - last_update) < LED_INTERVAL) {
    return;
  }
  last_update = ts;

  for (int i = 0; i < RPM_COUNT; i++) {
    led_strip_set_pixel(led_strip, i, 0, 0, 0);
  }

  bool unresponsive = rpm == SHIFTLIGHT_NO_DATA_RPM;
  const uint16_t* thresholds =
      (temp_c >= WARM_TEMP) ? RPM_THRESHOLDS_WARM : RPM_THRESHOLDS_COLD;
  const colour_t* colours =
      (temp_c >= WARM_TEMP) ? RPM_COLOURS_WARM : RPM_COLOURS_COLD;
  if (ts < STARTUP_TIME) {
    for (uint8_t i = 0; i < RPM_COUNT; i++) {
      if (ts > i * STARTUP_SWEEP_SPEED) {
        led_strip_set_pixel(led_strip, i, COLOUR(RPM_COLOURS_WARM[i]));
      }
    }
  } else if (unresponsive) {
    // No response from ECU.
    // Slow red flash first LED.
    if (SLOW_FLASH(ts)) {
      led_strip_set_pixel(led_strip, 0, COLOUR(unresponsive_colour));
    }
  } else if (rpm == 0) {
    // ECU responding but engine not running.
    // Slow orange flash first LED.
    if (SLOW_FLASH(ts)) {
      led_strip_set_pixel(led_strip, 0, COLOUR(engine_off_colour));
    }
  } else if (rpm >= thresholds[RPM_COUNT]) {
    // RPM above upper threshold.
    // Fast flash, all LEDs, single colour.
    if (FAST_FLASH(ts)) {
      for (uint8_t i = 0; i < RPM_COUNT; i++) {
        led_strip_set_pixel(led_strip, i, COLOUR(colours[RPM_COUNT - 1]));
      }
    }
  } else if (rpm >= thresholds[0]) {
    // RPM nonzero, but below upper threshold.
    // Set LEDs according to defined thresholds / colours.
    for (uint8_t i = 0; i < RPM_COUNT; i++) {
      if (rpm >= thresholds[i]) {
        led_strip_set_pixel(led_strip, i, COLOUR(colours[i]));
      }
    }
  }

  led_strip_refresh(led_strip);
}
