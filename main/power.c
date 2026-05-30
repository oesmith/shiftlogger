#include "power.h"
#include "driver/gpio.h"

#define SENSE_V_DIG 8
#define FORCE_ON 17

static bool usb_powered = false;

void power_init(void) {
  gpio_config_t sense_io_conf = {
    .pin_bit_mask = (1ULL << SENSE_V_DIG),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&sense_io_conf);
  // Assume we're USB powered if there's no power sensed at initialisation.
  usb_powered = !gpio_get_level(SENSE_V_DIG);

  gpio_config_t force_on_io_conf = {
    .pin_bit_mask = (1ULL << FORCE_ON),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&force_on_io_conf);
}

bool power_has_power(void) {
  return usb_powered || gpio_get_level(SENSE_V_DIG);
}

void power_force_on(bool force_on) {
  gpio_set_level(FORCE_ON, force_on ? 1 : 0);
}
