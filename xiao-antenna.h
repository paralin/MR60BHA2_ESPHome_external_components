#pragma once

#include "driver/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

// The attached external antenna requires the XIAO's RF switch to be powered
// and selected before Wi-Fi starts. GPIO14 low selects the ceramic antenna.
inline void configure_xiao_external_antenna() {
  gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_3, 0);
  esphome::delay(100);
  gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_14, 1);
  ESP_LOGI("xiao.antenna", "External antenna selected (GPIO3=0, GPIO14=1)");
}
