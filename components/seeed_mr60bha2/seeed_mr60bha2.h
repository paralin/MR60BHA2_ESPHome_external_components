#pragma once

#include "frame_parser.h"
#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include <array>
#include <string>

namespace esphome::seeed_mr60bha2 {

class MR60BHA2Component : public Component, public uart::UARTDevice {
  SUB_BINARY_SENSOR(has_target);
  SUB_SENSOR(breath_rate);
  SUB_SENSOR(heart_rate);
  SUB_SENSOR(distance);
  SUB_SENSOR(num_targets);
  SUB_SENSOR(total_phase);
  SUB_SENSOR(breath_phase);
  SUB_SENSOR(heart_phase);
  SUB_SENSOR(target_x);
  SUB_SENSOR(target_y);
  SUB_SENSOR(target_doppler);
  SUB_SENSOR(motion_energy);

 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void dump_config() override;
  void loop() override;
  void set_frames(text_sensor::TextSensor *sensor) { this->frames_sensor_ = sensor; }
  void set_health(text_sensor::TextSensor *sensor) { this->health_sensor_ = sensor; }
  void acknowledge_frames(const std::string &boot, int sequence);

 protected:
  struct Measurement {
    sensor::Sensor *sensor{nullptr};
    uint32_t updated_at{0};
    uint32_t published_at{0};
    bool valid{false};
  };
  struct Sample {
    RadarFrame frame;
    uint64_t device_us{0};
    uint32_t sequence{0};
    bool valid{false};
  };
  bool process_frame_(const RadarFrame &frame, uint32_t now);
  void publish_(size_t channel, float value, uint32_t now);
  void invalidate_();
  void publish_frames_();
  void publish_health_(uint32_t now);

  FrameParser parser_;
  std::array<Measurement, 11> measurements_{};
  std::array<Sample, 128> samples_{};
  size_t read_at_{0};
  size_t queued_{0};
  size_t queue_high_water_{0};
  uint32_t sequence_{0};
  uint32_t last_sent_sequence_{0};
  uint32_t dropped_{0};
  uint32_t invalid_payloads_{0};
  uint32_t uart_bytes_{0};
  uint32_t last_frame_at_{0};
  uint32_t last_presence_at_{0};
  uint32_t last_publish_at_{0};
  uint32_t last_health_at_{0};
  uint32_t max_loop_us_{0};
  uint32_t wifi_reconnects_{0};
  bool wifi_connected_{false};
  std::array<float, 32> phase_deltas_{};
  size_t phase_count_{0};
  size_t phase_at_{0};
  float previous_phase_{NAN};
  uint32_t previous_phase_at_{0};
  std::string boot_id_;
  text_sensor::TextSensor *frames_sensor_{nullptr};
  text_sensor::TextSensor *health_sensor_{nullptr};
};

}  // namespace esphome::seeed_mr60bha2
