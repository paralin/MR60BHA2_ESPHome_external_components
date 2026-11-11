#include "seeed_mr60bha2.h"
#include "esphome/core/log.h"
#include "esphome/components/api/api_server.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome::seeed_mr60bha2 {

static const char *const TAG = "seeed_mr60bha2";

static uint32_t decode_u32_le(const uint8_t *data) {
  return uint32_t(data[0]) | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
}

static float decode_float_le(const uint8_t *data) {
  uint32_t bits = decode_u32_le(data);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void MR60BHA2Component::setup() {
  char boot[17];
  std::snprintf(boot, sizeof(boot), "%08x%08x", unsigned(esp_random()), unsigned(esp_random()));
  this->boot_id_ = boot;
  sensor::Sensor *sensors[] = {this->breath_rate_sensor_, this->heart_rate_sensor_, this->distance_sensor_,
    this->num_targets_sensor_, this->total_phase_sensor_, this->breath_phase_sensor_, this->heart_phase_sensor_,
    this->target_x_sensor_, this->target_y_sensor_, this->target_doppler_sensor_, this->motion_energy_sensor_};
  for (size_t i = 0; i < this->measurements_.size(); i++) this->measurements_[i].sensor = sensors[i];
  ESP_LOGI(TAG, "Acquisition schema=2 firmware=2.3.0 boot=%s reset=%d", boot, int(esp_reset_reason()));
}

void MR60BHA2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MR60BHA2: bounded TinyFrame parser, acknowledged raw frames, schema 2");
  LOG_BINARY_SENSOR(" ", "Presence", this->has_target_binary_sensor_);
  for (auto &measurement : this->measurements_) LOG_SENSOR(" ", "Measurement", measurement.sensor);
}

void MR60BHA2Component::loop() {
  const uint64_t started = esp_timer_get_time();
  const uint32_t now = millis();
  this->parser_.expire(now);
  uint8_t byte;
  // Cap both bytes and elapsed work so a busy UART cannot monopolize the main loop.
  for (size_t consumed = 0; consumed < 256 && esp_timer_get_time() - started < 2000; consumed++) {
    if (!this->available() || !this->read_byte(&byte)) break;
    this->uart_bytes_++;
    if (!this->parser_.push(byte, now)) continue;
    this->last_frame_at_ = now;
    const auto &frame = this->parser_.frame();
    const uint64_t acquired_us = esp_timer_get_time();
    const bool valid = this->process_frame_(frame, now);
    if (!valid) this->invalid_payloads_++;
    this->sequence_++;
    if (this->queued_ == this->samples_.size()) {
      this->dropped_++;
      this->read_at_ = (this->read_at_ + 1) % this->samples_.size();
      this->queued_--;
    }
    auto &sample = this->samples_[(this->read_at_ + this->queued_) % this->samples_.size()];
    sample = {frame, acquired_us, this->sequence_, valid};
    this->queued_++;
    this->queue_high_water_ = std::max(this->queue_high_water_, this->queued_);
  }
  if (uint32_t(now - this->last_publish_at_) >= 100) {
    this->last_publish_at_ = now;
    this->publish_frames_();
    for (auto &measurement : this->measurements_) {
      if (measurement.valid && uint32_t(now - measurement.updated_at) > 5000) {
        measurement.valid = false;
        if (measurement.sensor) measurement.sensor->publish_state(NAN);
      }
    }
    if (uint32_t(now - this->last_presence_at_) > 5000 && this->has_target_binary_sensor_ &&
        this->has_target_binary_sensor_->has_state()) this->has_target_binary_sensor_->invalidate_state();
  }
  const bool connected = wifi::global_wifi_component->is_connected();
  if (connected && !this->wifi_connected_) this->wifi_reconnects_++;
  this->wifi_connected_ = connected;
  this->max_loop_us_ = std::max(this->max_loop_us_, uint32_t(esp_timer_get_time() - started));
  if (uint32_t(now - this->last_health_at_) >= 5000) {
    this->last_health_at_ = now;
    this->publish_health_(now);
  }
}

void MR60BHA2Component::publish_(size_t channel, float value, uint32_t now) {
  auto &measurement = this->measurements_[channel];
  const bool valid = std::isfinite(value);
  const bool send = valid != measurement.valid || uint32_t(now - measurement.published_at) >= 100;
  // updated_at records acquisition, including samples suppressed in the scalar display.
  if (send && measurement.sensor) {
    measurement.sensor->publish_state(valid ? value : NAN);
    measurement.published_at = now;
  }
  measurement.valid = valid;
  measurement.updated_at = now;
}

void MR60BHA2Component::invalidate_() {
  for (auto &measurement : this->measurements_) {
    if (measurement.sensor) measurement.sensor->publish_state(NAN);
    measurement.valid = false;
  }
  this->previous_phase_ = NAN;
  this->phase_count_ = 0;
  this->phase_at_ = 0;
}

bool MR60BHA2Component::process_frame_(const RadarFrame &frame, uint32_t now) {
  const auto *data = frame.data.data();
  const size_t length = frame.length;
  switch (frame.type) {
    case 0x0F09:
      if (length != 1 && length != 2) return false;
      if (data[0] > 1 || (length == 2 && data[1] != 0)) return false;
      this->last_presence_at_ = now;
      if (this->has_target_binary_sensor_) this->has_target_binary_sensor_->publish_state(data[0] != 0);
      if (data[0] == 0) this->invalidate_();
      return true;
    case 0x0A14:
    case 0x0A15: {
      if (length != 4) return false;
      const float value = decode_float_le(data);
      this->publish_(frame.type == 0x0A14 ? 0 : 1, value > 0 ? value : NAN, now);
      return std::isfinite(value) && value >= 0;
    }
    case 0x0A16: {
      if (length != 8) return false;
      const float distance = decode_float_le(data + 4);
      this->publish_(2, decode_u32_le(data) != 0 && distance > 0 ? distance : NAN, now);
      return std::isfinite(distance) && distance >= 0;
    }
    case 0x0A13: {
      if (length != 12) return false;
      const float total = decode_float_le(data);
      const float breath = decode_float_le(data + 4);
      const float heart = decode_float_le(data + 8);
      this->publish_(4, total, now);
      this->publish_(5, breath, now);
      this->publish_(6, heart, now);
      if (!std::isfinite(total) || uint32_t(now - this->previous_phase_at_) > 1000) {
        this->previous_phase_ = NAN;
        this->phase_count_ = 0;
        this->phase_at_ = 0;
      }
      if (std::isfinite(total) && std::isfinite(this->previous_phase_)) {
        // Retain the v1 display metric. Raw phase units/interpretation remain a vendor contract.
        const float delta = std::remainder(total - this->previous_phase_, 6.28318530718f);
        this->phase_deltas_[this->phase_at_] = delta;
        this->phase_at_ = (this->phase_at_ + 1) % this->phase_deltas_.size();
        this->phase_count_ = std::min(this->phase_count_ + 1, this->phase_deltas_.size());
        float energy = 0;
        for (size_t i = 0; i < this->phase_count_; i++) energy += this->phase_deltas_[i] * this->phase_deltas_[i];
        this->publish_(10, std::sqrt(energy / this->phase_count_), now);
      }
      this->previous_phase_ = total;
      this->previous_phase_at_ = now;
      return std::isfinite(total) && std::isfinite(breath) && std::isfinite(heart);
    }
    case 0x0A04:
    case 0x0A08: {
      if (length < 4) return false;
      const uint32_t count = decode_u32_le(data);
      if (count > (length - 4) / 16 || length != 4 + size_t(count) * 16) return false;
      // Tracked target info is authoritative; raw point clouds remain in the frame stream.
      if (frame.type == 0x0A08) return true;
      this->publish_(3, float(count), now);
      if (count == 0) {
        for (size_t i = 7; i <= 9; i++) this->publish_(i, NAN, now);
        return true;
      }
      const float x = decode_float_le(data + 4);
      const float y = decode_float_le(data + 8);
      const int32_t doppler = int32_t(decode_u32_le(data + 12));
      this->publish_(7, x, now);
      this->publish_(8, y, now);
      this->publish_(9, float(doppler), now);
      return std::isfinite(x) && std::isfinite(y);
    }
    default:
      return false;
  }
}

void MR60BHA2Component::publish_frames_() {
  if (this->queued_ == 0) return;
  std::string batch = "{\"v\":2,\"boot\":\"" + this->boot_id_ + "\",\"dropped\":" + std::to_string(this->dropped_) +
    ",\"sent_us\":" + std::to_string(esp_timer_get_time()) + ",\"frames\":[";
  size_t count = 0;
  while (count < this->queued_ && count < 64) {
    const auto &sample = this->samples_[(this->read_at_ + count) % this->samples_.size()];
    if (batch.size() + size_t(sample.frame.length) * 2 + 100 > 4096) break;
    char fields[100];
    std::snprintf(fields, sizeof(fields), "%s[%u,%llu,%u,%u,%s,\"", count == 0 ? "" : ",",
      unsigned(sample.sequence), static_cast<unsigned long long>(sample.device_us), unsigned(sample.frame.type),
      unsigned(sample.frame.id), sample.valid ? "true" : "false");
    batch += fields;
    static constexpr char HEX[] = "0123456789abcdef";
    for (size_t i = 0; i < sample.frame.length; i++) {
      const uint8_t byte = sample.frame.data[i];
      batch += HEX[byte >> 4];
      batch += HEX[byte & 15];
    }
    batch += "\"]";
    this->last_sent_sequence_ = sample.sequence;
    count++;
  }
  batch += "]}";
  if (this->frames_sensor_) this->frames_sensor_->publish_state(batch);
}

void MR60BHA2Component::acknowledge_frames(const std::string &boot, int sequence) {
  const uint32_t acknowledged = uint32_t(sequence);
  if (boot != this->boot_id_ || int32_t(acknowledged - this->last_sent_sequence_) > 0) return;
  // Keep each sample until the sole acquisition client has consumed its batch.
  while (this->queued_ != 0 && int32_t(acknowledged - this->samples_[this->read_at_].sequence) >= 0) {
    this->read_at_ = (this->read_at_ + 1) % this->samples_.size();
    this->queued_--;
  }
}

void MR60BHA2Component::publish_health_(uint32_t now) {
  char health[768];
  std::snprintf(health, sizeof(health),
    "{\"v\":1,\"boot\":\"%s\",\"uptime_ms\":%u,\"uart_bytes\":%u,\"frames\":%u,\"parse_errors\":%u,"
    "\"parse_timeouts\":%u,\"uninterpreted_payloads\":%u,\"sample_age_ms\":%u,\"queue_high_water\":%u,\"dropped\":%u,"
    "\"max_loop_us\":%u,\"min_heap\":%u,\"wifi_connects\":%u,\"wifi\":%s,\"api\":%s}",
    this->boot_id_.c_str(), unsigned(now), unsigned(this->uart_bytes_), unsigned(this->sequence_),
    unsigned(this->parser_.errors), unsigned(this->parser_.timeouts), unsigned(this->invalid_payloads_),
    unsigned(now - this->last_frame_at_), unsigned(this->queue_high_water_), unsigned(this->dropped_),
    unsigned(this->max_loop_us_), unsigned(esp_get_minimum_free_heap_size()), unsigned(this->wifi_reconnects_),
    this->wifi_connected_ ? "true" : "false", api::global_api_server->is_connected() ? "true" : "false");
  if (this->health_sensor_) this->health_sensor_->publish_state(health);
  ESP_LOGI(TAG, "%s", health);
}

}  // namespace esphome::seeed_mr60bha2
