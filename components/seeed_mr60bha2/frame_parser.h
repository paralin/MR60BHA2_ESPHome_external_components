#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace esphome::seeed_mr60bha2 {

// TinyFrame headers use big endian integers; the radar payload uses little endian.
struct RadarFrame {
  static constexpr size_t MAX_PAYLOAD = 512;
  uint16_t id{0};
  uint16_t type{0};
  uint16_t length{0};
  std::array<uint8_t, MAX_PAYLOAD> data{};
};

class FrameParser {
 public:
  uint32_t errors{0};
  uint32_t timeouts{0};

  bool push(uint8_t byte, uint32_t now) {
    this->expire(now);
    if (this->size_ == 0) {
      if (byte != 1) return false;
      this->started_at_ = now;
    }
    this->bytes_[this->size_++] = byte;
    if (this->size_ < 8) return false;
    if (this->size_ == 8) {
      this->frame_.length = (uint16_t(this->bytes_[3]) << 8) | this->bytes_[4];
      if (checksum(this->bytes_.data(), 7) != byte || this->frame_.length > RadarFrame::MAX_PAYLOAD) {
        this->errors++;
        this->size_ = 0;
        return false;
      }
      this->frame_.id = (uint16_t(this->bytes_[1]) << 8) | this->bytes_[2];
      this->frame_.type = (uint16_t(this->bytes_[5]) << 8) | this->bytes_[6];
      if (this->frame_.length == 0) {
        this->size_ = 0;
        return true;
      }
    }
    if (this->size_ < size_t(this->frame_.length) + 9) return false;
    this->size_ = 0;
    if (checksum(this->bytes_.data() + 8, this->frame_.length) != byte) {
      this->errors++;
      return false;
    }
    std::memcpy(this->frame_.data.data(), this->bytes_.data() + 8, this->frame_.length);
    return true;
  }

  void expire(uint32_t now) {
    if (this->size_ != 0 && uint32_t(now - this->started_at_) > 250) {
      this->size_ = 0;
      this->timeouts++;
    }
  }

  const RadarFrame &frame() const { return this->frame_; }

 private:
  static uint8_t checksum(const uint8_t *data, size_t length) {
    uint8_t result = 0xff;
    for (size_t i = 0; i < length; i++) result ^= data[i];
    return result;
  }

  std::array<uint8_t, RadarFrame::MAX_PAYLOAD + 9> bytes_{};
  size_t size_{0};
  uint32_t started_at_{0};
  RadarFrame frame_{};
};

}  // namespace esphome::seeed_mr60bha2
