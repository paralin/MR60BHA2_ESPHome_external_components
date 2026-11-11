#include "../components/seeed_mr60bha2/frame_parser.h"
#include <cassert>
#include <vector>
#include <iostream>

using esphome::seeed_mr60bha2::FrameParser;

static std::vector<uint8_t> frame(uint16_t type, const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> bytes{1, 0x12, 0x34, uint8_t(payload.size() >> 8), uint8_t(payload.size()), uint8_t(type >> 8), uint8_t(type)};
  uint8_t sum = 0xff;
  for (auto byte : bytes) sum ^= byte;
  bytes.push_back(sum);
  if (!payload.empty()) {
    sum = 0xff;
    for (auto byte : payload) { bytes.push_back(byte); sum ^= byte; }
    bytes.push_back(sum);
  }
  return bytes;
}

int main() {
  FrameParser parser;
  const auto valid = frame(0x0A13, std::vector<uint8_t>(12, 0));
  size_t received = 0;
  for (int repeat = 0; repeat < 100; repeat++) {
    for (auto byte : valid) received += parser.push(byte, repeat);
  }
  assert(received == 100);
  assert(parser.frame().id == 0x1234 && parser.frame().type == 0x0A13 && parser.frame().length == 12);
  auto corrupt = valid;
  corrupt.back() ^= 1;
  for (auto byte : corrupt) assert(!parser.push(byte, 100));
  corrupt = valid;
  corrupt[7] ^= 1;
  for (auto byte : corrupt) assert(!parser.push(byte, 100));
  assert(parser.errors == 2);
  auto oversized = frame(0x0A04, std::vector<uint8_t>(513, 0));
  for (auto byte : oversized) assert(!parser.push(byte, 100));
  assert(parser.errors == 3);
  for (size_t i = 0; i < 10; i++) assert(!parser.push(valid[i], 200));
  parser.expire(451);
  assert(parser.timeouts == 1);
  received = 0;
  for (auto byte : valid) received += parser.push(byte, 452);
  assert(received == 1);
  const auto maximum = frame(0x0A04, std::vector<uint8_t>(512, 0x34));
  received = 0;
  for (auto byte : maximum) received += parser.push(byte, 453);
  assert(received == 1 && parser.frame().length == 512);
  parser.push(1, 0xfffffffe);
  parser.expire(300);
  assert(parser.timeouts == 2);
  uint32_t random = 7;
  for (uint32_t i = 0; i < 1000000; i++) {
    random = random * 1664525 + 1013904223;
    parser.push(uint8_t(random >> 24), i);
  }
  parser.expire(1000300);
  received = 0;
  for (auto byte : valid) received += parser.push(byte, 1000301);
  assert(received == 1);
  std::cout << "Frame parser: concatenation, checksums, bounds, timeout, wrap, flood recovery passed\n";
}
