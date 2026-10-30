#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace tb4 {
// The DDS type name stayed the same when ROS changed Gid from 24 to 16 bytes.
// Read both layouts without invoking Humble's incompatible ROS deserializer.
inline bool graph_nodes(const std::vector<uint8_t>& data, size_t gid_size,
                        std::vector<std::string>& nodes) {
  nodes.clear();
  if (data.size() < 4 || data[0] != 0 || data[1] > 1) return false;
  size_t position = 4;
  const bool little = data[1] == 1;
  auto skip = [&](size_t size) {
    if (size > data.size() - position) throw std::out_of_range("CDR length");
    position += size;
  };
  auto number = [&]() {
    skip((4 - (position - 4) % 4) % 4);
    skip(4);
    uint32_t value = 0;
    for (size_t i = 0; i < 4; ++i)
      value |= uint32_t(data[position - 4 + i]) << (8 * (little ? i : 3 - i));
    return value;
  };
  auto string = [&]() {
    const auto size = number();
    if (size == 0 || size > 257) throw std::out_of_range("CDR string");
    const auto start = position;
    skip(size);
    if (data[position - 1] != 0) throw std::out_of_range("CDR terminator");
    return std::string(data.begin() + start, data.begin() + position - 1);
  };
  try {
    skip(gid_size);
    const auto count = number();
    if (count > data.size() / 16) return false;
    for (uint32_t i = 0; i < count; ++i) {
      const auto ns = string();
      const auto name = string();
      nodes.push_back((ns == "/" ? "" : ns) + "/" + name);
      for (int sequence = 0; sequence < 2; ++sequence) {
        const auto size = number();
        if (size > (data.size() - position) / gid_size) return false;
        skip(size * gid_size);
      }
    }
    return position == data.size();
  } catch (const std::out_of_range&) {
    return false;
  }
}
}  // namespace tb4
