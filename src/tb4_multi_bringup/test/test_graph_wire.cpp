#include "../src/graph_wire.hpp"
#include <iostream>

std::vector<uint8_t> sample(size_t gid, bool little) {
  std::vector<uint8_t> bytes{0, uint8_t(little ? 1 : 0), 0, 0};
  bytes.resize(4 + gid);
  auto number = [&](uint32_t n) {
    while ((bytes.size() - 4) % 4) bytes.push_back(0);
    for (size_t i = 0; i < 4; ++i)
      bytes.push_back(uint8_t(n >> (8 * (little ? i : 3 - i))));
  };
  auto string = [&](const std::string& s) {
    number(s.size() + 1);
    bytes.insert(bytes.end(), s.begin(), s.end());
    bytes.push_back(0);
  };
  number(1);
  string("/robot2/_do_not_use");
  string("motion_control");
  number(2);
  bytes.resize(bytes.size() + 2 * gid);
  number(1);
  bytes.resize(bytes.size() + gid);
  return bytes;
}

int main() {
  std::vector<std::string> nodes;
  auto require = [](bool condition) {
    if (!condition) throw std::runtime_error("graph wire regression");
  };
  for (size_t gid : {16, 24}) {
    for (bool little : {false, true}) {
      const auto bytes = sample(gid, little);
      require(tb4::graph_nodes(bytes, gid, nodes));
      require(nodes == std::vector<std::string>{"/robot2/_do_not_use/motion_control"});
      require(!tb4::graph_nodes(bytes, gid == 16 ? 24 : 16, nodes));
      for (size_t length = 0; length < bytes.size(); ++length) {
        const std::vector<uint8_t> truncated(bytes.begin(), bytes.begin() + length);
        require(!tb4::graph_nodes(truncated, gid, nodes));
      }
      auto corrupt = bytes;
      corrupt[4 + gid] = 255;
      require(!tb4::graph_nodes(corrupt, gid, nodes));
      auto trailing = bytes;
      trailing.push_back(0);
      require(!tb4::graph_nodes(trailing, gid, nodes));
    }
  }
  std::cout << "24/16-byte GID, both byte orders, truncation and invalid lengths passed\n";
}
