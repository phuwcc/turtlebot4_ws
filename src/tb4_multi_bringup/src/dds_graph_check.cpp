// Read-only DDS probe: no ROS graph deserialization and no robot commands.
#include "graph_wire.hpp"
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastrtps/rtps/common/SerializedPayload.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <set>
#include <thread>

using namespace eprosima::fastdds::dds;
using eprosima::fastrtps::rtps::SerializedPayload_t;

class RawGraph : public TopicDataType {
 public:
  RawGraph() {
    setName("rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_");
    m_typeSize = 1024 * 1024;
    m_isGetKeyDefined = false;
  }
  bool serialize(void*, SerializedPayload_t*) override { return false; }
  bool deserialize(SerializedPayload_t* payload, void* target) override {
    auto& bytes = *static_cast<std::vector<uint8_t>*>(target);
    bytes.assign(payload->data, payload->data + payload->length);
    return true;
  }
  std::function<uint32_t()> getSerializedSizeProvider(void*) override {
    return []() { return 1024 * 1024; };
  }
  void* createData() override { return new std::vector<uint8_t>; }
  void deleteData(void* value) override { delete static_cast<std::vector<uint8_t>*>(value); }
  bool getKey(void*, eprosima::fastrtps::rtps::InstanceHandle_t*, bool) override {
    return false;
  }
};

int main() {
  const char* domain = std::getenv("ROS_DOMAIN_ID");
  auto* factory = DomainParticipantFactory::get_instance();
  auto* participant = factory->create_participant(
      domain ? std::stoul(domain) : 0, PARTICIPANT_QOS_DEFAULT);
  if (!participant) return 1;
  TypeSupport type(new RawGraph);
  type.register_type(participant);
  auto* topic = participant->create_topic(
      "ros_discovery_info", type.get_type_name(), TOPIC_QOS_DEFAULT);
  auto* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
  DataReaderQos qos = DATAREADER_QOS_DEFAULT;
  qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  qos.history().kind = KEEP_ALL_HISTORY_QOS;
  auto* reader = topic && subscriber ? subscriber->create_datareader(topic, qos) : nullptr;
  if (!reader) {
    participant->delete_contained_entities();
    factory->delete_participant(participant);
    return 1;
  }
  std::vector<uint8_t> bytes;
  std::vector<std::string> nodes, humble_nodes;
  std::set<std::string> incompatible;
  SampleInfo info;
  size_t samples = 0;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(12);
  while (std::chrono::steady_clock::now() < end) {
    while (reader->take_next_sample(&bytes, &info) == ReturnCode_t::RETCODE_OK) {
      if (!info.valid_data) continue;
      ++samples;
      if (!tb4::graph_nodes(bytes, 24, humble_nodes) && tb4::graph_nodes(bytes, 16, nodes))
        incompatible.insert(nodes.begin(), nodes.end());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  participant->delete_contained_entities();
  factory->delete_participant(participant);
  if (!incompatible.empty()) {
    std::cerr << "DDS KHONG TUONG THICH: node dung GID 16 byte; Humble can 24 byte:\n";
    for (const auto& node : incompatible) std::cerr << "  " << node << '\n';
    std::cerr << "Day la nguon loi 'sequence size exceeds remaining buffer'.\n"
              << "Voi Create 3 /_do_not_use: kiem tra firmware trong webserver, dung ban H.x (Humble).\n"
              << "Huong dan: docs/NETWORK.md#loi-sequence-size-exceeds-remaining-buffer\n";
    return 2;
  }
  if (!samples) {
    std::cerr << "Khong nhan duoc DDS graph; chua the xac minh firmware. Kiem tra discovery server/IP.\n";
    return 1;
  }
  std::cout << "DDS graph: khong phat hien dinh dang GID 16 byte trong " << samples << " mau.\n";
  return 0;
}
