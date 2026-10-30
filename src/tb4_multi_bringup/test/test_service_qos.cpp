#include <cstdlib>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

int main() {
  using namespace eprosima::fastdds::dds;
  unsetenv("ROS_DISCOVERY_SERVER");unsetenv("ROS_SUPER_CLIENT");
  auto factory=DomainParticipantFactory::get_instance();
  if (factory->load_XML_profiles_file(TEST_SERVICE_PROFILE)!=ReturnCode_t::RETCODE_OK)
    throw std::runtime_error("Invalid Fast DDS service XML");
  DomainParticipantQos options;
  auto udp=std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
  udp->interfaceWhiteList.push_back("127.0.0.1");
  options.transport().use_builtin_transports=false;
  options.transport().user_transports.push_back(udp);
  auto participant=factory->create_participant(198,options);
  if (!participant) throw std::runtime_error("Isolated test participant failed");
  auto publisher=participant->create_publisher(PUBLISHER_QOS_DEFAULT);
  auto qos=publisher->get_default_datawriter_qos();
  const auto original=qos.reliability().max_blocking_time;
  if (original.seconds!=0 || original.nanosec!=100000000u)
    throw std::runtime_error("Service profile changed default topic QoS");
  if (publisher->get_datawriter_qos_from_profile("service",qos)!=ReturnCode_t::RETCODE_OK ||
      qos.reliability().max_blocking_time.seconds!=5 ||
      qos.reliability().max_blocking_time.nanosec!=0)
    throw std::runtime_error("Service discovery timeout is not five seconds");
  participant->delete_publisher(publisher);factory->delete_participant(participant);
}
