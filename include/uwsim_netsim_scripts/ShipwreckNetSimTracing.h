#include <ros/publisher.h>
#include <uwsim/NetSim.h>
#include <underwater_sensor_msgs/LedLight.h>

namespace uwsim_netstim {

using namespace uwsim;

class ShipwreckNetSimTracing : public NetSimTracing {
public:
  ShipwreckNetSimTracing();

  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3PacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt,
                   bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3PacketPtr pkt);

  void TxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void PacketDropsUpdated(std::string path, uint32_t oldValue,
                          uint32_t newValue);
  void Configure();
  void DoRun();

  ros::NodeHandle node;
  underwater_sensor_msgs::LedLight ledmsg;
  ros::Publisher bluerov2_gled_pub, buoy_gled_pub, boat_gled_pub,
      bluerov2_rled_pub, buoy_rled_pub, boat_rled_pub;
};
} // namespace uwsim_netstim
