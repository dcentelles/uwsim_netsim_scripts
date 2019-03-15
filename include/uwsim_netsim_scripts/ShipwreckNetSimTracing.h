#include <ros/publisher.h>
#include <uwsim/NetSim.h>
#include <underwater_sensor_msgs/LedLight.h>

namespace uwsim_netstim {

using namespace uwsim;

class ShipwreckNetSimTracing : public NetSimTracing {
public:
  ShipwreckNetSimTracing();

  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3ConstPacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev, ns3ConstPacketPtr pkt,
                   bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);

  void TxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void PacketDropsUpdated(std::string path, uint32_t oldValue,
                          uint32_t newValue);

  void MacTxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void MacPacketDropsUpdated(std::string path, uint32_t oldValue,
                          uint32_t newValue);

  void MacRx(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);
  void MacTx(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);

  void Configure();
  void DoRun();

  ros::NodeHandle node;
  underwater_sensor_msgs::LedLight ledmsg;
  ros::Publisher bluerov2_gled_pub, buoy_gled_pub, boat_gled_pub,
      bluerov2_rled_pub, buoy_rled_pub, boat_rled_pub;
};
} // namespace uwsim_netstim
