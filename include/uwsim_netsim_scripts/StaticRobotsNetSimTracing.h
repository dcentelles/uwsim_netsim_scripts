#include <ros/publisher.h>
#include <uwsim/NetSim.h>

namespace uwsim_netstim {

using namespace uwsim;

class StaticRobotsNetSimTracing : public NetSimTracing {
public:
  StaticRobotsNetSimTracing();
  void ShowDistance(bool &lastPosValid, tf::Vector3 &lastPos,
                    bool &peerPosValid, tf::Vector3 &peerPos, string path,
                    ROSCommsDevicePtr dev, const tf::Vector3 &pos);
  void ShowDistanceDev0(string path, ROSCommsDevicePtr dev,
                        const tf::Vector3 &pos);
  void ShowDistanceDev1(string path, ROSCommsDevicePtr dev,
                        const tf::Vector3 &pos);

  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3PacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt,
                   bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3PacketPtr pkt);

  void TxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void PacketDropsUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void Configure();
  void DoRun();

  tf::Vector3 dev0Pos, dev1Pos;
  bool dev0PosValid = false, dev1PosValid = false;
  ros::NodeHandle node;
  ros::Publisher bluerov2Pub, bluerov2fPub;
  dccomms::Timer showDistanceTimer;
};
}
