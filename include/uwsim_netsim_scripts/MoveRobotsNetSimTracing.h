#include <uwsim/NetSim.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/publisher.h>

namespace uwsim_netstim {

using namespace uwsim;

class MoveRobotsNetSimTracing : public NetSimTracing {
public:
  MoveRobotsNetSimTracing();
  void ShowDistance(bool &lastPosValid, tf::Vector3 &lastPos,
                    bool &peerPosValid, tf::Vector3 &peerPos, string path,
                    ROSCommsDevicePtr dev, const tf::Vector3 &pos);
  void ShowDistanceDev0(string path, ROSCommsDevicePtr dev,
                        const tf::Vector3 &pos);
  void ShowDistanceDev1(string path, ROSCommsDevicePtr dev,
                        const tf::Vector3 &pos);

  void PacketTransmitting(
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt);
  void PacketCollision(
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt);
  void PacketPropError(
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt);
  void PacketReceived(
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt);
  void Configure();
  void DoRun();

  tf::Vector3 dev0Pos, dev1Pos;
  bool dev0PosValid = false, dev1PosValid = false;
  ros::NodeHandle node;
  ros::Publisher bluerov2Pub, bluerov2fPub;
  dccomms::Timer showDistanceTimer;
};
}
