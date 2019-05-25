#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <underwater_sensor_msgs/LedLight.h>
#include <uwsim/NetSim.h>

namespace uwsim_netstim {

using namespace uwsim;

class TwinbotNetSimTracing : public NetSimTracing {
public:
  TwinbotNetSimTracing();

  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3ConstPacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev,
                   ns3ConstPacketPtr pkt, bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);

  void TxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void PacketDropsUpdated(std::string path, uint32_t oldValue,
                          uint32_t newValue);

  void MacTxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void MacPacketDropsUpdated(std::string path, uint32_t oldValue,
                             uint32_t newValue);

  void MacRx(std::string path, ROSCommsDevicePtr dev, ns3ConstPacketPtr pkt);
  void MacTx(std::string path, ROSCommsDevicePtr dev, ns3ConstPacketPtr pkt);

  void LeaderOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
  void FollowerOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
  void SupportOdomCb(const nav_msgs::Odometry::ConstPtr &msg);

  void Configure();
  void DoRun();

  void ShowPosition(string path, ROSCommsDevicePtr dev, const tf::Vector3 &pos);

  ros::NodeHandle node;
  underwater_sensor_msgs::LedLight ledmsg;
  ros::Publisher leader_pub, follower_pub, support_pub, e0_pub, e1_pub, e2_pub,
      e3_pub;
  ros::Publisher leader_joint_pub, follower_joint_pub;

  ros::Publisher leader_gled_pub, leader_rled_pub, follower_gled_pub,
      follower_rled_pub, support_gled_pub, support_rled_pub;

  // ros::Subscriber leader_sub, follower_sub, support_sub;
  // nav_msgs::Odometry leader_odom, follower_odom, support_odom;

  struct PIDStatus {};
  PIDStatus leader_pid, follower_pid, support_pid;
  int freq;
  void GetLinearVel(const double &diffx, const double &diffy,
                    const double &diffz, double &vx, double &vy, double &vz);

  void GetExplorerLinearVel(const double &diffx, const double &diffy,
                            const double &diffz, double &vx, double &vy,
                            double &vz);
  double GetAngularVel(const double &diff);
  double GetExplorerAngularVel(const double &diff);

  double AngleToRadians(const double &angle);
};
} // namespace uwsim_netstim
