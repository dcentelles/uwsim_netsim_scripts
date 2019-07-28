#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <uwsim_netsim_scripts/HILNetSimTracing.h>

namespace uwsim_netstim {

HILNetSimTracing::HILNetSimTracing() : NetSimTracing() {

  leader_pub =
      node.advertise<geometry_msgs::TwistStamped>("/uwsim/leader/velocity", 1);
  follower_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/follower/velocity", 1);
  support_pub =
      node.advertise<geometry_msgs::TwistStamped>("/uwsim/support/velocity", 1);
  e0_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer0/velocity", 1);
  e1_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer1/velocity", 1);
  e2_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer2/velocity", 1);
  e3_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer3/velocity", 1);

  leader_joint_pub = node.advertise<sensor_msgs::JointState>(
      "/uwsim/leader/joint_state_command", 1);
  follower_joint_pub = node.advertise<sensor_msgs::JointState>(
      "/uwsim/follower/joint_state_command", 1);

  leader_gled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/leader_leds/green", 1);
  leader_rled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/leader_leds/red", 1);

  follower_gled_pub = node.advertise<underwater_sensor_msgs::LedLight>(
      "/follower_leds/green", 1);
  follower_rled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/follower_leds/red", 1);
  support_gled_pub = node.advertise<underwater_sensor_msgs::LedLight>(
      "/support_leds/green", 1);
  support_rled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/support_leds/red", 1);

  ledmsg.duration = ros::Duration(0.3);
}

void HILNetSimTracing::PacketTransmitting(std::string path,
                                          ROSCommsDevicePtr dev,
                                          ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());

  auto mac = dev->GetMac();
  switch (mac) {
  case 1:
    support_rled_pub.publish(ledmsg);
    break;
  case 2:
    leader_rled_pub.publish(ledmsg);
    break;
  case 3:
    follower_rled_pub.publish(ledmsg);
    break;
  }
}

void HILNetSimTracing::PacketDropsUpdated(std::string path, uint32_t oldValue,
                                          uint32_t newValue) {
  Info("[{}] PKTDROPS {}", path, newValue);
}

void HILNetSimTracing::TxFifoUpdated(std::string path, uint32_t oldValue,
                                     uint32_t newValue) {
  Info("[{}] TXFIFO {}", path, newValue);
}

void HILNetSimTracing::MacPacketDropsUpdated(std::string path,
                                             uint32_t oldValue,
                                             uint32_t newValue) {
  Info("[{}] MAC PKTDROPS {}", path, newValue);
}

void HILNetSimTracing::MacTxFifoUpdated(std::string path, uint32_t oldValue,
                                        uint32_t newValue) {
  Info("[{}] MAC TXFIFO {}", path, newValue);
}

void HILNetSimTracing::PacketError(std::string path, ROSCommsDevicePtr dev,
                                   ns3ConstPacketPtr pkt, bool propErr,
                                   bool colErr) {

  NetsimHeader header;
  pkt->PeekHeader(header);
  if (propErr) {
    if (!colErr) {
      Warn("[{}] PERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
           dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
           header.GetPacketSize());
    } else {

      Warn("[{}] MERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
           dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
           header.GetPacketSize());
    }
  } else {

    Warn("[{}] COL -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
         dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
         header.GetPacketSize());
  }
}

void HILNetSimTracing::PacketReceived(std::string path, ROSCommsDevicePtr dev,
                                      ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());

  auto mac = dev->GetMac();
  switch (mac) {
  case 1:
    support_gled_pub.publish(ledmsg);
    break;
  case 2:
    leader_gled_pub.publish(ledmsg);
    break;
  case 3:
    follower_gled_pub.publish(ledmsg);
    break;
  }
}

void HILNetSimTracing::MacRx(std::string path, ROSCommsDevicePtr dev,
                             ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] MAC RX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
       dev->GetMac(), header.GetSize());
}

void HILNetSimTracing::MacTx(std::string path, ROSCommsDevicePtr dev,
                             ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] MAC TX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
       dev->GetMac(), header.GetSize());
}

void HILNetSimTracing::ShowPosition(string path, ROSCommsDevicePtr dev,
                                    const tf::Vector3 &pos) {

  Info("[{}] POS: {} {} {}", dev->GetMac(), pos.getX(), pos.getY(),
       pos.getZ());
}

void HILNetSimTracing::Configure() {
  SetLogName("uwsim_netsim_scripts");

  // The logging is managed by a spdlog (https://github.com/gabime/spdlog)
  // wrapper (https://github.com/dcentelles/cpplogging).
  // By default, all log messages will be prefixed by the script time in seconds
  // trying nanoseconds resolution (using the spdlog::formatter
  // dccomms_ros::NetsimLogFormatter)
  // Uncomment and customize the code below for adding more fields
  // to the log message's prefix
  // (https://github.com/gabime/spdlog/wiki/3.-Custom-formattingges):
  //  SetLogFormatter(std::make_shared<NetsimLogFormatter>("[%D %T.%F] %v"));

  // If you want to avoid showing the relative simulation time use
  // the native spdlog::pattern_formatter instead:
  // SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%D %T.%F]
  // %v"));
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));

  //---------------------------------------------------------------------

  // We recommend the callbacks to be very simple
  // since the ns3 simulation time is stopped during the ns3 callback
  // execution:
  // https://www.nsnam.org/docs/manual/html/realtime.html

  ns3::Config::Connect("/ROSDeviceList/*/PacketError",
                       ns3::MakeCallback(&HILNetSimTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&HILNetSimTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&HILNetSimTracing::PacketTransmitting, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxFifoSize",
      ns3::MakeCallback(&HILNetSimTracing::TxFifoUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxPacketDrops",
      ns3::MakeCallback(&HILNetSimTracing::PacketDropsUpdated, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacTx",
                       ns3::MakeCallback(&HILNetSimTracing::MacTx, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacRx",
                       ns3::MakeCallback(&HILNetSimTracing::MacRx, this));

  ns3::Config::Connect(
      "/NodeList/*/DeviceList/0/Mac/TxFifoSize",
      ns3::MakeCallback(&HILNetSimTracing::MacTxFifoUpdated, this));

  ns3::Config::Connect(
      "/NodeList/*/DeviceList/0/Mac/TxPacketDrops",
      ns3::MakeCallback(&HILNetSimTracing::MacPacketDropsUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/CourseChange",
      ns3::MakeCallback(&HILNetSimTracing::ShowPosition, this));

  freq = 10;
}

void HILNetSimTracing::GetLinearVel(const double &diffx, const double &diffy,
                                    const double &diffz, double &vx, double &vy,
                                    double &vz) {
  double kp = 0.1;
  double kp2 = 0.03;
  vx = diffx * kp;
  vy = diffy * kp;
  vz = diffz * kp;

  double mod = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (mod > 0.5) {
    vx = diffx * kp2;
    vy = diffy * kp2;
    vz = diffz * kp2;
  }
}

double HILNetSimTracing::GetAngularVel(const double &diff) {
  return diff * 0.1;
}

double HILNetSimTracing::GetExplorerAngularVel(const double &diff) {
  return diff * 0.8;
}

double HILNetSimTracing::AngleToRadians(const double &angle) {
  return angle * 2 * PI / 360;
}

void HILNetSimTracing::GetExplorerLinearVel(const double &diffx,
                                            const double &diffy,
                                            const double &diffz, double &vx,
                                            double &vy, double &vz) {
  double kp = 0.4;
  vx = diffx * kp;
  vy = diffy * kp;
  vz = diffz * kp;
}

void HILNetSimTracing::DoRun() {

  std::thread explorersWork([&]() {
    tf::TransformListener listener;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    tf::StampedTransform wMe0, wMe1, wMe2, wMe3, wMpipe, e1Mte1, e2Mte2, e3Mte3;
    tf::Transform e0Me1, e0Me2, e0Me3, wMeorig, eorigMe0target; //, pipeMe;
    geometry_msgs::TwistStamped explorer_msg;
    sensor_msgs::JointState joint_msg;

    tf::Quaternion rotation;
    rotation.setRPY(0, 0, 0);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "erov_comms";
    static_transformStamped.child_frame_id = "te1";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = -1.5;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "erov_comms";
    static_transformStamped.child_frame_id = "te2";
    static_transformStamped.transform.translation.x = -1.5;
    static_transformStamped.transform.translation.y = -1.5;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "erov_comms";
    static_transformStamped.child_frame_id = "te3";
    static_transformStamped.transform.translation.x = -1.5;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_broadcaster.sendTransform(static_transforms);
    ros::Rate rate(10);

    while (1) {
      try {
        listener.lookupTransform("explorer1", "te1", ros::Time(0), e1Mte1);
        listener.lookupTransform("explorer2", "te2", ros::Time(0), e2Mte2);
        listener.lookupTransform("explorer3", "te3", ros::Time(0), e3Mte3);
      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        continue;
      }

      // Get Translation
      auto e1Tte1 = e1Mte1.getOrigin();
      double e1x = e1Tte1.getX(), e1y = e1Tte1.getY(), e1z = e1Tte1.getZ();
      auto e2Tte2 = e2Mte2.getOrigin();
      double e2x = e2Tte2.getX(), e2y = e2Tte2.getY(), e2z = e2Tte2.getZ();
      auto e3Tte3 = e3Mte3.getOrigin();
      double e3x = e3Tte3.getX(), e3y = e3Tte3.getY(), e3z = e3Tte3.getZ();

      // Get Rotation
      tfScalar roll1, pitch1, yaw1;
      auto mat1 = e1Mte1.getBasis();
      mat1.getRPY(roll1, pitch1, yaw1);
      tfScalar roll2, pitch2, yaw2;
      auto mat2 = e2Mte2.getBasis();
      mat2.getRPY(roll2, pitch2, yaw2);
      tfScalar roll3, pitch3, yaw3;
      auto mat3 = e3Mte3.getBasis();
      mat3.getRPY(roll3, pitch3, yaw3);

      double vx, vy, vz;
      tfScalar roll, pitch, yaw;

      GetExplorerLinearVel(e1x, e1y, e1z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll1);
      pitch = GetExplorerAngularVel(pitch1);
      yaw = GetExplorerAngularVel(yaw1);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e1_pub.publish(explorer_msg);

      GetExplorerLinearVel(e2x, e2y, e2z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll2);
      pitch = GetExplorerAngularVel(pitch2);
      yaw = GetExplorerAngularVel(yaw2);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e2_pub.publish(explorer_msg);

      GetExplorerLinearVel(e3x, e3y, e3z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll3);
      pitch = GetExplorerAngularVel(pitch3);
      yaw = GetExplorerAngularVel(yaw3);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e3_pub.publish(explorer_msg);

      leader_joint_pub.publish(joint_msg);
      follower_joint_pub.publish(joint_msg);
      rate.sleep();
    }
  });
  explorersWork.detach();
}

CLASS_LOADER_REGISTER_CLASS(HILNetSimTracing, NetSimTracing)
} // namespace uwsim_netstim
