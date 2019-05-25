#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <uwsim_netsim_scripts/TwinbotNetSimTracing.h>

namespace uwsim_netstim {

TwinbotNetSimTracing::TwinbotNetSimTracing() : NetSimTracing() {

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

void TwinbotNetSimTracing::PacketTransmitting(std::string path,
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

void TwinbotNetSimTracing::PacketDropsUpdated(std::string path,
                                              uint32_t oldValue,
                                              uint32_t newValue) {
  Info("[{}] PKTDROPS {}", path, newValue);
}

void TwinbotNetSimTracing::TxFifoUpdated(std::string path, uint32_t oldValue,
                                         uint32_t newValue) {
  Info("[{}] TXFIFO {}", path, newValue);
}

void TwinbotNetSimTracing::MacPacketDropsUpdated(std::string path,
                                                 uint32_t oldValue,
                                                 uint32_t newValue) {
  Info("[{}] MAC PKTDROPS {}", path, newValue);
}

void TwinbotNetSimTracing::MacTxFifoUpdated(std::string path, uint32_t oldValue,
                                            uint32_t newValue) {
  Info("[{}] MAC TXFIFO {}", path, newValue);
}

void TwinbotNetSimTracing::PacketError(std::string path, ROSCommsDevicePtr dev,
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

void TwinbotNetSimTracing::PacketReceived(std::string path,
                                          ROSCommsDevicePtr dev,
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

void TwinbotNetSimTracing::MacRx(std::string path, ROSCommsDevicePtr dev,
                                 ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] MAC RX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
       dev->GetMac(), header.GetSize());
}

void TwinbotNetSimTracing::MacTx(std::string path, ROSCommsDevicePtr dev,
                                 ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] MAC TX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
       dev->GetMac(), header.GetSize());
}

void TwinbotNetSimTracing::ShowPosition(string path, ROSCommsDevicePtr dev,
                                        const tf::Vector3 &pos) {

  Info("[{}] POS: {} {} {}", dev->GetDccommsId(), pos.getX(), pos.getY(),
       pos.getZ());
}

void TwinbotNetSimTracing::Configure() {
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

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketError",
      ns3::MakeCallback(&TwinbotNetSimTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&TwinbotNetSimTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&TwinbotNetSimTracing::PacketTransmitting, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxFifoSize",
      ns3::MakeCallback(&TwinbotNetSimTracing::TxFifoUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxPacketDrops",
      ns3::MakeCallback(&TwinbotNetSimTracing::PacketDropsUpdated, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacTx",
                       ns3::MakeCallback(&TwinbotNetSimTracing::MacTx, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacRx",
                       ns3::MakeCallback(&TwinbotNetSimTracing::MacRx, this));

  ns3::Config::Connect(
      "/NodeList/*/DeviceList/0/Mac/TxFifoSize",
      ns3::MakeCallback(&TwinbotNetSimTracing::MacTxFifoUpdated, this));

  ns3::Config::Connect(
      "/NodeList/*/DeviceList/0/Mac/TxPacketDrops",
      ns3::MakeCallback(&TwinbotNetSimTracing::MacPacketDropsUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/CourseChange",
      ns3::MakeCallback(&TwinbotNetSimTracing::ShowPosition, this));

  freq = 10;
}

void TwinbotNetSimTracing::GetLinearVel(const double &diffx,
                                        const double &diffy,
                                        const double &diffz, double &vx,
                                        double &vy, double &vz) {
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

double TwinbotNetSimTracing::GetAngularVel(const double &diff) {
  return diff * 0.1;
}

double TwinbotNetSimTracing::GetExplorerAngularVel(const double &diff) {
  return diff * 0.8;
}

double TwinbotNetSimTracing::AngleToRadians(const double &angle) {
  return angle * 2 * PI / 360;
}

void TwinbotNetSimTracing::GetExplorerLinearVel(const double &diffx,
                                                const double &diffy,
                                                const double &diffz, double &vx,
                                                double &vy, double &vz) {
  double kp = 0.7;
  vx = diffx * kp;
  vy = diffy * kp;
  vz = diffz * kp;
}

void TwinbotNetSimTracing::DoRun() {

  std::thread explorersWork([&]() {
    tf::TransformListener listener;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    tf::StampedTransform wMe0, wMe1, wMe2, wMe3, wMpipe, e1Mte1, e2Mte2, e3Mte3;
    tf::Transform e0Me1, e0Me2, e0Me3, wMeorig, eorigMe0target; //, pipeMe;
    geometry_msgs::TwistStamped explorer_msg;
    sensor_msgs::JointState joint_msg;

    ros::Time::sleepUntil(ros::Time::now() + ros::Duration(40));
    joint_msg.name.push_back("Elbow");
    joint_msg.position.push_back(0.2);
    joint_msg.name.push_back("Shoulder");
    joint_msg.position.push_back(1.2);
    joint_msg.name.push_back("JawOpening");
    joint_msg.position.push_back(1);

    while (1) {
      try {
        listener.lookupTransform("world", "explorer0", ros::Time(0), wMe0);
        listener.lookupTransform("world", "explorer1", ros::Time(0), wMe1);
        listener.lookupTransform("world", "explorer2", ros::Time(0), wMe2);
        listener.lookupTransform("world", "explorer3", ros::Time(0), wMe3);
        break;
      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        continue;
      }
    }
    auto eOrig = (wMe0.getOrigin() + wMe1.getOrigin() + wMe2.getOrigin() +
                  wMe3.getOrigin()) /
                 4;
    wMeorig.setOrigin(eOrig);
    wMeorig.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "explorers_origin";
    static_transformStamped.transform.translation.x =
        wMeorig.getOrigin().getX();
    static_transformStamped.transform.translation.y =
        wMeorig.getOrigin().getY();
    static_transformStamped.transform.translation.z =
        wMeorig.getOrigin().getZ();
    static_transformStamped.transform.rotation.x = wMeorig.getRotation().x();
    static_transformStamped.transform.rotation.y = wMeorig.getRotation().y();
    static_transformStamped.transform.rotation.z = wMeorig.getRotation().z();
    static_transformStamped.transform.rotation.w = wMeorig.getRotation().w();
    static_transforms.push_back(static_transformStamped);

    auto e0Mw = wMe0.inverse();

    e0Me1 = e0Mw * wMe1;
    e0Me2 = e0Mw * wMe2;
    e0Me3 = e0Mw * wMe3;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "explorer0";
    static_transformStamped.child_frame_id = "te1";
    static_transformStamped.transform.translation.x = e0Me1.getOrigin().getX();
    static_transformStamped.transform.translation.y = e0Me1.getOrigin().getY();
    static_transformStamped.transform.translation.z = e0Me1.getOrigin().getZ();
    static_transformStamped.transform.rotation.x = e0Me1.getRotation().x();
    static_transformStamped.transform.rotation.y = e0Me1.getRotation().y();
    static_transformStamped.transform.rotation.z = e0Me1.getRotation().z();
    static_transformStamped.transform.rotation.w = e0Me1.getRotation().w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "explorer0";
    static_transformStamped.child_frame_id = "te2";
    static_transformStamped.transform.translation.x = e0Me2.getOrigin().getX();
    static_transformStamped.transform.translation.y = e0Me2.getOrigin().getY();
    static_transformStamped.transform.translation.z = e0Me2.getOrigin().getZ();
    static_transformStamped.transform.rotation.x = e0Me2.getRotation().x();
    static_transformStamped.transform.rotation.y = e0Me2.getRotation().y();
    static_transformStamped.transform.rotation.z = e0Me2.getRotation().z();
    static_transformStamped.transform.rotation.w = e0Me2.getRotation().w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "explorer0";
    static_transformStamped.child_frame_id = "te3";
    static_transformStamped.transform.translation.x = e0Me3.getOrigin().getX();
    static_transformStamped.transform.translation.y = e0Me3.getOrigin().getY();
    static_transformStamped.transform.translation.z = e0Me3.getOrigin().getZ();
    static_transformStamped.transform.rotation.x = e0Me3.getRotation().x();
    static_transformStamped.transform.rotation.y = e0Me3.getRotation().y();
    static_transformStamped.transform.rotation.z = e0Me3.getRotation().z();
    static_transformStamped.transform.rotation.w = e0Me3.getRotation().w();
    static_transforms.push_back(static_transformStamped);

    int numWaypointsPerRad = 360;
    int numRads = 3;
    double radInc = 5;
    double rad = 10;
    double angle = 0;
    double angleInc = 360. / numWaypointsPerRad;

    std::vector<tf::Transform> leaderTargetTfs;
    for (int r = 0; r < numRads; r++) {
      for (int i = 0; i < numWaypointsPerRad; i++) {
        tf::Transform eoMte0, rot, trans;
        double radians = AngleToRadians(angle);
        trans.setOrigin(tf::Vector3(0, rad, 0));
        trans.setRotation(tf::createQuaternionFromYaw(0));
        auto quat = tf::createQuaternionFromYaw(-radians).normalize();
        auto quatx = quat.x();
        auto quaty = quat.y();
        auto quatz = quat.z();
        auto quatw = quat.w();
        rot.setRotation(quat);
        rot.setOrigin(tf::Vector3(0, 0, 0));
        eoMte0 = rot * trans; // = explorers origin to explorer leader target
        quat = eoMte0.getRotation();
        quatx = quat.x();
        quaty = quat.y();
        quatz = quat.z();
        quatw = quat.w();
        angle += angleInc;
        leaderTargetTfs.push_back(eoMte0);

        //        static_transformStamped.header.stamp = ros::Time::now();
        //        static_transformStamped.header.frame_id = "explorers_origin";
        //        static_transformStamped.child_frame_id =
        //            "e0_target_" + std::to_string(i) + "_" +
        //            std::to_string(rad);
        //        static_transformStamped.transform.translation.x =
        //            eoMte0.getOrigin().getX();
        //        static_transformStamped.transform.translation.y =
        //            eoMte0.getOrigin().getY();
        //        static_transformStamped.transform.translation.z =
        //            eoMte0.getOrigin().getZ();
        //        static_transformStamped.transform.rotation.x =
        //        eoMte0.getRotation().x();
        //        static_transformStamped.transform.rotation.y =
        //        eoMte0.getRotation().y();
        //        static_transformStamped.transform.rotation.z =
        //        eoMte0.getRotation().z();
        //        static_transformStamped.transform.rotation.w =
        //        eoMte0.getRotation().w();
        //        static_transforms.push_back(static_transformStamped);
      }
      rad -= radInc;
    }
    static_broadcaster.sendTransform(static_transforms);
    double terror = 0.30;
    ros::Rate rate(freq);
    while (1) {
      for (unsigned int i = 0; i < leaderTargetTfs.size(); i++) {
        tf::Transform wMte0, e0Mte0, eorigMte0 = leaderTargetTfs[i];
        while (1) {
          try {
            listener.lookupTransform("world", "explorer0", ros::Time(0), wMe0);
            listener.lookupTransform("explorer1", "te1", ros::Time(0), e1Mte1);
            listener.lookupTransform("explorer2", "te2", ros::Time(0), e2Mte2);
            listener.lookupTransform("explorer3", "te3", ros::Time(0), e3Mte3);
          } catch (tf::TransformException &ex) {
            Warn("TF: {}", ex.what());
            continue;
          }
          wMte0 = wMeorig * eorigMte0;
          e0Mte0 = wMe0.inverse() * wMte0;

          // Get Translation
          auto e0Tte0 = e0Mte0.getOrigin();
          double e0x = e0Tte0.getX(), e0y = e0Tte0.getY(), e0z = e0Tte0.getZ();
          auto e1Tte1 = e1Mte1.getOrigin();
          double e1x = e1Tte1.getX(), e1y = e1Tte1.getY(), e1z = e1Tte1.getZ();
          auto e2Tte2 = e2Mte2.getOrigin();
          double e2x = e2Tte2.getX(), e2y = e2Tte2.getY(), e2z = e2Tte2.getZ();
          auto e3Tte3 = e3Mte3.getOrigin();
          double e3x = e3Tte3.getX(), e3y = e3Tte3.getY(), e3z = e3Tte3.getZ();

          // Get Rotation
          tfScalar roll0, pitch0, yaw0;
          auto mat0 = e0Mte0.getBasis();
          mat0.getRPY(roll0, pitch0, yaw0);
          tfScalar roll1, pitch1, yaw1;
          auto mat1 = e1Mte1.getBasis();
          mat1.getRPY(roll1, pitch1, yaw1);
          tfScalar roll2, pitch2, yaw2;
          auto mat2 = e2Mte2.getBasis();
          mat2.getRPY(roll2, pitch2, yaw2);
          tfScalar roll3, pitch3, yaw3;
          auto mat3 = e3Mte3.getBasis();
          mat3.getRPY(roll3, pitch3, yaw3);

          if (std::abs(e0x) <= terror && std::abs(e0y) <= terror &&
              std::abs(e0z) <= terror && std::abs(e1x) <= terror &&
              std::abs(e1y) <= terror && std::abs(e1z) <= terror &&
              std::abs(e2x) <= terror && std::abs(e2y) <= terror &&
              std::abs(e2z) <= terror && std::abs(e3x) <= terror &&
              std::abs(e3y) <= terror && std::abs(e3z) <= terror) {
            break;
          }

          double vx, vy, vz;
          tfScalar roll, pitch, yaw;

          GetExplorerLinearVel(e0x, e0y, e0z, vx, vy, vz);
          roll = GetExplorerAngularVel(roll0);
          pitch = GetExplorerAngularVel(pitch0);
          yaw = GetExplorerAngularVel(yaw0);

          explorer_msg.twist.linear.x = vx;
          explorer_msg.twist.linear.y = vy;
          explorer_msg.twist.linear.z = vz;
          explorer_msg.twist.angular.x = roll;
          explorer_msg.twist.angular.y = pitch;
          explorer_msg.twist.angular.z = yaw;

          e0_pub.publish(explorer_msg);

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
      }
    }
  });
  explorersWork.detach();

  std::thread iavWork([&]() {
    tf::TransformListener listener;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    ros::Time::sleepUntil(ros::Time::now() + ros::Duration(40));

    // LEADER - FOLLOWER
    double tfx = -0.329943, tfy = -1.38252, tfz = 16.9526;
    double tfroll = -2.06023e-16, tfpich = -5.21364e-16, tfyaw = -0.284755;

    //    followMarker is now at -0.329943, -1.38252, 16.9526
    //    followMarker orientation is -1.38959e-16, -2.43427e-16, -0.141897,
    //    0.989881,
    // ( roll: -2.06023e-16 ; pitch: -5.21364e-16 ; yaw: -0.284755)

    double tlx = 0.507488, tly = 1.34731, tlz = 16.9312;
    double tlroll = 6.17316e-17, tlpich = -9.13321e-17, tlyaw = -0.301682;

    double lxorig = -13.6125, lyorig = -10.7632, lzorig = 0.371301,
           lyaworig = 1.706;

    // followMarker is now at 0.507488, 1.34731, 16.9312
    // followMarker orientation is 2.36531e-17, -4.97857e-17, -0.15027,
    // 0.988645, ( roll: 6.17316e-17 ; pitch: -9.13321e-17 ; yaw: -0.301682

    geometry_msgs::TwistStamped f_msg, l_msg, s_msg;

    ros::Rate rate(freq);

    tf::Transform wMlp, lMlp;
    tf::StampedTransform wMl, wMf, lMfp, wMfp, fMfp, lMf;

    // GET wMlp and wMfp
    wMlp.setOrigin(tf::Vector3(tlx, tly, tlz));
    wMlp.setRotation(tf::createQuaternionFromRPY(0, 0, tlyaw));

    wMfp.setOrigin(tf::Vector3(tfx, tfy, tfz));
    wMfp.setRotation(tf::createQuaternionFromRPY(0, 0, tfyaw));

    tf::Transform lpMfp;
    lpMfp = wMlp.inverse() * wMfp;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "leader";
    static_transformStamped.child_frame_id = "follower_target";
    static_transformStamped.transform.translation.x = lpMfp.getOrigin().getX();
    static_transformStamped.transform.translation.y = lpMfp.getOrigin().getY();
    static_transformStamped.transform.translation.z = lpMfp.getOrigin().getZ();
    static_transformStamped.transform.rotation.x = lpMfp.getRotation().x();
    static_transformStamped.transform.rotation.y = lpMfp.getRotation().y();
    static_transformStamped.transform.rotation.z = lpMfp.getRotation().z();
    static_transformStamped.transform.rotation.w = lpMfp.getRotation().w();
    static_transforms.push_back(static_transformStamped);

    // GET wMl and wMf
    tf::StampedTransform lMs;
    while (1) {
      try {
        listener.waitForTransform("leader", "support", ros::Time(0),
                                  ros::Duration(2));
        listener.lookupTransform("leader", "support", ros::Time(0), lMs);
        break;

      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        continue;
      }
    }

    tf::Transform lMst = lMs;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "leader";
    static_transformStamped.child_frame_id = "support_target";
    static_transformStamped.transform.translation.x = lMst.getOrigin().getX();
    static_transformStamped.transform.translation.y = lMst.getOrigin().getY();
    static_transformStamped.transform.translation.z = lMst.getOrigin().getZ();
    static_transformStamped.transform.rotation.x = lMst.getRotation().x();
    static_transformStamped.transform.rotation.y = lMst.getRotation().y();
    static_transformStamped.transform.rotation.z = lMst.getRotation().z();
    static_transformStamped.transform.rotation.w = lMst.getRotation().w();
    static_transforms.push_back(static_transformStamped);

    static_broadcaster.sendTransform(static_transforms);

    double terror = 0.01;
    bool grapSucceed = false;
    double vx, vy, vz;

    tf::StampedTransform sMst;
    while (ros::ok()) {
      l_msg.twist.linear.x = 0;
      l_msg.twist.linear.y = 0;
      l_msg.twist.linear.z = 0;
      l_msg.twist.angular.x = 0;
      l_msg.twist.angular.y = 0;
      l_msg.twist.angular.z = 0;

      f_msg.twist.linear.x = 0;
      f_msg.twist.linear.y = 0;
      f_msg.twist.linear.z = 0;
      f_msg.twist.angular.x = 0;
      f_msg.twist.angular.y = 0;
      f_msg.twist.angular.z = 0;

      s_msg.twist.linear.x = 0;
      s_msg.twist.linear.y = 0;
      s_msg.twist.linear.z = 0;
      s_msg.twist.angular.x = 0;
      s_msg.twist.angular.y = 0;
      s_msg.twist.angular.z = 0;

      // GET wMl and wMf
      try {
        listener.waitForTransform("world", "leader", ros::Time(0),
                                  ros::Duration(2));
        listener.lookupTransform("world", "leader", ros::Time(0), wMl);

        listener.waitForTransform("world", "follower", ros::Time(0),
                                  ros::Duration(2));
        listener.lookupTransform("world", "follower", ros::Time(0), wMf);

        listener.waitForTransform("follower", "follower_target", ros::Time(0),
                                  ros::Duration(2));
        listener.lookupTransform("follower", "follower_target", ros::Time(0),
                                 fMfp);

        listener.waitForTransform("leader", "follower", ros::Time(0),
                                  ros::Duration(2));
        listener.lookupTransform("leader", "follower", ros::Time(0), lMf);

        listener.waitForTransform("support", "support_target", ros::Time(0),
                                  ros::Duration(2));
        listener.lookupTransform("support", "support_target", ros::Time(0),
                                 sMst);

      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        continue;
      }

      // Compute lMlp and fMfp
      lMlp = wMl.inverse() * wMlp;

      // Get Translation from vMlp and vMfp and sMst
      auto vTlp = lMlp.getOrigin();
      double vTlpX = vTlp.getX(), vTlpY = vTlp.getY(), vTlpZ = vTlp.getZ();

      auto vTfp = fMfp.getOrigin();
      double vTfpX = vTfp.getX(), vTfpY = vTfp.getY(), vTfpZ = vTfp.getZ();

      // Get Rotation from vMlp and vMfp
      tfScalar roll, pitch, yaw;

      auto vRlp = lMlp.getBasis();
      vRlp.getRPY(roll, pitch, yaw);

      double vrx = GetAngularVel(roll);
      double vry = GetAngularVel(pitch);
      double vrz = GetAngularVel(yaw);
      GetLinearVel(vTlpX, vTlpY, vTlpZ, vx, vy, vz);

      l_msg.twist.linear.x = vx;
      l_msg.twist.linear.y = vy;
      l_msg.twist.linear.z = vz;
      l_msg.twist.angular.x = vrx;
      l_msg.twist.angular.y = vry;
      l_msg.twist.angular.z = vrz;

      leader_pub.publish(l_msg);
      if (!grapSucceed) {
        auto vRfp = fMfp.getBasis();
        vRfp.getRPY(roll, pitch, yaw);

        vrx = GetAngularVel(roll);
        vry = GetAngularVel(pitch);
        vrz = GetAngularVel(yaw);
        GetLinearVel(vTfpX, vTfpY, vTfpZ, vx, vy, vz);

        f_msg.twist.linear.x = vx;
        f_msg.twist.linear.y = vy;
        f_msg.twist.linear.z = vz;
        f_msg.twist.angular.x = vrx;
        f_msg.twist.angular.y = vry;
        f_msg.twist.angular.z = vrz;
        follower_pub.publish(f_msg);
      } else {
        tf::Transform lv, wVl, fv;
        tf::Quaternion rotv = tf::createQuaternionFromRPY(vrx, vry, vrz);
        lv.setOrigin(tf::Vector3(vx, vy, vz));
        lv.setRotation(rotv);
        wVl = wMl * lv;
        fv = wMf.inverse() * wVl * lMf;

        auto orig = fv.getOrigin();
        auto rot = fv.getBasis();
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);
        f_msg.twist.linear.x = orig.getX();
        f_msg.twist.linear.y = orig.getY();
        f_msg.twist.linear.z = orig.getZ();
        f_msg.twist.angular.x = roll;
        f_msg.twist.angular.y = pitch;
        f_msg.twist.angular.z = yaw;
        follower_pub.publish(f_msg);
      }

      auto sTst = sMst.getOrigin();
      double sTstX = sTst.getX(), sTstY = sTst.getY(), sTstZ = sTst.getZ();

      auto sRst = lMlp.getBasis();
      sRst.getRPY(roll, pitch, yaw);

      vrx = GetAngularVel(roll);
      vry = GetAngularVel(pitch);
      vrz = GetAngularVel(yaw);
      //      GetLinearVel(sTstX, sTstY, sTstZ, vx, vy, vz);

      //      s_msg.twist.linear.x = vx;
      //      s_msg.twist.linear.y = vy;
      //      s_msg.twist.linear.z = vz;
      s_msg.twist.linear.x = sTstX * 0.7;
      s_msg.twist.linear.y = sTstY * 0.7;
      s_msg.twist.linear.z = sTstZ * 0.7;
      s_msg.twist.angular.x = vrx;
      s_msg.twist.angular.y = vry;
      s_msg.twist.angular.z = vrz;

      support_pub.publish(s_msg);

      if (std::abs(vTlpX) < terror && std::abs(vTlpY) < terror &&
          std::abs(vTlpZ) < terror && std::abs(vTfpX) < terror &&
          std::abs(vTfpY) < terror && std::abs(vTfpZ) < terror) {

        wMlp.setOrigin(tf::Vector3(lxorig, lyorig, lzorig));
        wMlp.setRotation(tf::createQuaternionFromRPY(0, 0, lyaworig));
        grapSucceed = true;
      }
      rate.sleep();
    }
  });
  iavWork.detach();
}

CLASS_LOADER_REGISTER_CLASS(TwinbotNetSimTracing, NetSimTracing)
} // namespace uwsim_netstim
