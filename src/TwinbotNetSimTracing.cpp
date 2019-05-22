#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
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

  //  leader_sub = node.subscribe<nav_msgs::Odometry>(
  //      "/uwsim/leader/odom", 1, &TwinbotNetSimTracing::LeaderOdomCb, this);
  //  follower_sub = node.subscribe<nav_msgs::Odometry>(
  //      "/uwsim/follower/odom", 1, &TwinbotNetSimTracing::FollowerOdomCb,
  //      this);
  //  support_sub = node.subscribe<nav_msgs::Odometry>(
  //      "/uwsim/support/odom", 1, &TwinbotNetSimTracing::SupportOdomCb, this);
}

// void TwinbotNetSimTracing::LeaderOdomCb(
//    const nav_msgs::Odometry::ConstPtr &msg) {
//  leader_odom = *msg;
//}
// void TwinbotNetSimTracing::FollowerOdomCb(
//    const nav_msgs::Odometry::ConstPtr &msg) {
//  follower_odom = *msg;
//}
// void TwinbotNetSimTracing::SupportOdomCb(
//    const nav_msgs::Odometry::ConstPtr &msg) {
//  support_odom = *msg;
//}

void TwinbotNetSimTracing::PacketTransmitting(std::string path,
                                              ROSCommsDevicePtr dev,
                                              ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
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

  freq = 10;
}

double TwinbotNetSimTracing::GetLinearVel(const double &diff) {
  return diff * 0.1;
  //  double vel = 0.5;
  //  double v;
  //  if (std::abs(diff) > 2) {
  //    v = vel;
  //    if (diff < 0) {
  //      v = -1 * v;
  //    }
  //  } else {
  //    v = diff * 0.5;
  //  }
}

double TwinbotNetSimTracing::GetAngularVel(const double &diff) {
  return diff * 0.1;
}

void TwinbotNetSimTracing::DoRun() {
  std::thread work([this]() {
    double tfx = 0.568033, tfy = 0.642953, tfz = 18.5175;
    double tfroll = -2.64855e-15, tfpich = -1.8232e-15, tfyaw = -3.10194;

    double tlx = 0.143354, tly = -0.847053, tlz = 18.5122;
    double tlroll = -1.0775e-15, tlpich = -2.94342e-15, tlyaw = 2.59545;

    double lxorig = -13.6125, lyorig = -10.7632, lzorig = 0.371301,
           lyaworig = 0; // 1.706;

    geometry_msgs::TwistStamped f_msg, l_msg, s_msg;
    tf::TransformListener listener;

    ros::Rate rate(freq);

    tf::Transform wMlp, lMlp;
    tf::StampedTransform wMl, wMf, lMfp, wMfp, fMfp;

    // GET wMlp and wMfp
    wMlp.setOrigin(tf::Vector3(tlx, tly, tlz));
    wMlp.setRotation(tf::createQuaternionFromRPY(0, 0, tlyaw));

    wMfp.setOrigin(tf::Vector3(tfx, tfy, tfz));
    wMfp.setRotation(tf::createQuaternionFromRPY(0, 0, tfyaw));

    tf::Transform lpMfp;
    lpMfp = wMlp.inverse() * wMfp;

    //    tf::Vector3 lpTlf = wMlp.getOrigin() - wMfp.getOrigin();
    //    // tf::Vector3 lpTlf2 = wMfp.getOrigin() - wMlp.getOrigin();
    //    tf::Quaternion lpRlf = wMlp.getRotation() - wMfp.getRotation();
    // lpRlf = lpRlf.getIdentity();
    //   lpRlf.normalize();

    // tf::Transform NRot;
    // tmp.setRotation(wMlp.getRotation());

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
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

    //    static_transformStamped.header.frame_id = "world";
    //    static_transformStamped.child_frame_id = "follower_target";
    //    static_transformStamped.transform.translation.x = tfx;
    //    static_transformStamped.transform.translation.y = tfy;
    //    static_transformStamped.transform.translation.z = tfz;
    //    auto quat = tf::createQuaternionFromRPY(tfroll, tfpich, tfyaw);
    //    static_transformStamped.transform.rotation.x = quat.getX();
    //    static_transformStamped.transform.rotation.y = quat.getY();
    //    static_transformStamped.transform.rotation.z = quat.getZ();
    //    static_transformStamped.transform.rotation.w = quat.getW();

    static_broadcaster.sendTransform(static_transformStamped);

    double terror = 0.01;
    bool grapSucceed = false;

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

      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        continue;
      }

      // Compute lMlp and fMfp
      lMlp = wMl.inverse() * wMlp;

      // Get Translation from vMlp and vMfp
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
      double vx = GetLinearVel(vTlpX);
      double vy = GetLinearVel(vTlpY);
      double vz = GetLinearVel(vTlpZ);

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
        vx = GetLinearVel(vTfpX);
        vy = GetLinearVel(vTfpY);
        vz = GetLinearVel(vTfpZ);

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
        fv = wMf.inverse() * wVl * lpMfp;

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

      if (std::abs(vTlpX) < terror && std::abs(vTlpY) < terror &&
          std::abs(vTlpZ) < terror && std::abs(vTfpX) < terror &&
          std::abs(vTfpY) < terror && std::abs(vTfpZ) < terror) {

        wMlp.setOrigin(tf::Vector3(lxorig, lyorig, lzorig));
        wMlp.setRotation(tf::createQuaternionFromRPY(0, 0, lyaworig));
        grapSucceed = true;
      }

      // Info("[ {} , {} , {} ] ", vTlpX, vTlpY, vTlpZ);
      //        auto lpMfpRot = lpMfp.getRotation();
      //        tf::Transform tmp;
      //        tmp.setRotation()
      rate.sleep();
    }
  });
  work.detach();
}

CLASS_LOADER_REGISTER_CLASS(TwinbotNetSimTracing, NetSimTracing)
} // namespace uwsim_netstim

/*
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
*/
