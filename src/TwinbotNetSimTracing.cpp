#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <uwsim_netsim_scripts/TwinbotNetSimTracing.h>

namespace uwsim_netstim {

TwinbotNetSimTracing::TwinbotNetSimTracing() : NetSimTracing() {

  leader_pub =
      node.advertise<geometry_msgs::TwistStamped>("/uwsim/leader/velocity", 1);
  follower_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/follower/velocity", 1);
  support_pub =
      node.advertise<geometry_msgs::TwistStamped>("/uwsim/support/velocity", 1);

  leader_sub = node.subscribe<nav_msgs::Odometry>(
      "/uwsim/leader/odom", 1, &TwinbotNetSimTracing::LeaderOdomCb, this);
  follower_sub = node.subscribe<nav_msgs::Odometry>(
      "/uwsim/follower/odom", 1, &TwinbotNetSimTracing::FollowerOdomCb, this);
  support_sub = node.subscribe<nav_msgs::Odometry>(
      "/uwsim/support/odom", 1, &TwinbotNetSimTracing::SupportOdomCb, this);
}

void TwinbotNetSimTracing::LeaderOdomCb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  leader_odom = *msg;
}
void TwinbotNetSimTracing::FollowerOdomCb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  follower_odom = *msg;
}
void TwinbotNetSimTracing::SupportOdomCb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  support_odom = *msg;
}

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
  double vel = 0.20; // 0.20 m/s
  double v;
  if (std::abs(diff) > 2) {
    v = vel / freq;
    if (diff < 0) {
      v = -1 * v;
    }
  } else {
    v = diff / 2;
  }
}

void TwinbotNetSimTracing::DoRun() {
  std::thread work([this]() {
    double tfx = 0.568033, tfy = 0.642953, tfz = 18.5175;
    double tfroll = -2.64855e-15, tfpich = -1.8232e-15, tfyaw = -3.10194;

    double tlx = 0.143354, tly = -0.847053, tlz = 18.5122;
    double tlroll = -1.0775e-15, tlpich = -2.94342e-15, tlyaw = 2.59545;

    geometry_msgs::TwistStamped f_msg, l_msg, s_msg;

    ros::Rate rate(freq);

    while (ros::ok()) {
      f_msg.twist.linear.x = 0;
      f_msg.twist.linear.y = 0;
      f_msg.twist.linear.z = 0;
      f_msg.twist.angular.x = 0;
      f_msg.twist.angular.y = 0;
      f_msg.twist.angular.z = 0;

      l_msg.twist.linear.x = 0;
      l_msg.twist.linear.y = 0;
      l_msg.twist.linear.z = 0;
      l_msg.twist.angular.x = 0;
      l_msg.twist.angular.y = 0;
      l_msg.twist.angular.z = 0;

      s_msg.twist.linear.x = 0;
      s_msg.twist.linear.y = 0;
      s_msg.twist.linear.z = 0;
      s_msg.twist.angular.x = 0;
      s_msg.twist.angular.y = 0;
      s_msg.twist.angular.z = 0;

      double xdiff = tlx - leader_odom.pose.pose.position.x;
      double ydiff = tly - leader_odom.pose.pose.position.y;
      double zdiff = tlz - leader_odom.pose.pose.position.z;

      double vx = GetLinearVel(xdiff);
      double vy = GetLinearVel(ydiff);
      double vz = GetLinearVel(zdiff);

      Info("[ {} , {} , {} ] --- [ {} , {} , {} ] --- [ {} , {} , {} ]",
           leader_odom.pose.pose.position.x, leader_odom.pose.pose.position.y,
           leader_odom.pose.pose.position.z, xdiff, ydiff, zdiff, vx, vy, vz);

      // leader_pub.publish(l_msg);
      rate.sleep();
    }
  });
  work.detach();
}

CLASS_LOADER_REGISTER_CLASS(TwinbotNetSimTracing, NetSimTracing)
} // namespace uwsim_netstim
