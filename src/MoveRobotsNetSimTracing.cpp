#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <uwsim_netsim_scripts/MoveRobotsNetSimTracing.h>
namespace uwsim_netstim {

MoveRobotsNetSimTracing::MoveRobotsNetSimTracing() : NetSimTracing() {
  bluerov2Pub = node.advertise<geometry_msgs::TwistStamped>(
      "/bluerov2/velocityCommand", 1);
  bluerov2fPub = node.advertise<geometry_msgs::TwistStamped>(
      "/bluerov2_f/velocityCommand", 1);
  showDistanceTimer.Reset();
}

void MoveRobotsNetSimTracing::ShowDistance(bool &lastPosValid,
                                           tf::Vector3 &lastPos,
                                           bool &peerPosValid,
                                           tf::Vector3 &peerPos, string path,
                                           ROSCommsDevicePtr dev,
                                           const tf::Vector3 &pos) {
  lastPos = pos;
  lastPosValid = true;
  if (peerPosValid) { // & showDistanceTimer.Elapsed() > 250) {
    double distance = lastPos.distance(peerPos);
    Info("DIST: {}", distance);
    // showDistanceTimer.Reset();
  }
}

void MoveRobotsNetSimTracing::ShowDistanceDev0(string path,
                                               ROSCommsDevicePtr dev,
                                               const tf::Vector3 &pos) {
  ShowDistance(dev0PosValid, dev0Pos, dev1PosValid, dev1Pos, path, dev, pos);
}
void MoveRobotsNetSimTracing::ShowDistanceDev1(string path,
                                               ROSCommsDevicePtr dev,
                                               const tf::Vector3 &pos) {
  ShowDistance(dev1PosValid, dev1Pos, dev0PosValid, dev0Pos, path, dev, pos);
}

void MoveRobotsNetSimTracing::PacketTransmitting(std::string path,
                                                 ROSCommsDevicePtr dev,
                                                 ns3PacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}
void MoveRobotsNetSimTracing::PacketDropsUpdated(std::string path,
                                                 uint32_t oldValue,
                                                 uint32_t newValue) {
  Info("[{}] PKTDROPS {}", path, newValue);
}

void MoveRobotsNetSimTracing::TxFifoUpdated(std::string path, uint32_t oldValue,
                                            uint32_t newValue) {
  Info("[{}] TXFIFO {}", path, newValue);
}

void MoveRobotsNetSimTracing::PacketError(std::string path,
                                          ROSCommsDevicePtr dev,
                                          ns3PacketPtr pkt, bool propErr,
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

void MoveRobotsNetSimTracing::PacketReceived(std::string path,
                                             ROSCommsDevicePtr dev,
                                             ns3PacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void MoveRobotsNetSimTracing::Configure() {
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
  //  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%D %T.%F]
  //  %v"));

  //---------------------------------------------------------------------

  // We recommend the callbacks to be very simple
  // since the ns3 simulation time is stopped during the ns3 callback
  // execution:
  // https://www.nsnam.org/docs/manual/html/realtime.html

  ns3::Config::Connect(
      "/ROSDeviceList/0/CourseChange",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::ShowDistanceDev0, this));
  ns3::Config::Connect(
      "/ROSDeviceList/1/CourseChange",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::ShowDistanceDev1, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketError",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketTransmitting, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxFifoSize",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::TxFifoUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxPacketDrops",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketDropsUpdated, this));
}

void MoveRobotsNetSimTracing::DoRun() {
  std::thread work([this]() {
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = 0;
    msg.twist.linear.y = 0;
    msg.twist.linear.z = 0;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;

    double baseVelocity = 0.1;
    int counter, its = 1000;

    ros::Rate rate(20);
    while (ros::ok()) {
      std::this_thread::sleep_for(10s);
      msg.twist.linear.x = baseVelocity;

      counter = 0;
      while (counter < its && ros::ok()) {
        bluerov2Pub.publish(msg);
        rate.sleep();
        counter += 1;
      }

      msg.twist.linear.x = -baseVelocity;

      counter = 0;
      while (counter < its && ros::ok()) {
        bluerov2Pub.publish(msg);
        rate.sleep();
        counter += 1;
      }
    }
  });
  work.detach();
}

CLASS_LOADER_REGISTER_CLASS(MoveRobotsNetSimTracing, NetSimTracing)
}
