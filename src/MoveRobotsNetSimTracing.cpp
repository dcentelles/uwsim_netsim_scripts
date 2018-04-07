#include <class_loader/multi_library_class_loader.h>
#include <functional>
#include <uwsim_netsim_scripts/MoveRobotsNetSimTracing.h>

namespace uwsim_netstim {

MoveRobotsNetSimTracing::MoveRobotsNetSimTracing() : NetSimTracing() {
  bluerov2Pub = node.advertise<geometry_msgs::TwistStamped>(
      "/bluerov2/velocityCommand", 1);
  bluerov2fPub = node.advertise<geometry_msgs::TwistStamped>(
      "/bluerov2_f/velocityCommand", 1);
}

void MoveRobotsNetSimTracing::ShowDistance(bool &lastPosValid,
                                           tf::Vector3 &lastPos,
                                           bool &peerPosValid,
                                           tf::Vector3 &peerPos, string path,
                                           ROSCommsDevicePtr dev,
                                           const tf::Vector3 &pos) {
  lastPos = pos;
  lastPosValid = true;
  if (peerPosValid) {
    double distance = lastPos.distance(peerPos);
    Info("DIST: {}", distance);
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

void MoveRobotsNetSimTracing::PacketCollision(std::string path,
                                              ROSCommsDevicePtr dev,
                                              ns3PacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Warn("[{}] COL -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void MoveRobotsNetSimTracing::PacketPropError(std::string path,
                                              ROSCommsDevicePtr dev,
                                              ns3PacketPtr pkt) {

  NetsimHeader header;
  pkt->PeekHeader(header);
  Warn("[{}] PERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
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
  LogToFile("netsim_log");
  // FlushLogOn(cpplogging::info); //Not recommended

  // For custom formatting of log messages:
  // https://github.com/gabime/spdlog/wiki/3.-Custom-formatting
  Log->set_pattern("[%D %T.%F] %v");

  //---------------------------------------------------------------------

  ns3::Config::Connect(
      "/ROSDeviceList/0/CourseChange",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::ShowDistanceDev0, this));
  ns3::Config::Connect(
      "/ROSDeviceList/1/CourseChange",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::ShowDistanceDev1, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketCollision",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketCollision, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketPropError",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketPropError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&MoveRobotsNetSimTracing::PacketTransmitting, this));
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
