#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <geometry_msgs/Pose.h>
#include <uwsim_netsim_scripts/ArdusubTest0.h>

namespace uwsim_netstim {

ArdusubTest0::ArdusubTest0() : NetSimTracing() {
  bluerov2Pub = node.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);
  showDistanceTimer.Reset();
}

void ArdusubTest0::ShowDistance(bool &lastPosValid, tf::Vector3 &lastPos,
                                bool &peerPosValid, tf::Vector3 &peerPos,
                                string path, ROSCommsDevicePtr dev,
                                const tf::Vector3 &pos) {
  lastPos = pos;
  lastPosValid = true;
  if (peerPosValid) { // & showDistanceTimer.Elapsed() > 250) {
    double distance = lastPos.distance(peerPos);
    Info("DIST: {}", distance);
    // showDistanceTimer.Reset();
  }
}

void ArdusubTest0::ShowDistanceDev0(string path, ROSCommsDevicePtr dev,
                                    const tf::Vector3 &pos) {
  ShowDistance(dev0PosValid, dev0Pos, dev1PosValid, dev1Pos, path, dev, pos);
}
void ArdusubTest0::ShowDistanceDev1(string path, ROSCommsDevicePtr dev,
                                    const tf::Vector3 &pos) {
  ShowDistance(dev1PosValid, dev1Pos, dev0PosValid, dev0Pos, path, dev, pos);
}

void ArdusubTest0::PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                                      ns3PacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}
void ArdusubTest0::PacketDropsUpdated(std::string path, uint32_t oldValue,
                                      uint32_t newValue) {
  Info("[{}] PKTDROPS {}", path, newValue);
}

void ArdusubTest0::TxFifoUpdated(std::string path, uint32_t oldValue,
                                 uint32_t newValue) {
  Info("[{}] TXFIFO {}", path, newValue);
}

void ArdusubTest0::PacketError(std::string path, ROSCommsDevicePtr dev,
                               ns3PacketPtr pkt, bool propErr, bool colErr) {

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

void ArdusubTest0::PacketReceived(std::string path, ROSCommsDevicePtr dev,
                                  ns3PacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void ArdusubTest0::Configure() {
  SetLogName("uwsim_netsim_scripts");

  // Uncomment to sent all log messages to a file:
  //LogToFile("netsim_log");
  LogToConsole(true);
  SetAsyncMode(true);

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
  // SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%D %T.%F] %v"));
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));

  //---------------------------------------------------------------------

  // We recommend the callbacks to be very simple
  // since the ns3 simulation time is stopped during the ns3 callback
  // execution:
  // https://www.nsnam.org/docs/manual/html/realtime.html

//  ns3::Config::Connect(
//      "/ROSDeviceList/0/CourseChange",
//      ns3::MakeCallback(&ArdusubTest0::ShowDistanceDev0, this));
//  ns3::Config::Connect(
//      "/ROSDeviceList/1/CourseChange",
//      ns3::MakeCallback(&ArdusubTest0::ShowDistanceDev1, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketError",
      ns3::MakeCallback(&ArdusubTest0::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&ArdusubTest0::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&ArdusubTest0::PacketTransmitting, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxFifoSize",
      ns3::MakeCallback(&ArdusubTest0::TxFifoUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxPacketDrops",
      ns3::MakeCallback(&ArdusubTest0::PacketDropsUpdated, this));
}

void ArdusubTest0::DoRun() {
  //  std::thread work([this]() {
  //    geometry_msgs::Pose bluerov2msg;
  //    bluerov2msg.position.x = 2;
  //    bluerov2msg.position.y = 0;
  //    bluerov2msg.position.z = 4;
  //    bluerov2msg.orientation.x = 0;
  //    bluerov2msg.orientation.y = 0;
  //    bluerov2msg.orientation.z = 0;
  //    bluerov2msg.orientation.w = 1;

  //    ros::Rate rate(1);
  //    while (ros::ok()) {
  //      bluerov2Pub.publish(bluerov2msg);
  //      rate.sleep();
  //    }
  //  });
  //  work.detach();
}

CLASS_LOADER_REGISTER_CLASS(ArdusubTest0, NetSimTracing)
}
