#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <uwsim_netsim_scripts/ShipwreckNetSimTracing.h>

namespace uwsim_netstim {

ShipwreckNetSimTracing::ShipwreckNetSimTracing() : NetSimTracing() {
  bluerov2_gled_pub = node.advertise<underwater_sensor_msgs::LedLight>(
      "/bluerov2_leds/green", 1);
  bluerov2_rled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/bluerov2_leds/red", 1);

  boat_rled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/boat_leds/red", 1);
  boat_gled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/boat_leds/green", 1);

  buoy_rled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/buoy_leds/red", 1);
  buoy_gled_pub =
      node.advertise<underwater_sensor_msgs::LedLight>("/buoy_leds/green", 1);

  ledmsg.duration = ros::Duration(0.1);
}

void ShipwreckNetSimTracing::PacketTransmitting(std::string path,
                                                ROSCommsDevicePtr dev,
                                                ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
  auto mac = dev->GetMac();
  switch (mac) {
  case 0:
    boat_gled_pub.publish(ledmsg);
    break;
  case 1:
    buoy_gled_pub.publish(ledmsg);
    break;
  case 2:
    buoy_gled_pub.publish(ledmsg);
    break;
  case 3:
    bluerov2_gled_pub.publish(ledmsg);
    break;
  }
}
void ShipwreckNetSimTracing::PacketDropsUpdated(std::string path,
                                                uint32_t oldValue,
                                                uint32_t newValue) {
  Info("[{}] PKTDROPS {}", path, newValue);
}

void ShipwreckNetSimTracing::TxFifoUpdated(std::string path, uint32_t oldValue,
                                           uint32_t newValue) {
  Info("[{}] TXFIFO {}", path, newValue);
}

void ShipwreckNetSimTracing::PacketError(std::string path,
                                         ROSCommsDevicePtr dev,
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

void ShipwreckNetSimTracing::PacketReceived(std::string path,
                                            ROSCommsDevicePtr dev,
                                            ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
  auto mac = dev->GetMac();
  switch (mac) {
  case 0:
    boat_rled_pub.publish(ledmsg);
    break;
  case 1:
    buoy_rled_pub.publish(ledmsg);
    break;
  case 2:
    buoy_rled_pub.publish(ledmsg);
    break;
  case 3:
    bluerov2_rled_pub.publish(ledmsg);
    break;
  }
}

void ShipwreckNetSimTracing::MacRx(std::string path, ROSCommsDevicePtr dev,
                                   ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] MAC RX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
       dev->GetMac(), header.GetSize());
}

void ShipwreckNetSimTracing::MacTx(std::string path, ROSCommsDevicePtr dev,
                                   ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] MAC TX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
       dev->GetMac(), header.GetSize());
}

void ShipwreckNetSimTracing::Configure() {
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
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));

  //---------------------------------------------------------------------

  // We recommend the callbacks to be very simple
  // since the ns3 simulation time is stopped during the ns3 callback
  // execution:
  // https://www.nsnam.org/docs/manual/html/realtime.html

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketError",
      ns3::MakeCallback(&ShipwreckNetSimTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&ShipwreckNetSimTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&ShipwreckNetSimTracing::PacketTransmitting, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxFifoSize",
      ns3::MakeCallback(&ShipwreckNetSimTracing::TxFifoUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxPacketDrops",
      ns3::MakeCallback(&ShipwreckNetSimTracing::PacketDropsUpdated, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacTx",
                       ns3::MakeCallback(&ShipwreckNetSimTracing::MacTx, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacRx",
                       ns3::MakeCallback(&ShipwreckNetSimTracing::MacRx, this));
}

void ShipwreckNetSimTracing::DoRun() {
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

CLASS_LOADER_REGISTER_CLASS(ShipwreckNetSimTracing, NetSimTracing)
} // namespace uwsim_netstim
