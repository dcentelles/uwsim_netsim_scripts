#include <class_loader/multi_library_class_loader.h>
#include <uwsim_netsim_scripts/SimpleExampleNetSimTracing.h>

namespace uwsim_netstim {

SimpleExampleNetSimTracing::SimpleExampleNetSimTracing() : NetSimTracing() {}

void SimpleExampleNetSimTracing::Configure() {
  SetLogName("uwsim_netsim_scripts");

  // Uncomment to sent all log messages to a file:
  LogToFile("netsim_log");

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

  /*
  In order to use an object created outside the lambda function
  we have to make it global or capture the object. However, capturing
  does not work if the lambda function is going to be converted to a
  function pointer. So we make all this objects static.
  From the std
  (https://stackoverflow.com/questions/28746744/passing-lambda-as-function-pointer):
  "The closure type for a lambda-expression with no lambda-capture
   has a public non-virtual non-explicit const conversion function to
   pointer to function having the same parameter and return types as the
   closure type’s function call operator. The value returned by this
   conversion function shall be the address of a function that, when invoked,
   has the same effect as invoking the closure type’s function call operator."
  */

  static NetSimTracing *tracing = this;

  //---------------------------------------------------------------------

  ROSCommsDevice::PacketTransmittingCallback txcb = [](
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt) {

    // A packet is being transmitted

    NetsimHeader header;
    pkt->PeekHeader(header);
    tracing->Info("{}: (ID: {} ; MAC: {} ; Seq. Num. : {}) Transmitting packet "
                  "(USER SCRIPT)",
                  path, dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum());
  };
  ns3::Config::Connect("/ROSDeviceList/*/PacketTransmitting",
                       ns3::MakeCallback(txcb));

  //---------------------------------------------------------------------

  ROSCommsDevice::PacketReceivedCallback rxcb = [](
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt) {

    // A packet has been completely received

    NetsimHeader header;
    pkt->PeekHeader(header);
    tracing->Info(
        "{}: (ID: {} ; MAC: {} ; Seq. Num. : {}) Packet received from {} ({} "
        "bytes) (USER SCRIPT)",
        path, dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
        header.GetSrc(), header.GetPacketSize());
  };
  ns3::Config::Connect("/ROSDeviceList/*/PacketReceived",
                       ns3::MakeCallback(rxcb));

  //---------------------------------------------------------------------

  ROSCommsDevice::PacketCollisionCallback collisionCb = [](
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt) {

    // A packet has been marked corrupted by a collision. This packet could
    // have  been marked corrupted by attenuation as well. So that the
    // correct form to get total packets with errors is computing
    // packets_sent - packets_received

    NetsimHeader header;
    pkt->PeekHeader(header);
    tracing->Warn("{}: (ID: {} ; MAC: {} ; Seq. Num. : {}) Packet "
                  "colisioned! {} ({} bytes) (USER SCRIPT)",
                  path, dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
                  header.GetSrc(), header.GetPacketSize());
  };
  ns3::Config::Connect("/ROSDeviceList/*/PacketCollision",
                       ns3::MakeCallback(collisionCb));

  //---------------------------------------------------------------------
  ROSCommsDevice::PacketPropagationErrorCallback propErrorCb = [](
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt) {

    // A packet has been marked corrupted by attenuation. This packet could
    // have been marked collisioned as well. So that the correct form to
    // get total packets with errors is computing packets_sent -
    // packets_received

    NetsimHeader header;
    pkt->PeekHeader(header);
    tracing->Warn("{}: (ID: {} ; MAC: {} ; Seq. Num. : {}) Packet "
                  "corrupted by propagation! {} ({} bytes) (USER SCRIPT)",
                  path, dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
                  header.GetSrc(), header.GetPacketSize());
  };
  ns3::Config::Connect("/ROSDeviceList/*/PacketPropError",
                       ns3::MakeCallback(propErrorCb));

  //---------------------------------------------------------------------
  ROSCommsDevice::CourseChangeCallback courseChange =
      [](std::string path, ROSCommsDevicePtr dev, const tf::Vector3 &pos) {

        // A device position have been updated

        tracing->Warn("{}: (ID: {} ; MAC: {}) New position: [{},{},{}]"
                      "(USER SCRIPT)",
                      path, dev->GetDccommsId(), dev->GetMac(), pos.getX(),
                      pos.getY(), pos.getZ());
      };
  ns3::Config::Connect("/ROSDeviceList/*/CourseChange",
                       ns3::MakeCallback(courseChange));
}

CLASS_LOADER_REGISTER_CLASS(SimpleExampleNetSimTracing, NetSimTracing)
}
