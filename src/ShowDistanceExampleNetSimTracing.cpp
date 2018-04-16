#include <class_loader/multi_library_class_loader.h>
#include <functional>
#include <uwsim_netsim_scripts/ShowDistanceExampleNetSimTracing.h>

namespace uwsim_netstim {

ShowDistanceExampleNetSimTracing::ShowDistanceExampleNetSimTracing()
    : NetSimTracing() {}

void ShowDistanceExampleNetSimTracing::ShowDistance(
    bool &lastPosValid, tf::Vector3 &lastPos, bool &peerPosValid,
    tf::Vector3 &peerPos, string path, ROSCommsDevicePtr dev,
    const tf::Vector3 &pos) {
  lastPos = pos;
  lastPosValid = true;
  if (peerPosValid) {
    double distance = lastPos.distance(peerPos);
    Info("distance: {} meters", distance);
  }
}

void ShowDistanceExampleNetSimTracing::ShowDistanceDev0(
    string path, ROSCommsDevicePtr dev, const tf::Vector3 &pos) {
  ShowDistance(dev0PosValid, dev0Pos, dev1PosValid, dev1Pos, path, dev, pos);
}
void ShowDistanceExampleNetSimTracing::ShowDistanceDev1(
    string path, ROSCommsDevicePtr dev, const tf::Vector3 &pos) {
  ShowDistance(dev1PosValid, dev1Pos, dev0PosValid, dev0Pos, path, dev, pos);
}
void ShowDistanceExampleNetSimTracing::Configure() {
  SetLogName("uwsim_netsim_scripts");

  // Uncomment to sent all log messages to a file:
  LogToFile("netsim_log");
  //LogToConsole(true);

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
      ns3::MakeCallback(&ShowDistanceExampleNetSimTracing::ShowDistanceDev0,
                        this));
  ns3::Config::Connect(
      "/ROSDeviceList/1/CourseChange",
      ns3::MakeCallback(&ShowDistanceExampleNetSimTracing::ShowDistanceDev1,
                        this));
}

CLASS_LOADER_REGISTER_CLASS(ShowDistanceExampleNetSimTracing, NetSimTracing)
}
