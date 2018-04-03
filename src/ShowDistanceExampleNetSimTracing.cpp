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

  //---------------------------------------------------------------------

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
