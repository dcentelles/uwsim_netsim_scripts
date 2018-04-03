#include <uwsim/NetSim.h>

namespace uwsim_netstim {

using namespace uwsim;

class ShowDistanceExampleNetSimTracing : public NetSimTracing {
public:
  ShowDistanceExampleNetSimTracing();
  void ShowDistance(bool &lastPosValid, tf::Vector3 &lastPos,
                    bool &peerPosValid, tf::Vector3 &peerPos, string path,
                    ROSCommsDevicePtr dev, const tf::Vector3 &pos);
  void ShowDistanceDev0(string path, ROSCommsDevicePtr dev,
                        const tf::Vector3 &pos);
  void ShowDistanceDev1(string path, ROSCommsDevicePtr dev,
                        const tf::Vector3 &pos);
  void Configure();

  tf::Vector3 dev0Pos, dev1Pos;
  bool dev0PosValid = false, dev1PosValid = false;
};
}
