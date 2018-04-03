#include <uwsim/NetSim.h>

namespace uwsim_netstim {

using namespace uwsim;

class SimpleExampleNetSimTracing : public NetSimTracing {
public:
  SimpleExampleNetSimTracing();
  void Configure();
};
}
