#include "pr2_hardware_interface/hardware_interface.h"
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

using namespace pr2_hardware_interface;
using namespace std;
using namespace KDL;
int main()
{
  HardwareInterface hw;
  Actuator a;
  a.name_=string("funny");
  hw.addActuator(&a);
  printf("%s\n",hw.getActuator(string("funny"))->name_.c_str());
  Rotation r2 = Rotation::RPY(0,0,1.57);
  cout<<r2.UnitX()<<endl<<r2.UnitY()<<endl<<r2.UnitZ()<<endl;
  return 0;
}
