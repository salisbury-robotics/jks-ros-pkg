#ifndef BOSCH_ACTUATOR_HARDWARE_H
#define BOSCH_ACTUATOR_HARDWARE_H
#include "bosch_arm_device.h"
class BoschActuator : public BoschArmDevice
{
  public:
   BoschActuator();
   ~BoschActuator();
   void sendCommand(bool halt, bool reset);
   bool updateState();
   int initialize(pr2_hardware_interface::HardwareInterface *);
   
};

#endif