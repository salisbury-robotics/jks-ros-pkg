#include "bosch_actuator.h"
int BoschActuator::initialize(pr2_hardware_interface::HardwareInterface *)
{
  return 0;
}
void BoschActuator::sendCommand(bool halt, bool reset)
{
  return;
}

bool BoschActuator::updateState() 
{
  return true;
}

BoschActuator::~BoschActuator()
{
}

BoschActuator::BoschActuator()
{
}