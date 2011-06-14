#ifndef BOSCHARM_HARDWARE_H
#define BOSCHARM_HARDWARE_H

#include <pr2_hardware_interface/hardware_interface.h>

// #include <al/ethercat_AL.h>
// #include <al/ethercat_master.h>
// #include <al/ethercat_slave_handler.h>
#include<diagnostic_msgs/DiagnosticArray.h> 
#include "bosch_actuator.h"
// #include "ethercat_hardware/ethercat_com.h"
// #include "ethercat_hardware/ethernet_interface_info.h"
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <realtime_tools/realtime_publisher.h>
  
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <pluginlib/class_loader.h>

#include <std_msgs/Bool.h>
#include <string>
using namespace boost::accumulators;
using namespace std; 

class BoschArmHardware
{
public:
  /*!
   * \brief Constructor
   */
  BoschArmHardware(const std::string& name);

  /*!
   * \brief Destructor
   */
  ~BoschArmHardware();

  /*!
   * \brief Send most recent motor commands and retrieve updates. This command must be run at a sufficient rate or else the motors will be disabled.
   * \param reset A boolean indicating if the motor controller boards should be reset
   * \param halt A boolean indicating if the motors should be halted
   */
  void update(bool reset, bool halt);

  /*!
   * \brief Initialize the EtherCAT Master Library.
   * \param interface The socket interface that is connected to the EtherCAT devices (e.g., eth0)
   * \param allow_unprogrammed A boolean indicating if the driver should treat the discovery of unprogrammed boards as a fatal error.  Set to 'true' during board configuration, and set to 'false' otherwise.
   */
  void init(TiXmlElement* config);
  pr2_hardware_interface::HardwareInterface *hw_;

private:
  ros::NodeHandle node_;
  vector<BoschArmDevice *>slaves_;
  //static const unsigned int num_slaves_=4; //four motors.
};

#endif /* BOSCHARM_HARDWARE_H */
