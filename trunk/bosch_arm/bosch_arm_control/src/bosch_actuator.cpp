#include "bosch_actuator.h"
#include "daq.h"
#include <boost/lexical_cast.hpp>
int BoschActuator::initialize ( pr2_hardware_interface::HardwareInterface * hw, TiXmlElement *xit )
{
  //actuator_.name_=name;
  const char *name = xit->Attribute ( "name" );
  actuator_.name_ = name ? name : "";
  TiXmlElement *jel = xit->FirstChildElement ( "cmd_addr" );

  try
  {
    const char *str=jel->Attribute ( "value" );
    cmd_addr_ = boost::lexical_cast<int> ( str );
  }
  catch ( boost::bad_lexical_cast &e )
  {
    ROS_ERROR ( "a (%s) is not a float",jel );
    return false;
  }
  jel = xit->FirstChildElement ( "encoder_addr" );
  try
  {
    const char *str=jel->Attribute ( "value" );
    encoder_addr_ = boost::lexical_cast<int> ( str );
  }
  catch ( boost::bad_lexical_cast &e )
  {
    ROS_ERROR ( "a (%s) is not a float",jel );
    return false;
  }


  if ( hw && !hw->addActuator ( &actuator_ ) )
  {
    ROS_FATAL ( "An actuator of the name '%s' already exists.", actuator_.name_.c_str() );
    return -1;
  }
  //printf("%s\n",hw->getActuator(actuator_.name_)->name_.c_str());
  return 0;
}

///forward kinematics, i.e. the jocabian depends on the start position.
///right now we just manually put the arm joints at the zero position.
///the controller should read the motor encoder and set the zero offset.
///calibration means the physical state is consistent with the model
///every device needs to be calibrated before use. This should
///be in the starter part of controller.
///the update part assumes the previous state is right.
///doTxRx only sync the hardware and softwareinterface.
///how to interprete the meaning is left to controllers.

bool BoschActuator::doTxRx ( bool halt, bool reset )
{

  pr2_hardware_interface::ActuatorState &state = actuator_.state_;
  pr2_hardware_interface::ActuatorCommand &cmd = actuator_.command_;
  //TODO: the effort here is not in N.m unit. find out the motor_toque_constant and encoder_reduction.
  ///The time messured here is not absolutely accurate due to process time-sharing, how bad is this?
  write_torque ( cmd_addr_,cmd.effort_ );
  int32_t encoder_count=read_encoder ( encoder_addr_ );
  struct timespec tick;
  clock_gettime ( CLOCK_REALTIME, &tick );
  state.sample_timestamp_ = ros::Duration ( tick.tv_sec,tick.tv_nsec );   //ros::Duration is preferred source of time for controllers
  state.timestamp_ = state.sample_timestamp_.toSec();  //double value is for backwards compatibility
  state.encoder_count_ = encoder_count;
  state.position_ = double ( encoder_count ) / pulses_per_revolution_ * 2 * M_PI - state.zero_offset_;
  ///How does the encoder deal with overflow?
  state.encoder_velocity_ = double ( positionDiff ( encoder_count,prev_encoder_count_ ) ) / ( state.sample_timestamp_-prev_timestamp_ ).toSec();
  state.velocity_ = state.encoder_velocity_ / pulses_per_revolution_ * 2 * M_PI;
  state.last_executed_effort_ = cmd.effort_;
  //TODO: a lot of state field are ignored, some of them might be used by certain controllers.
  //and the unit of effort used here may not be compatible with the controllers.
  prev_encoder_count_=encoder_count;
  prev_timestamp_=state.sample_timestamp_;
  return true;
}

BoschActuator::~BoschActuator()
{
}

BoschActuator::BoschActuator()
{
}


/**
 * Returns (new_position - old_position).  Accounts for wrap-around of 32-bit position values.
 *
 * It is assumed that each position value is exactly 32bit and can wrap from -2147483648 to +2147483647.
 */
int32_t BoschActuator::positionDiff ( int32_t new_position, int32_t old_position )
{
  return int32_t ( new_position - old_position );
}
