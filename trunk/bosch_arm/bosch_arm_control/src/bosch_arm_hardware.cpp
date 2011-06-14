#include "bosch_arm_hardware.h"
#include <net/if.h>
#include <sys/ioctl.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

BoschArmHardware::BoschArmHardware(const std::string& name) :
  hw_(0), node_(ros::NodeHandle(name))
{
  
}

BoschArmHardware::~BoschArmHardware()
{

  if (slaves_.size()>0)
  {
    for (unsigned int i = 0; i < slaves_.size(); ++i)
    {
      if(slaves_[i])
        delete slaves_[i];
    }
  }
  delete hw_;
}


void BoschArmHardware::init(TiXmlElement* config)
{

   // Configure slaves
   // Initialize slaves
   string name;
   //slaves_=new BoschArmDevice*[num_slaves_];
   hw_ = new pr2_hardware_interface::HardwareInterface();
   hw_->current_time_ = ros::Time::now();
   TiXmlElement *xit;
   int num=0;
   for (xit = config->FirstChildElement("actuator"); xit;
       xit = xit->NextSiblingElement("actuator"))
   {
       slaves_.push_back(new BoschActuator());
       slaves_.back()->initialize(hw_,xit);
   }
//      for (unsigned int slave = 0; slave < num_slaves_; ++slave)
//      {
//          slaves_[slave]= new BoschActuator();
//          //TODO: add configSlave
//          
//          name="bosch_arm_joint"+boost::lexical_cast<string>( slave+1 )+"_motor";
//          //printf("%s\n",name.c_str());
//          
//          slaves_[slave]->initialize(hw_,ael);
//          //printf("%s\n",hw_->getActuator(name)->name_.c_str());
//      }
  

  

}

void BoschArmHardware::update(bool reset, bool halt)
{

  for (unsigned int s = 0; s < slaves_.size(); ++s)
  {
    slaves_[s]->doTxRx( halt, reset);
  }

  
}


