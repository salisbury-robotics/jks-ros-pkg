#ifndef BOSCH_ACTUATOR_HARDWARE_H
#define BOSCH_ACTUATOR_HARDWARE_H
#include "bosch_arm_device.h"


///TODO: arm calibration for joint limit, initial orientation and joint position
///TODO: think about all the other hard parameters(e.g. motor_torque_constant_) whether they can be or need to be calibrated

class BoschActuator : public BoschArmDevice
{
public:
    BoschActuator();
    ~BoschActuator();

    bool doTxRx(bool halt, bool reset);
    int initialize(pr2_hardware_interface::HardwareInterface *,string name);
    pr2_hardware_interface::Actuator actuator_;
    int cmd_addr_;
    int encoder_addr_;
    int32_t prev_encoder_count_;

    uint32_t pulses_per_revolution_;

    ros::Duration prev_timestamp_;

    static int32_t positionDiff(int32_t new_position, int32_t old_position);


};

#endif
