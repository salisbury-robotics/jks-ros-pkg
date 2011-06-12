#ifndef BOSCH_ACTUATOR_HARDWARE_H
#define BOSCH_ACTUATOR_HARDWARE_H
#include "bosch_arm_device.h"

// struct ActuatorInfo
// {
//   uint16_t major_;              // Major revision
//   uint16_t minor_;              // Minor revision
//   uint32_t id_;                 // Actuator ID
//   char name_[64];               // Actuator name
//   char robot_name_[32];         // Robot name
//   char motor_make_[32];         // Motor manufacturer
//   char motor_model_[32];        // Motor model #
//   double max_current_;          // Maximum current
//   double speed_constant_;       // Speed constant
//   double resistance_;           // Resistance
//   double motor_torque_constant_; // Motor torque constant
//   double encoder_reduction_;    // Reduction and sign between motor and encoder
//   uint32_t pulses_per_revolution_; // # of encoder ticks per revolution
//   uint8_t pad1[40];              // Pad structure to 256-4 bytes.  
//   uint32_t crc32_256_;          // CRC32 of first 256-4 bytes. (minus 4 bytes for first CRC)
//   uint8_t pad2[4];              // Pad structure to 264-4 bytes
//   uint32_t crc32_264_;          // CRC32 over entire structure (minus 4 bytes for second CRC)
// };
// 
// struct Status
// {
//   uint8_t mode_;
//   uint8_t digital_out_;
//   int16_t programmed_pwm_value_;
//   int16_t programmed_current_;
//   int16_t measured_current_;
//   uint32_t timestamp_;
//   int32_t encoder_count_;
//   int32_t encoder_index_pos_;
//   uint16_t num_encoder_errors_;
//   uint8_t encoder_status_;
//   uint8_t calibration_reading_;
//   int32_t last_calibration_rising_edge_;
//   int32_t last_calibration_falling_edge_;
//   uint16_t board_temperature_;
//   uint16_t bridge_temperature_;
//   uint16_t supply_voltage_;
//   int16_t motor_voltage_;
//   uint16_t packet_count_;
//   uint8_t pad_;
//   uint8_t checksum_;
// }__attribute__ ((__packed__));

///TODO: arm calibration for joint limit, initial orientation and joint position
///TODO: think about all the other hard parameters(e.g. motor_torque_constant_) whether they can be or need to be calibrated

class BoschActuator : public BoschArmDevice
{
  public:
   BoschActuator();
   ~BoschActuator();

   bool doTxRx(bool halt, bool reset);
   int initialize(pr2_hardware_interface::HardwareInterface *);
     pr2_hardware_interface::Actuator actuator_;
  int cmd_addr_;
  int encoder_addr_;
  int32_t prev_encoder_count_;
  
  uint32_t pulses_per_revolution_;

  ros::Duration prev_timestamp_;

  static int32_t positionDiff(int32_t new_position, int32_t old_position);

   
};

#endif