
#include <haptic_sandbox/abstract_interaction_tool.h>

#include <Eigen/Geometry>
#include <chai3d.h>



namespace something {

class HapticInteractionTool: public AbstractInteractionTool{


public:
  // Methods only!

  // Constructor
  HapticInteractionTool()
  {
    chai_device_handler_ = new cHapticDeviceHandler();
    chai_device_handler_->getDevice(chai_device_, 0);

    // open a connection with the haptic device
    if(chai_device_->open() != 0)
      printf("Error opening chai device!");
  }

  ~HapticInteractionTool()
  {
    chai_device_->close();
  }

  // Read the state of the binary switches on the tool.
  virtual bool getToolButtonState(const unsigned int &index) const
  {
    bool state = false;
    chai_device_->getUserSwitch(index, state);
    return state;
  }

  // Get the number of buttons available on the tool.
  virtual unsigned int getToolButtonCount() const
  {
    std::string modelName = chai_device_->getSpecifications().m_modelName;

    if(modelName == "PHANTOM") return 2;
    else if(modelName == "Falcon") return 4;
    else if(modelName == "omega") return 1;
    else return 0;
  }

  // Get the name of this tool.
  virtual std::string getToolName() const;

  // Set the force applied to the tool
  virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

  // Set the torque applied to the tool
  virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

  // Set the force and torque applied to the tool
  virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

  // Set the gripper force on the tool
  virtual void setToolGripperForce(const float &force);

  //! Device's base frame is transparent, client sees only a device with "infinite workspace"
  // Set tool position
  virtual void setToolPosition(const Vector3 &position)
  {
    cVector3d device_position;
    chai_device_->getPosition(device_position);
    base_frame_position_ = position - base_frame_quaternion_ * device_position;
  }

  // Set tool quaternion
  virtual void setToolQuaternion(const Quaternion &quaternion)
  {
    cMatrix3d device_matrix, device_matrix_transpose;
    chai_device_->getRotation(device_matrix);
    device_matrix.transr(device_matrix_transpose);
    Quaternion device_quaternion(device_matrix);
    base_frame_quaternion_ = quaternion * device_quaternion.conjugate();
  }

  // Set tool position
  virtual void setInteractionFramePosition(const Vector3 &position)
  {
    base_frame_position_ = position;
  }

  // Set tool quaternion
  virtual void setInteractionFrameQuaternion(const Quaternion &quaternion)
  {
    base_frame_quaternion_ = quaternion;
  }

  // Set workspace size for the tool interaction
  virtual void setToolWorkspaceRadius(const float &radius);



protected:
// Methods

  // Call an update?
  virtual void update();

  virtual void draw();


// Members

  cHapticDeviceHandler *chai_device_handler_;
  cGenericHapticDevice *chai_device_;

};


}  // namespace something
