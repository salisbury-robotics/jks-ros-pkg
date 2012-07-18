

#include <Eigen/Geometry>



namespace something {

typedef Eigen::Vector3d Vector3;
typedef Eigen::Quaterniond Quaternion;

class AbstractInteractionTool{


public:
  // Methods only!

  // Constructor
  AbstractInteractionTool() {}
  
  // Read the position of the tool.
  virtual Vector3 getToolPosition() const           { return tool_position_; }
  
  // Read the quaternion of the tool.
  virtual Quaternion getToolQuaternion() const      { return tool_quaternion_; }

  // Read the state of the binary switches on the tool.
  virtual bool getToolButtonState(const unsigned int &index) const  { return false; }

  // Get the number of buttons available on the tool.
  virtual unsigned int getToolButtonCount() const                   { return 0; }

  // Read the position of the tool.
  virtual Vector3 getInteractionFramePosition() const           { return base_frame_position_; }

  // Read the quaternion of the tool.
  virtual Quaternion getInteractionFrameQuaternion() const      { return base_frame_quaternion_; }
  
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
  virtual void setToolPosition(const Vector3 &position);

  // Set tool quaternion
  virtual void setToolQuaternion(const Quaternion &quaternion);

  // Set tool position
  virtual void setInteractionFramePosition(const Vector3 &position);

  // Set tool quaternion
  virtual void setInteractionFrameQuaternion(const Quaternion &quaternion);

  // Set workspace size for the tool interaction
  virtual void setToolWorkspaceRadius(const float &radius);



protected: 
// Methods

  // Call an update?
  virtual void update();

  virtual void draw();


// Members

  Vector3 tool_position_;
  Quaternion tool_quaternion_;

  Vector3 base_frame_position_;
  Quaternion base_frame_quaternion_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

};





}  // namespace something
