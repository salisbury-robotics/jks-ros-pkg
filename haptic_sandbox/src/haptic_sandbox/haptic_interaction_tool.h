#ifndef _HAPTIC_INTERACTION_TOOL_H_
#define _HAPTIC_INTERACTION_TOOL_H_

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <haptic_sandbox/abstract_interaction_tool.h>

#include <chai3d.h>
#include <conversions/tf_chai.h>





namespace something {

class HapticInteractionTool: public AbstractInteractionTool{


public:
  // Methods only!

  // Constructor
    HapticInteractionTool(const std::string &frame_id,
             tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
    : AbstractInteractionTool(frame_id, tfl, tfb)
    {
        init();
    }

    void init();

    virtual ~HapticInteractionTool();

    // Read the state of the binary switches on the tool.
    virtual bool getToolButtonState(const unsigned int &index) const;

    // Get the number of buttons available on the tool.
    virtual unsigned int getToolButtonCount() const;

//  // Get the name of this tool.
//  virtual std::string getToolName() const;

//  // Set the force applied to the tool
//  virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

//  // Set the torque applied to the tool
//  virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

//  // Set the force and torque applied to the tool
//  virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

//  // Set the gripper force on the tool
//  virtual void setToolGripperForce(const float &force);

//  //! Device's base frame is transparent, client sees only a device with "infinite workspace"
//  // Set tool position
//  virtual void setToolPosition(const Vector3 &position)
//  {
//    cVector3d device_position;
//    chai_device_->getPosition(device_position);
//    base_frame_position_ = position - base_frame_quaternion_ * device_position;
//  }

//  // Set tool quaternion
//  virtual void setToolQuaternion(const Quaternion &quaternion)
//  {
//    cMatrix3d device_matrix;
//    chai_device_->getRotation(device_matrix);
//    Quaternion device_quaternion(device_matrix);
//    base_frame_quaternion_ = quaternion * device_quaternion.conjugate();
//  }

//  // Set tool position
//  virtual void setInteractionFramePosition(const Vector3 &position)
//  {
//    base_frame_position_ = position;
//  }

//  // Set tool quaternion
//  virtual void setInteractionFrameQuaternion(const Quaternion &quaternion)
//  {
//    base_frame_quaternion_ = quaternion;
//  }

//  // Set workspace size for the tool interaction
//  virtual void setToolWorkspaceRadius(const float &radius);



protected:
// Methods

  // Call an update?
  virtual void updateDevice();

// Members

  cHapticDeviceHandler *chai_device_handler_;
  cGenericHapticDevice *chai_device_;

  ros::Timer interaction_timer_;

};


}  // namespace something

#endif
