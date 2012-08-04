#ifndef _ABSTRACT_INTERACTION_TOOL_H_
#define _ABSTRACT_INTERACTION_TOOL_H_


#include <haptic_sandbox/tf_scenegraph_object.h>
#include <haptic_sandbox/abstract_handle.h>

#include <Eigen/Geometry>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class AbstractInteractionTool: public tf::SceneGraphNode{

public:
  // Methods only!

  // Constructor
    // Constructor
      AbstractInteractionTool(const std::string &frame_id,
                 tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
          : SceneGraphNode(frame_id, tfl, tfb)
    {
        init();
    }

    virtual ~AbstractInteractionTool()
    {
          if(handle_) delete handle_;
    }

    void init()
    {
        handle_ = new something::AbstractHandle(transform_.child_frame_id_ + "_handle", tfl_, tfb_);
        addChild(handle_);
    }


    const something::AbstractHandle* getHandle()
    {
        return handle_;
    }


    // Read the state of the binary switches on the tool.
    bool getToolButtonState(const unsigned int &index) const
    {
        if(index >= getToolButtonCount()) return false;
        return button_state_[index];
    }

    // Get the number of buttons available on the tool.
    unsigned int getToolButtonCount() const
    {
        return (unsigned int)button_state_.size();
    }

    // Set the force applied to the tool
    virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

    // Set the torque applied to the tool
    virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

    // Set the force and torque applied to the tool
    virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

//    // Set the gripper force on the tool
//    virtual void setToolGripperForce(const float &force);


protected: 
// Methods

    void setToolButtonCount(const unsigned int &count)
    {
        button_state_.resize(count, false);
    }

    void setToolButtonState(const size_t &index, const bool &state)
    {
        if(index >= button_state_.size())
        {
            ROS_ERROR("Can't set button %zd state, max size is %zd", index, button_state_.size());
            return;
        }
        button_state_[index] = state;
    }

//    virtual void updateDevice()
//    {
//      // Here is where, for example, we:
//      // readDevicePosition();
//      // getAttachedFramePosition();
//      // computeVirtualCouplingForce();
//      // sendDeviceForce();
//    }


// Members

    something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

  std::vector<bool> button_state_;

};

}  // namespace something

#endif
