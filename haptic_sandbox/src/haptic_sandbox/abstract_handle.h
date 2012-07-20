#ifndef _ABSTRACT_HANDLE_H_
#define _ABSTRACT_HANDLE_H_

#include <Eigen/Geometry>
#include <haptic_sandbox/tf_scenegraph_object.h>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class AbstractHandle: public tf::SceneGraphNode{

public:
  // Methods only!

  // Constructor
    // Constructor
      AbstractHandle(const std::string &frame_id,
                 tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
        : SceneGraphNode(frame_id, tfl, tfb)
    {
        init();
    }

    void init()
    {
//        handle_ = new something::AbstractHandle(transform_.child_frame_id_ + "_handle", tfl_, tfb_);
//        addChild(handle_);
    }



//    // Read the state of the binary switches on the tool.
//    virtual bool getToolButtonState(const unsigned int &index) const  { return false; }

//    // Get the number of buttons available on the tool.
//    virtual unsigned int getToolButtonCount() const                   { return 0; }

//    // Set the force applied to the tool
//    virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

//    // Set the torque applied to the tool
//    virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

//    // Set the force and torque applied to the tool
//    virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

//    // Set the gripper force on the tool
//    virtual void setToolGripperForce(const float &force);

//    void update()
//    {
//        transform_.setOrigin(tf::Vector3(0,0,0));
//        transform_.setRotation(tf::Quaternion::getIdentity());
//    }

//      void addTransformsToVector(const ros::Time now, std::vector<tf::StampedTransform> &transforms)
//      {
//          tf::SceneGraphNode::addTransformsToVector(now, transforms);

//          // Add this node to the list
//          transform_.stamp_ = now;
//          transforms.push_back(transform_);

//          // Recursively add all children to the list
//          std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
//          for( ; it != children_.end(); it++)
//          {
//              it->second->addTransformsToVector(now, transforms);
//          }
//      }

protected:
// Methods


// Members

    //something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

};

}  // namespace something

#endif
