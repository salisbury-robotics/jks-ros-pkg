#ifndef _ABSTRACT_HANDLE_H_
#define _ABSTRACT_HANDLE_H_

#include <haptic_sandbox/tf_scenegraph_object.h>
#include <Eigen/Geometry>

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


protected:
// Methods


// Members

    //something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

};

}  // namespace something

#endif
