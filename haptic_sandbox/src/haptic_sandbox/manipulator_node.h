
#ifndef _MANIPULATOR_NODE_H_
#define _MANIPULATOR_NODE_H_

#include <Eigen/Geometry>
#include <haptic_sandbox/tf_scenegraph_object.h>
//#include <haptic_sandbox/abstract_interaction_tool.h>
#include <haptic_sandbox/haptic_interaction_tool.h>


namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class ManipulatorNode: public tf::SceneGraphNode {


public:
  // Methods only!

  // Constructor
    ManipulatorNode(const std::string &frame_id,
               tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
      : SceneGraphNode(frame_id, tfl, tfb)
    {
        init();
    }

    void init()
    {
        device_ = new something::HapticInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
        addChild(device_);
    }

    // Get button stuff should go here, so user can query it?



protected:
    // Methods

protected:
    // Members

    something::HapticInteractionTool *device_;
};

}  // namespace something

#endif
