
#ifndef _MANIPULATOR_NODE_H_
#define _MANIPULATOR_NODE_H_

#include <haptic_sandbox/abstract_interaction_tool.h>
#include <haptic_sandbox/haptic_interaction_tool.h>
#include <haptic_sandbox/hydra_interaction_tool.h>
#include <haptic_sandbox/tf_scenegraph_object.h>

#include <Eigen/Geometry>


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

    virtual ~ManipulatorNode()
    {
        if(tool_) delete tool_;
    }

    void init();

    // Get button stuff should go here, so user can query it?

    bool isGrabbing()
    {
        unsigned int index = button_name_map_["grab"];
        return tool_->getToolButtonState(index);
    }


protected:
    // Methods

protected:
    // Members

    something::AbstractInteractionTool *tool_;
    std::map<std::string, unsigned int> button_name_map_;
};

}  // namespace something

#endif
