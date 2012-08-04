

#include <haptic_sandbox/manipulator_node.h>

namespace something{

void ManipulatorNode::init()
{
    //tool_ = new something::HapticInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
    tool_ = new something::HydraInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
    //tool_ = new something::AbstractInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
    addChild(tool_);

    button_name_map_["grab"] = 0;
}


} //namespace something
