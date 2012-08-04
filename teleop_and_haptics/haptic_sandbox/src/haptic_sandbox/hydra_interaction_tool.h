#ifndef _HYDRA_INTERACTION_TOOL_H_
#define _HYDRA_INTERACTION_TOOL_H_

#include <haptic_sandbox/abstract_interaction_tool.h>
#include <hydra/Calib.h>
#include <ros/ros.h>

namespace something {

class HydraInteractionTool: public AbstractInteractionTool{


public:
  // Methods only!

    HydraInteractionTool(const std::string &frame_id,
                                                 tf::TransformListener *tfl,
                                                 tf::TransformBroadcaster *tfb)
        : AbstractInteractionTool(frame_id, tfl, tfb),
          workspace_radius_(0.5)
    {
        // Finish other intialization stuff.
        init();
    }

    virtual ~HydraInteractionTool();

    void init();


protected:
// Methods

    void updateFromMsg(const hydra::CalibConstPtr &calib);

// Members
    float workspace_radius_;
    ros::Subscriber hydra_sub_;

};


}  // namespace something

#endif
