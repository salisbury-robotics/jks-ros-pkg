
#include <haptic_sandbox/hydra_interaction_tool.h>
#include <hydra/CalibPaddle.h>

#include <tf/tf.h>
#include <Eigen/Geometry>
#include <ros/ros.h>



namespace something {


HydraInteractionTool::~HydraInteractionTool()
{
}


void HydraInteractionTool::init()
{

//    setPosition(tf::Vector3(pos.x(), pos.y(), pos.z()));
//    setQuaternion(tf::createQuaternionFromYaw(M_PI)*tf::createQuaternionFromRPY(0, 0.4, 0));

    ros::NodeHandle nh;
    hydra_sub_ = nh.subscribe<hydra::Calib>("hydra_calib", 1, boost::bind(&HydraInteractionTool::updateFromMsg, this, _1));

}

/////////////////////////////////////////////////////////////////////
// PROTECTED FUNCTIONS LIVE UNDER HERE
/////////////////////////////////////////////////////////////////////

void HydraInteractionTool::updateFromMsg(const hydra::CalibConstPtr &calib)
{
    ROS_DEBUG_NAMED("hydra", "Got hydra update!");
    hydra::CalibPaddle paddle = calib->paddles[0];

    // Update pose info
    tf::Transform interaction_handle;
    tf::transformMsgToTF(paddle.transform, interaction_handle);
    handle_->setTransform(interaction_handle);

    // Update button info
    if(getToolButtonCount() != paddle.buttons.size())
        setToolButtonCount(paddle.buttons.size());
    for(size_t i = 0; i < getToolButtonCount(); ++i)
    {
        setToolButtonState(i, paddle.buttons[i]);
    }
}



}  // namespace something

