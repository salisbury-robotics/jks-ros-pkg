#ifndef _USER_ENTITY_H_
#define _USER_ENTITY_H_

#include <Eigen/Geometry>
#include <haptic_sandbox/tf_scenegraph_object.h>
#include <haptic_sandbox/camera_node.h>
#include <haptic_sandbox/manipulator_node.h>


namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class UserEntity: public tf::SceneGraphNode {


public:
  // Methods only!

  // Constructor
    UserEntity(const std::string &tf_parent_frame_id, const std::string &frame_prefix,
               tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
      : SceneGraphNode(frame_prefix + "frame", tfl, tfb), prefix_(frame_prefix)
    {
        ros::NodeHandle nh;
        update_timer_ = nh.createTimer(ros::Duration(0.01), boost::bind(&UserEntity::update, this));
        changeParentFrameId(tf_parent_frame_id);

        init();
    }

    void init()
    {
        printf("Initializing!\n");
        right_ = new something::ManipulatorNode(prefix_ + "right_workspace", tfl_, tfb_);
        right_->setPosition(tf::Vector3(0, -0.2, 0));

        left_ =  new something::ManipulatorNode(prefix_ + "left_workspace", tfl_, tfb_);
        left_->setPosition(tf::Vector3(0, 0.2, 0));

        view_ = new something::CameraNode(prefix_ + "camera", tfl_, tfb_);
        view_->setPosition(tf::Vector3(-1.0, 0, 0));

        addChild(right_);
        addChild(left_);
        addChild(view_);

        printChildren(true);
    }

    void attachCoupling();

    void update()
    {
        //printf("Timer update!\n");
        ros::Time now = ros::Time::now();
        publishTransformTree(now);
    }

    void changeParentFrameId(const std::string &parent_id)
    {
        transform_.frame_id_ = parent_id;
    }

  // Moves the ENTIRE user entity
  //void moveUserViewFrame();

//  void getGrabbingState()
//  {
//      return right_.getButtonState(grab_button_) || 0; //clutchExternal;
//  }

//  void updateClutch()
//  {
//      // detect a rising edge on the clutch button
//      if (!grabbing_ && getGrabState())
//      {
//          grabbing_ = true;
//          //grab_start_transform_ = workspace_to_handle;
//          grab_start_position_ = right_.getHandlePosition();
//          grab_start_quaternion_ = right_.getHandleQuaternion();
//      }
//      // detect a falling edge on the clutch button
//      else if (grabbing_ && !getGrabState())
//      {
//          grabbing_ = false;
//      }
//      // set the resulting clutched position and orientation of the device
//      if (grabbing_)
//      {
//          workspace_position_   +=  right_.getQuaternion()*(grab_start_position - right_.getHandlePosition());
//          workspace_quaternion_ *=  right_.getQuaternion()*( grab_start_quaternion_ * right_.getHandleOrientation().conjugate());
//      }

//  }

protected:
    // Methods

protected:
    // Members

    // The transform from workspace to handle when we started moving the world
    Transform clutch_start_transform_;

    // Is there any good reason to support n widgets?
    something::ManipulatorNode *right_, *left_;
    something::CameraNode *view_;

    std::string prefix_;

    ros::Timer update_timer_;
};

}  // namespace something

#endif
