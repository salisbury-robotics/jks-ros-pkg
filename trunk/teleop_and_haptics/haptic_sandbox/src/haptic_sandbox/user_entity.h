#ifndef _USER_ENTITY_H_
#define _USER_ENTITY_H_

#include <haptic_sandbox/tf_scenegraph_object.h>
#include <haptic_sandbox/camera_node.h>
#include <haptic_sandbox/manipulator_node.h>

#include <ros/ros.h>
#include <Eigen/Geometry>

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
      : SceneGraphNode(frame_prefix + "frame", tfl, tfb),
        prefix_(frame_prefix), grabbing_(false)
    {
        ros::NodeHandle nh;
        update_timer_ = nh.createTimer(ros::Duration(0.01), boost::bind(&UserEntity::update, this));
        changeParentFrameId(tf_parent_frame_id);

        init();
    }

    virtual ~UserEntity()
    {
        if(right_)  delete right_;
        if(left_)   delete left_;
        if(view_)   delete view_;
    }

    void init()
    {
        printf("Initializing!\n");
        right_ = new something::ManipulatorNode(prefix_ + "right_workspace", tfl_, tfb_);
        right_->setPosition(tf::Vector3(0, -0.2, 0));
        addChild(right_);

        //left_ =  new something::ManipulatorNode(prefix_ + "left_workspace", tfl_, tfb_);
        //left_->setPosition(tf::Vector3(0, 0.2, 0));
        //addChild(left_);

        view_ = new something::CameraNode(prefix_ + "camera", tfl_, tfb_);
        view_->setPosition(tf::Vector3(-0.5, 0, 0));
        addChild(view_);

        grab_start_world_to_handle_.setIdentity();

        //printChildren(true);
        ROS_INFO("Done! Here we go...");
    }

    void attachCoupling();

    void update()
    {
        updateClutch();

        ros::Time now = ros::Time::now();
        publishTransformTree(now);

        publishMarkers(true);
    }

    void changeParentFrameId(const std::string &parent_id)
    {
        transform_.frame_id_ = parent_id;
    }

  // Moves the ENTIRE user entity
  //void moveUserViewFrame();

  bool getGrabState()
  {
      return right_->isGrabbing();
  }

    void updateClutch()
    {
        // detect a rising edge on the clutch button
        if (!grabbing_ && getGrabState())
        {
            grabbing_ = true;
            tfl_->lookupTransform(getParentFrameId(), "user1_right_workspace_device_handle", ros::Time(0), grab_start_world_to_handle_);
        }
        // detect a falling edge on the clutch button
        else if (grabbing_ && !getGrabState())
        {
            grabbing_ = false;
        }
        // set the resulting clutched position and orientation of the device
        if (grabbing_)
        {
            tf::StampedTransform st_handle_to_user;
            tfl_->lookupTransform("user1_right_workspace_device_handle", getFrameId(), ros::Time(0), st_handle_to_user);
            tf::Transform world_to_user;
            tf::Transform world_to_handle(grab_start_world_to_handle_.getRotation(), grab_start_world_to_handle_.getOrigin());
            tf::Transform handle_to_user(st_handle_to_user.getRotation(), st_handle_to_user.getOrigin());
            world_to_user = world_to_handle * handle_to_user;

            setTransform(world_to_user);
        }

  }

protected:
    // Methods

protected:
    // Members

    // The transform from workspace to handle when we started moving the world
    Transform clutch_start_transform_;

    tf::StampedTransform grab_start_world_to_handle_;

    // Is there any good reason to support n widgets?
    something::ManipulatorNode *right_, *left_;
    something::CameraNode *view_;

    std::string prefix_;

    ros::Timer update_timer_;

    bool grabbing_;
};

}  // namespace something

#endif
