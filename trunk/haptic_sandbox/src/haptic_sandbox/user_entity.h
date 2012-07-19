

#include <Eigen/Geometry>



namespace something {

typedef Eigen::Vector3d Vector3;
typedef Eigen::Quaterniond Quaternion;

class UserEntity{


public:
  // Methods only!

  // Constructor
  UserEntity() {}

  virtual Vector3 getWorkspacePosition() const           { return workspace_transform_.translation(); }
  virtual Quaternion getWorkspaceQuaternion() const      { return workspace_quaternion_.rotation(); }
  virtual Transform getWorkspaceTransform() const      { return workspace_transform_; }

  virtual void setWorkspacePosition(const Vector3 &position);
  virtual void setWorkspaceQuaternion(const Quaternion &quaternion);
  virtual void setWorkspaceTransform(const Transform &transform);


  void attachCoupling();

  // Moves the ENTIRE user entity
  moveUserToViewFrame(const Transform &transform);

  void getGrabbingState()
  {
      return right_.getButtonState(grab_button_) || 0; //clutchExternal;
  }

  void UserEntity::updateClutch()
  {
      // detect a rising edge on the clutch button
      if (!grabbing_ && getGrabState())
      {
          grabbing_ = true;
          //grab_start_transform_ = workspace_to_handle;
          grab_start_position_ = right_.getHandlePosition();
          grab_start_quaternion_ = right_.getHandleQuaternion();
      }
      // detect a falling edge on the clutch button
      else if (grabbing_ && !getGrabState())
      {
          grabbing_ = false;
      }
      // set the resulting clutched position and orientation of the device
      if (grabbing_)
      {
          workspace_position_   +=  right_.getQuaternion()*(grab_start_position - right_.getHandlePosition());
          workspace_quaternion_ *=  right_.getQuaternion()*( grab_start_quaternion_ * right_.getHandleOrientation().conjugate());
      }

  }

protected:
// Methods



protected:
// Members
  Transform workspace_transform_;
  Vector3 workspace_position_;
  Quaternion workspace_quaternion_;

  // Should this be a "user_view" class that also publishes messages?
  Transform camera_transform_;

  // The transform from workspace to handle when we started moving the world
  Transform clutch_start_transform_;

  // Is there any good reason to support n widgets?
  InteractionWidget right_, left_;

};





}  // namespace something
