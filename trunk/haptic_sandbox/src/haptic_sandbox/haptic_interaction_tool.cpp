#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Geometry>
#include <conversions/tf_chai.h>
#include <chai3d.h>
#include <haptic_sandbox/haptic_interaction_tool.h>



namespace something {

    void HapticInteractionTool::init()
    {
        chai_device_handler_ = new cHapticDeviceHandler();
        chai_device_handler_->getDevice(chai_device_, 0);

        // open a connection with the haptic device
        if(chai_device_->open() != 0)
        {
            ROS_ERROR("Error opening chai device!");
            return;
        }

        cHapticDeviceInfo info = chai_device_->getSpecifications();
        cVector3d pos = info.m_positionOffset;
        setPosition(tf::Vector3(pos.x(), pos.y(), pos.z()));
        setQuaternion(tf::createQuaternionFromYaw(M_PI));


        // TODO this should be a chai thread...!
        ros::NodeHandle nh;
        float update_period = 0.01;
        interaction_timer_ = nh.createTimer(ros::Duration(update_period), boost::bind( &HapticInteractionTool::updateDevice, this ) );
    }

    HapticInteractionTool::~HapticInteractionTool()
    {
        chai_device_->setForceAndTorqueAndGripperForce(cVector3d(0,0,0), cVector3d(0,0,0), 0);
        chai_device_->close();
    }

    // Read the state of the binary switches on the tool.
    bool HapticInteractionTool::getToolButtonState(const unsigned int &index) const
    {
        bool state = false;
        chai_device_->getUserSwitch(index, state);
        return state;
    }

    // Get the number of buttons available on the tool.
    unsigned int HapticInteractionTool::getToolButtonCount() const
    {
        std::string modelName = chai_device_->getSpecifications().m_modelName;

        if(modelName == "PHANTOM Omni") return 2;
        else if(modelName == "Falcon") return 4;
        else if(modelName == "omega") return 1;
        else return 0;
    }



  // Call an update?
  void HapticInteractionTool::updateDevice()
  {
    /////////////////////////////////////////////////////////////////////
    // READ HAPTIC DEVICE
    /////////////////////////////////////////////////////////////////////

    // read position
    cVector3d position;
    chai_device_->getPosition(position);

    // read orientation
    cMatrix3d rotation;
    chai_device_->getRotation(rotation);

    // read gripper position
    double gripperAngle;
    chai_device_->getGripperAngleRad(gripperAngle);

    // read linear velocity
    cVector3d linearVelocity;
    chai_device_->getLinearVelocity(linearVelocity);

    // read angular velocity
    cVector3d angularVelocity;
    chai_device_->getAngularVelocity(angularVelocity);

    // read gripper angular velocity
    double gripperAngularVelocity;
    chai_device_->getGripperAngularVelocity(gripperAngularVelocity);

    tf::Transform handle = tf::Transform(tf::matrixChaiToTf(rotation), tf::vectorChaiToTf(position));
    tf::Transform desired_frame = tf::Transform(tf::createQuaternionFromRPY(0,0,M_PI))*handle;

    handle_->setTransform(handle);


//    // read userswitch status (button 0)
//    bool button0, button1, button2, button3;
//    button0 = false;
//    button1 = false;
//    button2 = false;
//    button3 = false;

//    hapticDevice->getUserSwitch(0, button0);
//    hapticDevice->getUserSwitch(1, button1);
//    hapticDevice->getUserSwitch(2, button2);
//    hapticDevice->getUserSwitch(3, button3);


    cVector3d force, torque;
    float gripperForce = 0;

    // send computed force, torque and gripper force to haptic device
    chai_device_->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

  }



}  // namespace something

