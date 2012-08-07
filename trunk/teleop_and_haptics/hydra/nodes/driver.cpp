#include <string>
#include <ros/ros.h>
#include "hydra.h"
#include "hydra/Raw.h"
#include "hydra/Calib.h"
#include "tf/tf.h"

// Visualization
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

using namespace hydra;
using std::string;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hydra_driver");
    ros::NodeHandle n, n_private("~");

    // Get configuration params
    string device;
    n_private.param<std::string>("device", device, "/dev/hidraw0");
    bool publish_tf = false;
    n_private.param<bool>("publish_tf", publish_tf, false);

    // Initialize ROS stuff
    ros::Publisher raw_pub = n.advertise<hydra::Raw>("hydra_raw", 1);
    ros::Publisher calib_pub = n.advertise<hydra::Calib>("hydra_calib", 1);
    tf::TransformBroadcaster *broadcaster = 0;
    if(publish_tf) broadcaster = new tf::TransformBroadcaster();


    Hydra hydra;
    ROS_INFO("opening hydra on %s", device.c_str());
    if (!hydra.init(device.c_str()))
    {
        ROS_FATAL("couldn't open hydra on %s", device.c_str());
        return 1;
    }
    ROS_INFO("starting stream...");
    while (n.ok())
    {
        if (hydra.poll(10))
        {
            hydra::Raw msg;
            msg.header.stamp = ros::Time::now();
            for (int i = 0; i < 6; i++)
                msg.pos[i] = hydra.raw_pos[i];
            for (int i = 0; i < 8; i++)
                msg.quat[i] = hydra.raw_quat[i];
            for (int i = 0; i < 2; i++)
                msg.buttons[i] = hydra.raw_buttons[i];
            for (int i = 0; i < 6; i++)
                msg.analog[i] = hydra.raw_analog[i];
            raw_pub.publish(msg);

            hydra::Calib c_msg;
            c_msg.header.stamp = msg.header.stamp;
            for (int i = 0; i < 2; i++)
                tf::transformTFToMsg(tf::Transform(hydra.quat[i], hydra.pos[i]),
                                 c_msg.paddles[i].transform);
            for (int i = 0; i < 7; i++)
            {
                c_msg.paddles[0].buttons[i] = hydra.buttons[i];
                c_msg.paddles[1].buttons[i] = hydra.buttons[i+7];
            }
            for (int i = 0; i < 2; i++)
            {
                c_msg.paddles[0].joy[i] = hydra.analog[i];
                c_msg.paddles[1].joy[i] = hydra.analog[i+3];
            }
            c_msg.paddles[0].trigger = hydra.analog[2];
            c_msg.paddles[1].trigger = hydra.analog[5];
            calib_pub.publish(c_msg);

            if(broadcaster)
            {
                std::vector<geometry_msgs::TransformStamped> transforms;
                transforms.resize(2);
                geometry_msgs::TransformStamped ts;

                std::string frames[2] = {"hydra_left", "hydra_right"};
                for(int kk = 0; kk < 2; kk++)
                {
                    transforms[kk].transform = c_msg.paddles[kk].transform;
                    transforms[kk].header.stamp = c_msg.header.stamp;
                    transforms[kk].header.frame_id = "hydra_base";
                    transforms[kk].child_frame_id = frames[kk];
                }
                broadcaster->sendTransform(transforms);
            }

            ros::spinOnce();
        }
    }

    // clean up
    if(broadcaster) delete broadcaster;
    return 0;
}
