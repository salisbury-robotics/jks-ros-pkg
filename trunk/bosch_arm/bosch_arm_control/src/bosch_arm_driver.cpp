/**
A ros wrapper for bosch_arm_driver.
It read from port 10050
It publishes joint state
It listens to command messages from ros
It sends the command to port 10051
**/
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include "bosch_arm_srvs/SetJointAngles.h"
#include "sensor_msgs/JointState.h"
#include <boost/thread/thread.hpp>
using namespace std;

ros::Publisher joint_state_pub;

bool set_joint_angles_srv(bosch_arm_srvs::SetJointAngles::Request &req,
                          bosch_arm_srvs::SetJointAngles::Response &res)
{
    int hSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (hSock <=-1)
        return 1;
    struct hostent *pServer = gethostbyname("localhost");
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    memcpy(&addr.sin_addr.s_addr, pServer->h_addr, pServer->h_length);
    addr.sin_port = htons(10051);
    char buf[100];
    sprintf(buf,"%%g90j1p%fj2p%fj3p%fj4p%f",req.joint_angles[0],req.joint_angles[1],
            req.joint_angles[2],req.joint_angles[3]);
    int n = sendto(hSock, buf, strlen(buf), 0, (sockaddr*)&addr, sizeof(addr));
    if (n < 0) {
        ROS_ERROR("Sending command %s failed.",buf);
        shutdown(hSock, SHUT_RDWR);
        close(hSock);
        return false;
    }
    return true;
}

void pubJointStates() {

    char cmdbuf[512];
    int buflen = 512;
    int port = 10050;
    struct sockaddr_in  si_me, si_other;
    int s;
    socklen_t slen = sizeof (si_other);

    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
        ROS_ERROR("Fail to open joint state socket.");
        exit(-1);
    }
    int flags = fcntl(s, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(s, F_SETFL, flags);
    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s,(struct sockaddr *) &si_me, sizeof(si_me))==-1) {
        ROS_ERROR("Fail to bind joint state socket.");
        exit(-1);
    }
    sensor_msgs::JointState js;
    js.name.resize(4);
    js.name[0]="joint1";
    js.name[1]="joint2";
    js.name[2]="joint3";
    js.name[3]="joint4";
    int seq=0;
    /*
    out << q1 << "," << q2 << "," << q3 << "," << q4 << ",";     // Motor positions
    out << q1d << "," << q2d << "," << q3d << "," << q4d << ","; // Commanded Motor positions
    out << v1 << "," << v2 << "," << v3 << "," << v4 << ",";     // Motor velocities
    out << torque1 << "," << torque2 << "," << torque3 << "," << torque4 << ","; // Motor torques
    out << ts.tv_sec << ','<< ts.tv_nsec;                         // Servo cycle count, timestamp
    */
    while (ros::ok()) {
        memset(cmdbuf,'\0',buflen);
        int size  = recvfrom(s, cmdbuf, buflen, 0, (struct sockaddr *) &si_other, &slen);
		if(size<=0){
			usleep(500);
			continue;
		}
//         if (seq%10==0)
//             printf("%s\n",cmdbuf);
        char *pch;
        pch=strtok(cmdbuf,",");
        js.position.resize(4);
        for (int i=0;i<4;i++) {
// 			if(pch==NULL){
// 				ROS_INFO("Drop corrupted message.");
// 				continue;
// 			}
            js.position[i]=atof(pch);
            pch=strtok(NULL,",");
        }
        for (int i=0;i<4;i++)

            pch=strtok(NULL,",");
        js.velocity.resize(4);
        for (int i=0;i<4;i++) {

            js.velocity[i]=atof(pch);
            pch=strtok(NULL,",");
        }
        js.effort.resize(4);
        for (int i=0;i<4;i++) {
            js.effort[i]=atof(pch);
            pch=strtok(NULL,",");
        }
        js.header.stamp.sec=atol(pch);
        pch=strtok(NULL,",");
        js.header.stamp.nsec=atol(pch);
        js.header.seq=seq++;
        js.header.frame_id="";
        joint_state_pub.publish(js);
    }
    close(s);
    return;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bosch_arm_node");
    ros::NodeHandle n;
    joint_state_pub =  n.advertise<sensor_msgs::JointState>("/joint_states",100);
    ros::ServiceServer service = n.advertiseService("set_joint_angles", set_joint_angles_srv);
    boost::thread t2 = boost::thread::thread(boost::bind(&pubJointStates));
    ros::spin();
    t2.interrupt();
    t2.join();
    return 0;
}
