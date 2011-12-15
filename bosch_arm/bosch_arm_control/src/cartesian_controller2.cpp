#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "bosch_arm.h"
#include <kdl/frames.hpp>
#include "cartesian_controller2.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
#include <sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include "cc2.h"
#include <Eigen/Core>
#include<Eigen/Array>
#include <Eigen/LU>
#include <Eigen/QR>
#include <action_cart.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/kd.h>


using namespace std;
void CartesianController::singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg)
{
    MoveAbsAction* a=new MoveAbsAction(msg.coordinates,this,msg.duration);
    act_que.push_back(a);
}

void CartesianController::GMSCallBack(const GMS120::GMS120_measurement& msg)
{


    tf::StampedTransform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "world"));

    pc.header.stamp = ros::Time::now();
    geometry_msgs::Point32 point;
    point.x=x[0];
    point.y=x[1];
    //point.z=x[2];
    //point.z=msg.capacitance_measured/2000.0;
    point.z=msg.inductance/300;
    if(msg.inductance>0)
    {
    pc.points.push_back(point);
    //pc.channels[0].values.push_back(msg.inductance);
    pc.channels[0].values.push_back(100);
//     if (msg.inductance>mu&&msg.inductance<last_inductance&&state!=SERVO)
//     {
//         state=SERVO;
//         char ascii=7;
//         cout<<"inductance:"<<msg.inductance<<", last inductance:"<<last_inductance<<". Stopped."<<endl;
//         //cout<<'\a'<<flush;
// 
//     }
    last_inductance=msg.inductance;
    point_pub.publish(pc);
    }
    //cout<<msg.inductance<<endl;
}
CartesianController::CartesianController(BoschArm *ptr)
{
    rob=ptr;
    diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ("/diagnostics",100);
    point_pub=n.advertise<sensor_msgs::PointCloud>("/gmsdata",100);
    sigle_point_cmd_sub = n.subscribe("/joint_point_cmd",3, &CartesianController::singlePtCmdCallBack, this);
    GMS_sub = n.subscribe("/GMS120_measurements",3,&CartesianController::GMSCallBack,this);
    joint_state_pub =  n.advertise<bosch_arm_control::ArmState> ("/joint_states",100);
    js.position.resize(3);
    js.velocity.resize(3);
    js.effort.resize(3);

    pc.header.frame_id = "/world";
    pc.points.resize(0);
    pc.channels.resize(1);
    pc.channels[0].values.resize(0);
    pc.channels[0].name = "intensity";


    Kp[0]=80;
    Kp[1]=80;
    Kp[2]=80;

    Kc[0]=0.3;
    Kc[1]=0.3;
    Kc[2]=0.3;
    Kc[3]=0.3;
//    for(int i=0;i<4;i++)
//      Kp[i]=Kp[i]*0.02;
    Kv[0]=0.3;
    Kv[1]=0.3;
    Kv[2]=0.3;

    Ks[0]=0.05;
    Ks[1]=0.02;
    Ks[2]=0.30;
    mu=2.6;
//   for(int i=0;i<4;i++)
//     Kv[i]=0;
//   joint_limit_low[0]=-1.7;
//   joint_limit_high[0]=2.1;
//   joint_limit_low[1]=-0.6;
//   joint_limit_high[1]=0.55;
//   joint_limit_low[2]=-2.5;
//   joint_limit_high[2]=2.3;
//   joint_limit_low[3]=-2.3;
//   joint_limit_high[3]=2.4;
    cur_act=NULL;
    start_semi=false;
    end_semi=false;
    last_inductance=0;
    inductance_sum_last=0;
    cnta=0;
    n_cur=0;
    radius=0.04;
    cycle=4000;
    fd = open("/dev/console", O_WRONLY);
}

//calculate the frames given the joint values.
void CartesianController::updateKinematics()
{
    frm_cur[0]=frm_home[0];
    frm0[0]=frm_home[0];
    for (int i=1;i<=4;i++)
    {
        frm_cur[i].p=frm_home[i].p;
        frm_cur[i].M=frm_home[i].M*Rotation::RotZ(q[i-1]);
        frm0[i]=frm0[i-1]*frm_cur[i];
    }
    frm_cur[5]=frm_home[5];
    frm0[5]=frm0[4]*frm_cur[5];

    Vector z(0,0,1);
    Vector mx=frm0[5].M.UnitZ()*z;
    double norm=dot(mx,mx);
    if (norm<1e-4)
    {
        mx=z*frm0[5].M.UnitX();
        double n2=dot(mx,mx);
        mx=mx/sqrt(n2);
    }
    else
        mx=mx/sqrt(norm);
    Vector my=z*mx;

    frm0[6].M=Rotation(mx,my,z);

    frm0[6].p=frm0[6].M*scanner_p+frm0[5].p;


    frmn[4]=frm_cur[5];


    //jacobian[3](joint 4) = z4*p54
    for (int i=3;i>=0;i--)
    {
        frmn[i]=frm_cur[i+1]*frmn[i+1];
        jacobian[i]=z*frmn[i+1].p;
        jacobian[i]=frm0[i+1].M*jacobian[i];
    }
    Eigen::MatrixXf jcb(3,4);
    for (int i=0;i<3;i++)
        for (int j=0;j<4;j++)
            jcb(i,j)=jacobian[j][i];

    Eigen::Matrix3f mm=jcb*jcb.transpose();
    //cout<<mm<<endl;
    Eigen::Matrix3f inv=mm.inverse();
    //cout<<inv<<endl;
    nullspace=Eigen::Matrix4f::Identity()-jcb.transpose()*inv*jcb;
    //cout<<nullspace<<endl;
//
//   for(int i=0;i<3;i++)
//     for(int j=0;j<3;j++)
//     {
//       mm(i,j)=0;
//       for(int k=0;k<4;k++)
//         mm(i,j)=mm(i,j)+jacobian[k][i]*jacobian[k][j];
//     }
//   mm=mm.inverse();

}

void CartesianController::start()
{
    rob->initialize();

    for (int i=0;i<6;i++)
    {
        frm_home[i].p=Vector(constants::home_frame_p[i*3],
                             constants::home_frame_p[i*3+1],
                             constants::home_frame_p[i*3+2]);

        frm_home[i].M=Rotation(constants::home_frame_M[i*9],
                               constants::home_frame_M[i*9+1],
                               constants::home_frame_M[i*9+2],
                               constants::home_frame_M[i*9+3],
                               constants::home_frame_M[i*9+4],
                               constants::home_frame_M[i*9+5],
                               constants::home_frame_M[i*9+6],
                               constants::home_frame_M[i*9+7],
                               constants::home_frame_M[i*9+8]);
    }
    scanner_p=Vector(constants::scan_p[0],
                     constants::scan_p[1],
                     constants::scan_p[2]);

    for (int i=0;i<4;i++)
        gc[i]=Vector(constants::gravc[i*2],constants::gravc[i*2+1],0);

    rob->motor2JointPosition(rob->q,q);
    updateKinematics();
    for (int i=0;i<3;i++)
    {
        x[i]=frm0[5].p[i];
        v[i]=0;
        s[i]=0;
        dx[i]=0;
        f[i]=0;
        xl[i]=x[i];
        xd[i]=x[i];
        vd[i]=v[i];
    }
    //state=CALIBRATION;
    //state=SEMIAUTO;
    state=SERVO;
    ctr_mode=NOCONTROL;

    if (state==CALIBRATION)
    {
        genCalibTraj2();
        cur_act=act_que.front();
        cur_act->initialize();
        act_que.pop_front();
        cout<<"start in calibration"<<endl;
    }
    else if (state==SEMIAUTO)
    {
        vector<double> des(3,0);
        cali1_act=new MoveAbsAction(des,this);
        calib1=new Calibration("calibsemi");
        cali1_act->calib=calib1;
        act_que.push_back(cali1_act);
        cur_act=act_que.front();
        cur_act->initialize();
        act_que.pop_front();
        cout<<"start in semi auto"<<endl;
    }
    else if (state==LOADTRACE)
    {
//     for (int i=0;i<4;i++)
//       gc[i]=Vector(constants::gravc[i*2],constants::gravc[i*2+1],0);
//     vector<double> des(3,0);
//     des[0]=0.5;
//     des[1]=0;
//     des[2]=0.3;
//     act_que.push_back(new MoveAbsAction(des,this));
//     des[0]=0;
//     des[1]=0;
//     des[2]=1.2;
//     act_que.push_back(new MoveAbsAction(des,this));
//     genCalibTraj2();
//     cur_act=act_que.front();
//     cur_act->initialize();
//     act_que.pop_front();

        //state=SERVO;

    }
}

// void CartesianController::genCalibTraj2()
// {
//
//   vector<double> des(3,0);
// //  int t=8000;
//   //move the the zero position.
//   des[0]=0.30;
//   des[1]=-0.5;
//   des[2]=0.10;
//   act_que.push_back(new MoveAbsAction(des,this));
//   des[0]=0.19;
//   des[1]=-0.18;
//   des[2]=-0.05;
//   act_que.push_back(new MoveAbsAction(des,this));
//   des[0]=0.54;
//   des[1]=-0.18;
//   des[2]=-0.05;
//   cali1_act=new MoveAbsAction(des,this);
//   calib1=new Calibration("calib1");
//   cali1_act->calib=calib1;
//   act_que.push_back(cali1_act);
//   des[0]=0.54;
//   des[1]=0.07;
//   des[2]=-0.05;
//   cali2_act=new MoveAbsAction(des,this);
//   calib2=new Calibration("calib2");
//   cali2_act->calib=calib2;
//   act_que.push_back(cali2_act);
//
//
// }

void CartesianController::stopTrajectory()
{
    cout<<"stop scanning"<<endl;
    state=SERVO;
}

void CartesianController::genCalibTraj2()
{
    vector<double> des(3,0);
    while (act_que.size()!=0)
        act_que.pop_front();

    des[0]=0.5;
    des[1]=0.2;
    des[2]=0;
    cali1_act=new MoveAbsAction(des,this);
    cali1_act->t_end=2000;
    act_que.push_back(cali1_act);

    des[0]=0.5;
    des[1]=-0.3;
    des[2]=0;
    cali1_act=new MoveAbsAction(des,this);
    cali1_act->t_end=8000;
    act_que.push_back(cali1_act);

    cur_act=act_que.front();
    cur_act->initialize();
    act_que.pop_front();
    cout<<"start trajectory"<<endl;
    state=LOADTRACE;
}

// void CartesianController::genCalibTraj2()
// {
//
//   vector<double> des(3,0);
// //  int t=8000;
//   //move the the zero position.
// //   des[0]=0.35;
// //   des[1]=0;
// //   des[2]=0;
// //   act_que.push_back(new MoveAbsAction(des,this));
// //   act_que.back()->t_end=2000;
//
//   //double zz=0.03
//
//   des[0]=0.55;
//   des[1]=0;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=2000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.35;
//   des[1]=-0.05;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.55;
//   des[1]=-0.1;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.35;
//   des[1]=-0.15;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.55;
//   des[1]=-0.2;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.35;
//   des[1]=-0.2;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.55;
//   des[1]=-0.15;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.35;
//   des[1]=-0.1;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.55;
//   des[1]=-0.05;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//   des[0]=0.35;
//   des[1]=-0.0;
//   des[2]=0;
//   cali1_act=new MoveAbsAction(des,this);
//   cali1_act->t_end=8000;
//   act_que.push_back(cali1_act);
//
//
//   des[0]=0.55;
//   des[1]=-0.0;
//   des[2]=0;
//   cali2_act=new MoveAbsAction(des,this);
//   cali2_act->t_end=8000;
//   act_que.push_back(cali2_act);
//
//
//
//
//   cur_act=act_que.front();
//       cur_act->initialize();
//       act_que.pop_front();
//       cout<<"start trajectory"<<endl;
//   state=LOADTRACE;
// }


void CartesianController::calcLinkLength()
{

    int bin=calib1->bin;
    double* A[8];
    for (int i=0;i<8;i++)
        A[i]=new double[bin];
    double* b;
    b=new double[bin];

    for (int tt=0;tt<calib1->bin;tt++)
    {
        for (int i=0;i<8;i++)
            A[i][tt]=calib1->A[i][tt];
        b[tt]=calib1->b[tt];
    }


    Eigen::MatrixXf AA(8,8);
    for (int i=0;i<8;i++)
    {
        for (int j=0;j<8;j++)
        {
            double s=0;
            for (int k=0;k<bin;k++)
                s+=A[i][k]*A[j][k];
            AA(i,j)=s;
        }
    }
    cout<<AA<<endl;
    Eigen::VectorXf Ab(8);
    for (int i=0;i<8;i++)
    {
        double s=0;
        for (int j=0;j<bin;j++)
            s+=A[i][j]*b[j];
        Ab(i)=s;
    }
    cout<<Ab<<endl;
    Eigen::MatrixXf AV=AA.inverse();
    cout<<AV<<endl;
    Eigen::MatrixXf M=AV*Ab;
    //cout<<M<<endl;
//   for(int i=0;i<4;i++)
//   {
//     link[i]=Vector(M(i*2,0),M(i*2+1,0),0);
//   }
//   for(int i=0;i<4;i++)
//   {
//     for(int j=0;j<3;j++)
//       cout<<link[i][j]<<',';
//     cout<<endl;
//   }
}

void CartesianController::update()
{
    //n_cur++;
    rob->update();
    clock_gettime(CLOCK_REALTIME, &ts);
    //save the last joint position
//   for (int i=0;i<4;i++)
//     ql[i]=q[i];
    rob->motor2JointPosition(rob->q,q);
    rob->motor2JointVelocity(rob->v,qv);
    updateKinematics();
    for (int i=0;i<3;i++)
    {
        x[i]=frm0[5].p[i];
        v[i]=0;
        for (int j=0;j<4;j++)
            v[i]+=jacobian[j][i]*qv[j];
    }
    if (state==CALIBRATION)
    {
        //update desired position
        if (cur_act->finished)
        {
            if (cur_act->calib==NULL)
                delete cur_act;

            if (act_que.size()==0)
            {
                //calcLinkLength();
                state=SERVO;
                return;
            }
            cur_act=act_que.front();
            //cout<<cur_act->rel[3]<<endl;
            cur_act->initialize();
            act_que.pop_front();
        }
        cur_act->update();
        for (int i=0;i<3;i++)
        {
            xd[i]=cur_act->cmd.coordinates[i];
            vd[i]=cur_act->cmd.velocity[i];

        }
        //update torque command
        PDWithGC();

        logging();
    }
    else if (state==SEMIAUTO)
    {
        if (start_semi==true)
            cur_act->update2();
        if (end_semi==true)
        {
            state=SERVO;
            cur_act->calib->finish();
            calcLinkLength();
        }
        gravity_compensation();
        logging();
    }
    else if (state==LOADTRACE)
    {
        if (cur_act==NULL&&act_que.size()==0)
        {
            state=SERVO;
            return;
        }
        if (cur_act->finished)
        {
            if (cur_act->calib==NULL)
                delete cur_act;

            if (act_que.size()==0)
            {
                //calcGravityCompensation();
                state=SERVO;
                return;
            }

            cur_act=act_que.front();
            //cout<<cur_act->rel[3]<<endl;
            cur_act->initialize();
            act_que.pop_front();

        }
        cur_act->update();
        for (int i=0;i<3;i++)
        {
            xd[i]=cur_act->cmd.coordinates[i];
            vd[i]=cur_act->cmd.velocity[i];

        }
        //update torque command
        PDWithGC();
        //PDControl();
        logging();
    }
    else if (state==SERVO)
    {
        //PDControl();
        if (ctr_mode==PDWITHGC)
            PDWithGC();
        else if (ctr_mode==FULLSCAN)
            full_scan();
        else if (ctr_mode==PEAKFINDER)
          peak_finder();
            //PDControl();
        else if (ctr_mode==GRAVITY)
            gravity_compensation();
        else if (ctr_mode==NOCONTROL)
        {
            for (int i=0;i<4;i++)
                rob->torque[i]=tzero[i];
        }
        //floating();
        //gravity_compensation();
        logging();
    }
}

void CartesianController::startPDWithGC()
{
    ctr_mode=CartesianController::PDWITHGC;
    //space='m';
    for (int i=0;i<3;i++)
    {
        xd[i]=x[i];
        v[i]=0;
        s[i]=0;
        vd[i]=0;
    }
    cout<<"PD control(GC) in Cartesian space"<<endl;
}

void CartesianController::start_peak_finder()
{
    ctr_mode=CartesianController::PEAKFINDER;
    //space='m';
    for (int i=0;i<3;i++)
    {
        xd[i]=x[i];
        dx[i]=0;
        //dv[i]=0;
        v[i]=0;
        s[i]=0;
        vd[i]=0;
    }
    n_sum=0;
    n_cur=0;
    cnta=0;
    inductance_sum_last=0;
    search_mode=0;
    //for circular sweeping
    double phase=2.0*M_PI*(n_cur%cycle)/cycle;
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            AA(i,j)=0;
    for (int i=0;i<3;i++)
        Ab(i)=0;
    
    center_x=x[0]-cos(phase)*radius;
    center_y=x[1]-sin(phase)*radius;
    cout<<"start sweeping the surface."<<endl;
}

void CartesianController::start_fullscan()
{
    ctr_mode=CartesianController::FULLSCAN;
    //space='m';
    for (int i=0;i<3;i++)
    {
        xd[i]=x[i];
        dx[i]=0;
        //dv[i]=0;
        v[i]=0;
        s[i]=0;
        vd[i]=0;
    }
    n_cur=0;

    cout<<"start sweeping the surface."<<endl;
}


void CartesianController::full_scan()
{
    n_cur++;
    updateKinematics();
    double width=0.25;
    double interval=0.01;
    double speed=0.05*1e-3;
    int nw=width/speed;
    int ni=interval/speed;
    int n_all=2*(nw+ni);
    int phase=n_cur%n_all;
    double eh=sqrt(dx[0]*dx[0]+dx[1]*dx[1]);
    if(last_inductance>2)
    {
        int freq = 2000; // freq in hz
  int len = 1; // len in ms

  //int fd = open("/dev/console", O_WRONLY);
  ioctl(fd, KIOCSOUND, (int)(1193180/freq));
  usleep(len);
  ioctl(fd, KIOCSOUND, 0);
  //close(fd);
    }
    //Ks[2]+=(eh-Ks[1])*fabs(eh-Ks[1])*Ks[0];
    if (phase<nw)
    {
        xd[0]-=speed;
    }
    else if (phase<nw+ni)
    {
        xd[1]-=speed;
    }
    else if (phase<nw+ni+nw)
    {
        xd[0]+=speed;
    }
    else
    {
        xd[1]-=speed;
    }

    xd[2]+=(eh-Ks[1])*fabs(eh-Ks[1])*Ks[0];
    //upper limit to avoid instability.
    if(xd[2]>Ks[2])
      xd[2]=Ks[2];
    //xd[2]=Ks[2]+eh*Ks[1];
    //if(xd[2]>Ks[2])
    //xd[2]=Ks[2];

    for (int i=0;i<3;i++)
        vd[i]=0;

    for (int i=0;i<3;i++)
    {

        dx[i]=xd[i]-x[i];
        f[i]=Kp[i]*dx[i]-Kv[i]*(v[i]-vd[i]);
    }

    for (int i=0;i<4;i++)
    {
        tq[i]=0;
        for (int j=0;j<3;j++)
        {
            tq[i]+=jacobian[i][j]*f[j];
        }

    }

    //Gravity compensation
    //T=sum(gcj) dot (g*Zi)
    Vector g(0,0,-1);
    for (int i=0;i<4;i++)
    {
        Vector Zi=frm0[i+1].M.UnitZ();
        for (int j=i;j<4;j++)
        {
            Rotation R0j=frm0[j+1].M.Inverse();
            //coordinates of vector g*Zi in frame j
            Vector tmp=R0j*(g*Zi);
            tq[i]-=dot(gc[j],tmp);
        }
    }

    //resolve redundancy: away from joint limits.
    Eigen::Vector4f c;
    for (int i=0;i<4;i++)
        c(i)=-q[i];
    Eigen::Vector4f cf=nullspace*c;
    for (int i=0;i<4;i++)
        tq[i]+=Kc[i]*cf(i);

    //project force into motor space
    for (int i=0;i<4;i++)
    {
        tm[i]=0;
        for (int j=0;j<4;j++)
        {
            tm[i]+=rob->m2j[4*j+i]*tq[j];

        }

    }
    for (int i=0;i<4;i++)
        rob->torque[i]=tm[i];
}

/* This function sweeps the table surface in a zigzag manner
b=Ax
Eigen::MatrixXf AA(8,8);
Eigen::VectorXf Ab(8);
AAij=sum(xi*xj)
Ab=sum(xi*bi)
*/
void CartesianController::peak_finder()
{
    updateKinematics();
    n_cur++;
//  int cycle=int(mu);
//   double z_des=Ks[2];
//   double x_force=Ks[0];
//   double y_force=Ks[1];
//   //use position control in z direction
//   dx[2]=z_des-x[2];
//   f[2]=Kp[2]*dx[2]-Kv[2]*v[2];
//
//   f[0]=(n_cur%cycle>cycle/2)?x_force:-x_force;
//   f[1]=-y_force;

//   double vel_x=Ks[0]*1e-3;
//   double vel_y=Ks[1]*1e-3;
//   //double pos_z=Ks[2];
//   if(n_cur%cycle>=cycle/2)
//     vd[0]=vel_x;
//   else
//     vd[0]=-vel_x;
//   vd[1]=-vel_y;
//   vd[2]=0;
//   for(int i=0;i<2;i++)
//     xd[i]+=vd[i];
//     if(last_inductance>30)
//     {
//         int freq = 2000; // freq in hz
//   int len = 1; // len in ms
// 
//   int fd = open("/dev/console", O_WRONLY);
//   ioctl(fd, KIOCSOUND, (int)(1193180/freq));
//   usleep(len);
//   ioctl(fd, KIOCSOUND, 0);
//   close(fd);
//     }

    double phase=2.0*M_PI*(n_cur%cycle)/cycle;
    double eh=sqrt(dx[0]*dx[0]+dx[1]*dx[1]);
    //Ks[2]+=(eh-0.03)/2;
    center_x+=dcx/cycle;
    center_y+=dcy/cycle;
    xd[0]=center_x+cos(phase)*radius;
    xd[1]=center_y+sin(phase)*radius;
    //xd[2]=Ks[2];
    xd[2]+=(eh-Ks[1])*fabs(eh-Ks[1])*Ks[0];
    //upper limit to avoid instability.
    if(xd[2]>Ks[2])
      xd[2]=Ks[2];
    int num_pts=20;
    int sample_cycle=cycle/num_pts;
    if (n_cur%sample_cycle==0)
    {
        AA(0,0)+=x[0]*x[0];
        //cout<<AA(0,0)<<endl;
        AA(0,1)+=x[0]*x[1];
        AA(0,2)+=x[0];
        AA(1,0)+=x[1]*x[0];
        AA(1,1)+=x[1]*x[1];
        AA(1,2)+=x[1];
        AA(2,0)+=x[0];
        AA(2,1)+=x[1];
        AA(2,2)+=1;
        Ab(0)+=x[0]*last_inductance;
        Ab(1)+=x[1]*last_inductance;
        Ab(2)+=last_inductance;
        n_sum++;
        //cout<<last_inductance<<',';
    }
    
    if (n_sum==num_pts)//has finished a circle
    {
       // cout<<endl;
        //cout<<AA<<endl<<endl;
        //cout<<Ab<<endl<<endl;
        //Eigen::Matrix3f inv=AA.inverse();
        //double det=AA.determinant();
        //cout<<"determinant:"<<AA.determinant()<<endl;
        //if (det>0.0025)
        
     
//         if (search_mode==0)
//         {
            if (Ab(2)>=num_pts)
            {
                
                if (Ab(2)>inductance_sum_last)
                {
                    
                    M=AA.inverse()*Ab;
                    //cout<<M<<endl;
                    dcx=M(0,0)*6e-5;
                    dcy=M(1,0)*6e-5;
                    if (dcx>0.02)
                        dcx=0.02;
                    else if(dcx<-0.02)
                      dcx=-0.02;
                    if (dcy>0.02)
                        dcy=0.02;
                    else if(dcy<-0.02)
                      dcy=-0.02;
                    //center_x+=dcx;
                    //center_y+=dcy;
                    inductance_sum_last=Ab(2);
                    cnta=0;
                    //cout<<"update center(gradient):"<<center_x<<","<<center_y<<". inductance sum:"<<Ab(2)<<endl;
                    cout<<"inductance sum:"<<Ab(2)<<". direction:"<<dcx<<","<<dcy<<endl;
                }
                else
                {
//                     cnta++;
//                     if (cnta>3)
//                     {
//                         dcx=center_x-x[0];
//                         dcy=center_y-x[1];
//                         search_mode=1;
//                         cout<<"moving to the center"<<endl;
//                     }
//                     else
//                     {
//                       //cout<<"wait"<<". inductance sum:"<<Ab(2)<<endl;
                      dcx=0;
                      dcy=0;
                      cout<<"inductance sum:"<<Ab(2)<<". direction:"<<dcx<<","<<dcy<<endl;
//                    }
                }
                

            }
            else if (Ab(2)<num_pts)
            {
                //center_x+=0e-5;
                //center_y+=-1e-2;
                dcx=0;
                dcy=-0.01;
                //cout<<"update center(-y):"<<center_x<<","<<center_y<<". inductance sum:"<<Ab(2)<<endl;
                cout<<"inductance sum:"<<Ab(2)<<". direction:"<<dcx<<","<<dcy<<endl;
            }
       // }

        
        for (int i=0;i<3;i++)
                for (int j=0;j<3;j++)
                    AA(i,j)=0;
            for (int i=0;i<3;i++)
                Ab(i)=0;
        n_sum=0;
    }

    for (int i=0;i<3;i++)
        vd[i]=0;

    for (int i=0;i<3;i++)
    {

        dx[i]=xd[i]-x[i];
        f[i]=Kp[i]*dx[i]-Kv[i]*(v[i]-vd[i]);
    }

    for (int i=0;i<4;i++)
    {
        tq[i]=0;
        for (int j=0;j<3;j++)
        {
            tq[i]+=jacobian[i][j]*f[j];
        }

    }

    //Gravity compensation
    //T=sum(gcj) dot (g*Zi)
    Vector g(0,0,-1);
    for (int i=0;i<4;i++)
    {
        Vector Zi=frm0[i+1].M.UnitZ();
        for (int j=i;j<4;j++)
        {
            Rotation R0j=frm0[j+1].M.Inverse();
            //coordinates of vector g*Zi in frame j
            Vector tmp=R0j*(g*Zi);
            tq[i]-=dot(gc[j],tmp);
        }
    }

    //resolve redundancy: away from joint limits.
    Eigen::Vector4f c;
    for (int i=0;i<4;i++)
        c(i)=-q[i];
    Eigen::Vector4f cf=nullspace*c;
    for (int i=0;i<4;i++)
        tq[i]+=Kc[i]*cf(i);

    //project force into motor space
    for (int i=0;i<4;i++)
    {
        tm[i]=0;
        for (int j=0;j<4;j++)
        {
            tm[i]+=rob->m2j[4*j+i]*tq[j];

        }

    }
    for (int i=0;i<4;i++)
        rob->torque[i]=tm[i];
}

void CartesianController::gravity_compensation()
{

    updateKinematics();
    //T=sum(gcj) dot (g*Zi
    Vector g(0,0,-1);
    //double tq[4];
    for (int i=0;i<4;i++)
    {
        Vector Zi=frm0[i+1].M.UnitZ();
        tq[i]=0;
        for (int j=i;j<4;j++)
        {
            Rotation R0j=frm0[j+1].M.Inverse();
            Vector tmp=R0j*(g*Zi);
            tq[i]-=dot(gc[j],tmp);
        }
    }

    for (int i=0;i<4;i++)
    {
        tm[i]=0;
        for (int j=0;j<4;j++)
        {
            tm[i]+=rob->m2j[4*j+i]*tq[j];

        }

    }
    for (int i=0;i<4;i++)
        rob->torque[i]=tm[i];
    //logging();

}

void CartesianController::PDWithGC()
{
    updateKinematics();
    //PID control in cartesian space
    for (int i=0;i<3;i++)
    {

        dx[i]=xd[i]-x[i];
        f[i]=Kp[i]*dx[i]-Kv[i]*(v[i]-vd[i]);
        s[i]=s[i]+dx[i];
        f[i]+=Ks[i]*s[i];
    }
    //for basket tip:modulating the normal force according to the horizontal error.
//   double eh=sqrt(dx[0]*dx[0]+dx[1]*dx[1]);
//   f[2]-=Kp[2]*eh*mu;
    //project force into joint space
    for (int i=0;i<4;i++)
    {
        tq[i]=0;
        for (int j=0;j<3;j++)
        {
            tq[i]+=jacobian[i][j]*f[j];
            //cout<<jacobian[4*j+i]<<',';
        }
        // cout<<endl;
    }

    //Gravity compensation
    //T=sum(gcj) dot (g*Zi)
    Vector g(0,0,-1);
    for (int i=0;i<4;i++)
    {
        Vector Zi=frm0[i+1].M.UnitZ();
        for (int j=i;j<4;j++)
        {
            Rotation R0j=frm0[j+1].M.Inverse();
            //coordinates of vector g*Zi in frame j
            Vector tmp=R0j*(g*Zi);
            tq[i]-=dot(gc[j],tmp);
        }
    }

    //resolve redundancy: away from joint limits.
    Eigen::Vector4f c;
    for (int i=0;i<4;i++)
        c(i)=-q[i];
    Eigen::Vector4f cf=nullspace*c;
    for (int i=0;i<4;i++)
        tq[i]+=Kc[i]*cf(i);

    //project force into motor space
    for (int i=0;i<4;i++)
    {
        tm[i]=0;
        for (int j=0;j<4;j++)
        {
            tm[i]+=rob->m2j[4*j+i]*tq[j];

        }

    }
    for (int i=0;i<4;i++)
        rob->torque[i]=tm[i];
    //logging();
}


void CartesianController::PDControl()
{
    for (int i=0;i<3;i++)
    {

        dx[i]=xd[i]-x[i];
        f[i]=Kp[i]*dx[i]-Kv[i]*(v[i]-vd[i]);
        s[i]=s[i]+dx[i];
        f[i]+=Ks[i]*s[i];
    }
    for (int i=0;i<4;i++)
    {
        tq[i]=0;
        for (int j=0;j<3;j++)
        {
            tq[i]+=jacobian[i][j]*f[j];
            //cout<<jacobian[4*j+i]<<',';
        }
        // cout<<endl;
    }
    for (int i=0;i<4;i++)
    {
        tm[i]=0;
        for (int j=0;j<4;j++)
        {
            tm[i]+=rob->m2j[4*j+i]*tq[j];

        }

    }
    for (int i=0;i<4;i++)
        rob->torque[i]=tm[i];
    //logging();
}

void CartesianController::set_position_ref()
{
    for (int i=0;i<4;i++)
    {
        mzero[i]=rob->q[i];
        qzero[i]=q[i];
    }
    for (int i=0;i<3;i++)
        xzero[i]=x[i];
}
void CartesianController::set_gc()
{
    for (int i=0;i<4;i++)
    {
        tzero[3]=tm[i];
    }
}
void CartesianController::clear_ref()
{
    for (int i=0;i<4;i++)
    {
        mzero[i]=0;
        qzero[i]=0;
        tzero[i]=0;
    }
    for (int i=0;i<3;i++)
        xzero[i]=0;
}


void CartesianController::logging()
{
    std::string log;
    std::stringstream out;
    for (int i=0;i<3;i++)
        out<<x[i]<<",";
    for (int i=0;i<3;i++)
        out<<xd[i]<<",";
    for (int i=0;i<3;i++)
        out<<v[i]<<",";
    for (int i=0;i<3;i++)
        out<<f[i]<<",";
    for (int i=0;i<3;i++)
        out<<(x[i]-xd[i])<<",";
    for (int i=0;i<4;i++)
        out<<q[i]<<',';
    for (int i=0;i<4;i++)
        out<<tq[i]<<',';
    for (int i=0;i<4;i++)
        out<<rob->q[i]<<',';
    for (int i=0;i<4;i++)
        out<<rob->v[i]<<',';
    for (int i=0;i<4;i++)
        out<<rob->torque[i]<<',';
    out << ts.tv_sec << ','<< ts.tv_nsec;
    log = out.str();
    diag.data=log;
    diag.header.stamp.sec=ts.tv_sec;
    diag.header.stamp.nsec=ts.tv_nsec;
    diagnostic_pub.publish(diag);
    //vector<double> cur_pos_act(q,q+4);
    js.header.stamp.sec=ts.tv_sec;
    js.header.stamp.nsec=ts.tv_nsec;
    //js.position=cur_pos_act;

    for (int i=0;i<3;i++)
    {
        js.position[i]=x[i];
        js.velocity[i]=v[i];
        js.effort[i]=f[i];
    }
    joint_state_pub.publish(js);
}
void CartesianController::stop()
{
  close(fd);
    rob->close();
    
}
