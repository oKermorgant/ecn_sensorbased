#include <ecn_sensorbased/pioneer_cam.h>
#include <vrep_common/simRosGetObjectHandle.h>


PioneerCam::PioneerCam(ros::NodeHandle &_nh)
{    
    // wheels
    radius_ = .16;
    base_ = .3;
    w_max_ = 10;

    // camera position offset
    cam_x_ = .2;
    cam_z = .02;

    // US sensors
    geometry_msgs::Pose2D pose;
    s_us_.resize(16);
    s_us_ = 1;
    us_subs_.resize(16);
    for(unsigned int i=0;i<16;++i)
    {
        // sensor pose
        us_pose_.push_back(pose);
        // this subscriber
        us_subs_[i].init(s_us_[i], i, _nh);
    }

    // joints subscriber
    joint_sub_ = _nh.subscribe("/vrep/joint_states", 1, &PioneerCam::getJointState, this);
    joint_names_ = {"Pioneer_p3dx_leftMotor", "Pioneer_p3dx_rightMotor"};
    q_.resize(joint_names_.size());

    // joint publisher
    joint_pub_  =_nh.advertise<vrep_common::JointSetStateData>("/vrep/joint_setpoint", 1);
    // get handles
    ros::ServiceClient client = _nh.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    client.waitForExistence();
    vrep_common::simRosGetObjectHandle srv;
    for(auto joint: {"Pioneer_p3dx_leftMotor", "Pioneer_p3dx_rightMotor", "camera_pan", "camera_tilt"})
    {
        srv.request.objectName = joint;
        if(client.call(srv))
        {
            std::cout << "Handle for " << joint << ": " << srv.response.handle << std::endl;
            joint_setpoint_.handles.data.push_back(srv.response.handle);
            joint_setpoint_.setModes.data.push_back(2);
            joint_setpoint_.values.data.push_back(0);
        }
    }
    std::cout << "Pioneer init ok" << std::endl;

}

void PioneerCam::setVelocity(vpColVector v)
{
    for(unsigned int i=0;i<v.size();++i)
        joint_setpoint_.values.data[i] = v[i];
    joint_pub_.publish(joint_setpoint_);
}




void PioneerCam::getJointState(const sensor_msgs::JointStateConstPtr &_msg)
{
    for(unsigned int j=0;j<joint_names_.size();++j)
    {
        for(unsigned int i=0;i<_msg->name.size();++i)
        {
            if(_msg->name[i] == joint_names_[j])
            {
                q_[j] = _msg->position[i];
                break;
            }
        }
    }

}

