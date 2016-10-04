#ifndef PIONEERCAM_H
#define PIONEERCAM_H
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vrep_common/JointSetStateData.h>
#include <functional>
#include <geometry_msgs/Pose2D.h>
#include <vrep_common/ProximitySensorData.h>
#include <sstream>


struct USSub
{
    void init(double &_d, int _i, ros::NodeHandle &_nh)
    {
        d_ = &_d;
        std::stringstream ss;
        ss << "/vrep/us" << _i+1;
        i = _i;
      sub_ = _nh.subscribe(ss.str(), 1, &USSub::getUSDistance, this);
    }

    ros::Subscriber sub_;
    double* d_;
    int i;
    void getUSDistance(const vrep_common::ProximitySensorDataConstPtr &msg)
    {
        *d_ = msg->detectedPoint.z;
    }
};

class PioneerCam
{
public:
    PioneerCam(ros::NodeHandle &_nh);

    void setVelocity(vpColVector v);






protected:





    // robot model
    // wheels
    double radius_, base_, w_max_;
    // US sensors positions
    std::vector<geometry_msgs::Pose2D> us_pose_;
    // camera position offset
    double cam_x_, cam_z;

    // V-REP interface
    // joint subscriber
    ros::Subscriber joint_sub_;
    std::vector<std::string> joint_names_;
    vpColVector q_;
    // joint velocity publisher
    ros::Publisher joint_pub_;
    // joint velocity message
    vrep_common::JointSetStateData joint_setpoint_;

    // handle for all US subscriptions
    std::vector<USSub> us_subs_;
    // ultrasound sensor values
    vpColVector s_us_;
    // callbacks
    void getJointState(const sensor_msgs::JointStateConstPtr &_msg);

};

#endif // PIONEERCAM_H
