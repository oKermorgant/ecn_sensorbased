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
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMeterPixelConversion.h>


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

    // send a velocity to the joints
    void setVelocity(const vpColVector &v);

    // gives the desired visual features
    void setSd(vpColVector _s)
    {
        vpMeterPixelConversion::convertPoint(cam_, _s[0], _s[1], pd_.x, pd_.y);
        rd_ = sqrt(_s[2]*cam_.get_px()*cam_.get_py()/M_PI);
        std::cout << "Desired point: (" << pd_.x << ", " << pd_.y << ", " << rd_ << ")" << std::endl;
    }

    // get Jacobian of camera wrt joint velocities    
    vpMatrix getCamJacobian(const vpColVector &_q);    
    inline vpMatrix getCamJacobian() {return getCamJacobian(q_);}

    // get the joint values
    inline vpColVector getJoints() {return q_;}

    // get the image point
    inline vpColVector getXY() {return s_im_;}

    // get Jacobian of US sensor measurement wrt joint velocities
    // is actually constant
    inline vpMatrix getUSJacobian(const int &_i) {return us_jacobian_[_i];}

protected:

    // robot model
    // wheels
    double radius_, base_, w_max_;
    // US sensors Jacobians
    std::vector<vpMatrix> us_jacobian_;
    // camera position offset
    vpCameraParameters cam_;

    // V-REP interface
    // joint subscriber
    ros::Subscriber joint_sub_;
    std::vector<std::string> joint_names_;
    vpColVector q_;
    // joint velocity publisher
    ros::Publisher joint_pub_;
    // joint velocity message
    vrep_common::JointSetStateData joint_setpoint_;
    // image message
    image_transport::ImageTransport it_;
    image_transport::Subscriber im_sub_;

    // handle for all US subscriptions
    std::vector<USSub> us_subs_;
    // ultrasound sensor values
    vpColVector s_us_;
    // image point
    vpColVector s_im_;
    cv::Point2d pd_;
    double rd_;
    // callbacks
    void readJointState(const sensor_msgs::JointStateConstPtr &_msg);
    void readImage(const sensor_msgs::ImageConstPtr& msg);

};

#endif // PIONEERCAM_H
