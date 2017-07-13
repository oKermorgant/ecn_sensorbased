#ifndef PIONEERCAM_H
#define PIONEERCAM_H
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <functional>
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMeterPixelConversion.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>


class PioneerCam
{
public:
    PioneerCam(ros::NodeHandle &_nh);

    // send a velocity to the joints
    void setVelocity(const vpColVector &v);

    // if the robot has received sensor data
    bool ok() {
        if(!(joint_ok_ && im_ok_ && target_ok_))
            std::cout << "Waiting for incoming messages... did you start the simulation?\n";
        return joint_ok_ && im_ok_ && target_ok_;

    }

    // getters
    double radius() {return radius_;}
    double base() {return base_;}
    double wmax() {return w_max_;}

    // get target pose in robot frame
    geometry_msgs::Pose2D getTargetRelativePose() {return target_pose_;}

    // get the joint values
    inline vpColVector getJoints() {return q_;}

    // get the current image point
    inline void getImagePoint(vpColVector &_s) {_s = s_im_;}

    // gives the desired visual features
    void setSd(vpColVector _s)
    {
        vpMeterPixelConversion::convertPoint(cam_, _s[0], _s[1], pd_.x, pd_.y);
    }

    // get the camera x-y visibility limits
    inline vpColVector getCamLimits()
    {
        vpColVector l(2);
        l[0] = cam_.get_u0()*cam_.get_px_inverse();
        l[1] = cam_.get_v0()*cam_.get_py_inverse();
        return l;
    }

    // get Jacobian of camera wrt joint velocities
    vpMatrix getCamJacobian(const vpColVector &_q);
    inline vpMatrix getCamJacobian() {return getCamJacobian(q_);}

protected:
    // robot model
    // wheels
    double radius_, base_, w_max_;
    // US sensors Jacobians
    std::vector<vpMatrix> us_jac_;
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
    sensor_msgs::JointState joint_setpoint_;
    // image message
    image_transport::ImageTransport it_;
    image_transport::Subscriber im_sub_;
    // target positions
    ros::Subscriber target_sub_, sphere_sub_;
    geometry_msgs::Pose2D target_pose_;
    // init subs
    bool joint_ok_, us_ok_, im_ok_, target_ok_;

    // image point
    vpColVector s_im_;
    cv::Point2d pd_;

    // callbacks
    void readJointState(const sensor_msgs::JointStateConstPtr &_msg);
    void readImage(const sensor_msgs::ImageConstPtr& msg);
    void readTargetPose(const geometry_msgs::PoseConstPtr &msg);
    void readSpherePose(const geometry_msgs::PoseConstPtr &msg);


};

#endif // PIONEERCAM_H
