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
#include <cv_bridge/cv_bridge.h>
#include <visp/vpCameraParameters.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <ecn_common/color_detector.h>
#include <visp/vpFeaturePoint.h>


class PioneerCam
{
public:
    PioneerCam();

    // send a velocity to the joints
    void sendVelocity(const vpColVector &v);

    // if the robot has received sensor data
    bool stateReceived() {
        ros::spinOnce();
        loop_.sleep();

        if(!(joint_ok_ && im_ok_ && target_ok_))
            std::cout << "Waiting for incoming messages... did you start the simulation?\n";
        return joint_ok_ && im_ok_ && target_ok_;
    }

    // getters
    double radius() {return radius_;}
    double base() {return base_;}
    double wmax() {return w_max_;}

    // get target pose in robot frame
    geometry_msgs::Pose2D targetRelativePose() const {return target_pose_;}

    // get the joint values
    inline vpColVector q() const {return q_;}

    // get the current image point
    inline vpFeaturePoint imagePoint() const
    {
      vpFeaturePoint p;
      p.set_xyZ(s_im_[0], s_im_[1], 1.);
      return p;
    }

    // get the camera x-y visibility limits
    inline vpColVector camLimits()
    {
        vpColVector l(2);
        l[0] = color_detector.xLim();
        l[1] = color_detector.yLim();
        return l;
    }

    // get Jacobian of camera wrt joint velocities
    vpMatrix camJacobian(const vpColVector &_q) const ;
    inline vpMatrix camJacobian()  const {return camJacobian(q_);}

protected:
    // robot model
    // wheels
    double radius_, base_, w_max_;

    // V-REP interface
    ros::NodeHandle nh_;
    ros::Rate loop_;
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
    bool joint_ok_, im_ok_, target_ok_;

    // image point
    ecn::ColorDetector color_detector;
    vpColVector s_im_;
    cv::Point2d pd_;

    // callbacks
    void readJointState(const sensor_msgs::JointStateConstPtr &_msg);
    void readImage(const sensor_msgs::ImageConstPtr& msg);
    void readTargetPose(const geometry_msgs::PoseConstPtr &msg);
    void readSpherePose(const geometry_msgs::PoseConstPtr &msg);
};

#endif // PIONEERCAM_H
