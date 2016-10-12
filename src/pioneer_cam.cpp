#include <ecn_sensorbased/pioneer_cam.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosSetObjectPose.h>
#include <vrep_common/simRosStartSimulation.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <algorithm>
#include <visp/vpSubMatrix.h>
using namespace std;


PioneerCam::PioneerCam(ros::NodeHandle &_nh) : it_(_nh)
{
    // restart simulation
    ros::ServiceClient client = _nh.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    client.waitForExistence();
    vrep_common::simRosStartSimulation srv_start;
    client.call(srv_start);

    // joint setpoint publisher
    joint_pub_  =_nh.advertise<vrep_common::JointSetStateData>("/vrep/joint_setpoint", 1);
    // get V-REP joint handles
    client = _nh.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
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

    // wheels
    radius_ = .0975;
    base_ = .331;
    w_max_ = 4;

    // camera calibration
    cam_.initFromFov(640,480,vpMath::rad(60), vpMath::rad(60.*480/640));

    // US sensors subscribers, pose, Jacobian, etc.
    s_us_.resize(16);
    s_us_ = 1;
    us_subs_.resize(16);
    us_jac_.resize(16);
    us_poses_.resize(16);
    double x, y, c, s;
    for(unsigned int i=0;i<16;++i)
    {
        std::stringstream ss;
        ss << "/sensors/us" << i+1;

        // sensor pose to Jacobian
        while(!ros::param::has(ss.str()))
            sleep(1);
        ros::param::get(ss.str() + "/x", us_poses_[i].x);
        ros::param::get(ss.str() + "/y", us_poses_[i].y);
        ros::param::get(ss.str() + "/theta", us_poses_[i].theta);
        us_subs_[i].init(s_us_[i], i+1, _nh);

        // Jacobian of sensor position wrt robot (v,w,pan,tilt)
        us_jac_[i].resize(3,4);
        x = us_poses_[i].x;
        y = us_poses_[i].y;
        c = cos(us_poses_[i].theta);
        s = sin(us_poses_[i].theta);
        us_jac_[i][0][0] = c;
        us_jac_[i][0][1] = x*s - y*c;
        us_jac_[i][1][0] = -s;
        us_jac_[i][1][1] = x*c+y*s;
        us_jac_[i][2][0] = 0;
        us_jac_[i][2][1] = 1;
    }

    // joints subscriber
    joint_ok_ = false;
    joint_sub_ = _nh.subscribe("/vrep/joint_states", 1, &PioneerCam::readJointState, this);
    joint_names_ = {"camera_pan", "camera_tilt"};
    q_.resize(joint_names_.size());

    // image transport
    im_ok_ = false;
    s_im_.resize(2);
    cv::startWindowThread();
    im_sub_ = it_.subscribe("/vrep/image", 1, &PioneerCam::readImage, this);
    std::cout << "Pioneer init ok" << std::endl;
    sphere_sub_ = _nh.subscribe("/vrep/sphere", 1, &PioneerCam::readSpherePose, this);

    // pose
    target_ok_ = false;
    target_sub_ = _nh.subscribe("/vrep/target", 1, &PioneerCam::readTargetPose, this);

}


void PioneerCam::getUSMeasureAndJacobian(vpColVector &_s, vpMatrix &_J)
{
    _J.resize(16,4);
    // update measurements from timestamp
    const double t = ros::Time::now().toSec();
    for(unsigned int i=0;i<16;++i)
        if(t - us_subs_[i].t_ > 0.5)
            s_us_[i] = 1;
    _s = s_us_;

    // for all sensors, get previous and next point in this frame
    // then compute the lines describing the shape around here
    double xp,yp,xn,yn,d,c,s,a,b;
    vpMatrix Y(2,2);vpColVector X(2),AB;
    vpMatrix dDdX(1,3);
    vpSubMatrix Ji;
    geometry_msgs::Pose2D p, p2;
    for(unsigned int i=0;i<16;++i)
    {
        dDdX = 0;
        if(s_us_[i] != 1)
        {
            p = us_poses_[i];
            c = cos(p.theta);
            s = sin(p.theta);

            p2 = us_poses_[(i+1)%16];
            d = s_us_[(i+1)%16];
            xn = d*cos(p.theta - p2.theta) - p.x*c + p2.x*c - p.y*s + p2.y*s;
            yn = -d*sin(p.theta - p2.theta) + p.x*s - p2.x*s - p.y*c + p2.y*c;

            p2 = us_poses_[(i-1)%16];
            d = s_us_[(i-1)%16];
            xp = d*cos(p.theta - p2.theta) - p.x*c + p2.x*c - p.y*s + p2.y*s;
            yp = -d*sin(p.theta - p2.theta) + p.x*s - p2.x*s - p.y*c + p2.y*c;

            //cout << "us #" << i+1 << ": d = " << s_us_[i] << ", pp = (" << xp  << ", " << yp << "), pn = (" << xn << ", " << yn << ")";

            // local derivative
            dDdX[0][0] = -1;
            // get corresponding line
            d = s_us_[i];
            a = 0.5*((xn-d)/yn + (xp-d)/yp);
           // cout << " -> a = " << a << endl;
            //dDdX[0][1] = a;
            //dDdX[0][2] = -a*d;

            //us_jacobian_[i][1] = p.x*cos(p.theta)-p.y*sin(p.theta) + s_us_[i]*a;             // ds/dw
            //cout << " -> slope = " << a << endl;
            //dx/dtheta = (-b*tan(t) + sqrt(-4*a*d*tan(t)**2 + b**2*tan(t)**2 - 2*b*tan(t) (t)**2)

            // get parabol
            Y[0][0] = yn*yn;
            Y[0][1] = yn;
            Y[1][0] = yp*yp;
            Y[1][1] = yp;
            X[0] = xn-d;
            X[1] = xp-d;
            AB = Y.inverseByQR() * X;
            a = AB[0];
            b = AB[1];
            dDdX[0][1] = -b;

            // test
            //cout << "pn: " << a*yn*yn + b*yn + d - xn << endl;
            //cout << "pp: " << a*yp*yp + b*yp + d - xp << endl;



            //us_jacobian_[i][1] = p.x*cos(p.theta)-p.y*sin(p.theta);             // ds/dw
        }

        Ji.init(_J, i, 0, 1, 4);
        Ji = dDdX * us_jac_[i];


    }
}



void PioneerCam::setVelocity(const vpColVector &v)
{
    if(v.size() != 4)
    {
        std::cout << "PioneerCam::setVelocity: v should be length 4, is " << v.size() << std::endl;
        return;
    }

    // change (v,w) to left and right wheel velocities
    joint_setpoint_.values.data[0] = (v[0]-base_*v[1])/radius_;
    joint_setpoint_.values.data[1] = (v[0]+base_*v[1])/radius_;
    double a = std::max(vpMath::abs(joint_setpoint_.values.data[0])/w_max_, vpMath::abs(joint_setpoint_.values.data[1])/w_max_);
    // scale wheel velocities
    if(a<1)
        a = 1;

    joint_setpoint_.values.data[0] *= 1./a;
    joint_setpoint_.values.data[1] *= 1./a;

    // copy camera joints
    joint_setpoint_.values.data[2] = v[2]/a;
    joint_setpoint_.values.data[3] = v[3]/a;

    joint_pub_.publish(joint_setpoint_);
}


vpMatrix PioneerCam::getCamJacobian(const vpColVector &_q)
{
    vpMatrix J(6,4);
    const double c1 = cos(_q[0]);
    const double c2 = cos(_q[1]);
    const double s1 = sin(_q[0]);
    const double s2 = sin(_q[1]);

    J[0][0] = s1;
    J[0][1] = -0.155485*c1 - 0.035*c2;
    J[0][2] = -0.035*c2;
    //J[0][3] = 0;
    J[1][0] = s2*c1;
    //J[1][1] = 0;
    //J[1][2] = 0;
    J[1][3] = -0.035;
    J[2][0] = c1*c2;
    //J[2][1] = 0;
    //J[2][2] = 0;
    //J[2][3] = 0;
    //J[3][0] = 0;
    //J[3][1] = 0;
    //J[3][2] = 0;
    J[3][3] = 1.;
    //J[4][0] = 0;
    J[4][1] = -c2;
    J[4][2] = -c2;
    //J[4][3] = 0;
    //J[5][0] = 0;
    J[5][1] = s2;
    J[5][2] = s2;
    //J[5][3] = 0;

    return J;
}


void PioneerCam::readJointState(const sensor_msgs::JointStateConstPtr &_msg)
{
    joint_ok_ = true;
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

void PioneerCam::readImage(const sensor_msgs::ImageConstPtr& msg)
{

    cv::Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;
    try
    {
        cv::Mat img;
        cv::cvtColor(im, img, cv::COLOR_BGR2HSV);
        cv::inRange(img, cv::Scalar(55,0,0), cv::Scalar(65,255,255), img);
        cv::Canny(img, img, 20, 150);
        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        std::sort(contours.begin(), contours.end(),
                  [](const vector<cv::Point> &c1, const vector<cv::Point> &c2)
        {return cv::contourArea(c1) > cv::contourArea(c2);});

        if(contours.size() > 0)
        {
            cv::Moments m = cv::moments(contours[0], false);

            // s_im_[0] = m.m10/m.m00;
            // s_im_[1] = m.m01/m.m00;
            //s_im_[2] = m.m00;
            cv::drawContours(im, contours, 0, cv::Scalar(0,0,255), 2);

            cv::circle(im, pd_, sqrt(m.m00/M_PI), cv::Scalar(255,0,0), 2);
            // to normalized values
            double x,y;
            // vpPixelMeterConversion::convertPoint(cam_, m.m10/m.m00, m.m01/m.m00, s_im_[0], s_im_[1]);
        }
    }
    catch (...)
    {
        ROS_WARN("Could not detect sphere");
    }
    cv::imshow("view", im);
    cv::waitKey(1);
}


void PioneerCam::readTargetPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    target_ok_ = true;
    target_pose_.x = msg->pose.position.x;
    target_pose_.y = msg->pose.position.y;
    target_pose_.theta = 2*atan2(msg->pose.orientation.z, msg->pose.orientation.w);
}

void PioneerCam::readSpherePose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    im_ok_ = true;
    s_im_[0] = -msg->pose.position.x/msg->pose.position.z;
    s_im_[1] = -msg->pose.position.y/msg->pose.position.z;
}
