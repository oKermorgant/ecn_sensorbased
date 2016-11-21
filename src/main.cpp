#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_sensorbased/utils.h>
#include <ecn_sensorbased/qp.h>

using namespace std;


int main(int argc, char** argv)
{

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    PioneerCam robot(nh);

    ros::Rate loop(10);

    // gains
    // pose error gain
    double lv = .5;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector v(4);

    while(ros::ok())
    {
        if(robot.ok())
        {
            it++;
            cout << "-------------" << endl;

            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();

            // linear velocity
            v[0] = lv*(target.x - .1);
            // angular velocity
            v[1] = 10*lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);

            cout << "v: " << v.t() << endl;

            robot.setVelocity(v);

        }
        //robot.loop();
        ros::spinOnce();
        loop.sleep();

    }

}
