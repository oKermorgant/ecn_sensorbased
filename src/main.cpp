#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_common/vpQuadProg.h>
#include <ecn_common/visp_utils.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PioneerCam robot;

    // pose error gain
    const double lv = .5;
    // constraints gain
    const double lc = 2;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector v(4);

    // QP solver
    vpQuadProg qp;


    while(ros::ok())
    {
        it++;
        cout << "-------------" << endl;

        if(robot.ok())
        {
            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();

            // linear velocity
            v[0] = lv*(target.x - .1);
            // angular velocity
            v[1] = 10*lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);

            cout << "v: " << v.t() << endl;

            robot.setVelocity(v);
        }
    }
}
