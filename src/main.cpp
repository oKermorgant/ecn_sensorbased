#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpQuadProg.h>
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
    // robot command u = (v, omega, dot q_p, dot q_t)
    vpColVector u(4);

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
            u[0] = lv*(target.x - .1);
            // angular velocity
            u[1] = 5*lv*std::atan2(target.y, target.x);

            cout << "u: " << u.t() << endl;

            robot.setVelocity(u);
        }
    }
}
