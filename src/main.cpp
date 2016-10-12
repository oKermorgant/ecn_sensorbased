#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_sensorbased/utils.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    PioneerCam robot(nh);
    ros::Rate loop(10);

    // Jacobians
    vpMatrix J1(2,4), J2, J(20,4);
    J1[0][0] =  J1[1][1] = 1;
    putAt(J, J1, 0, 0);
    //putAt(J, robot.getUSJacobian(), 4, 0);

    // weighting matrix
    vpMatrix H;H.eye(J.getRows());

    // image visibility
    vpColVector xy_lim = 0.9*robot.getCamLimits();
    vpColVector xy_act = 0.7*xy_lim;
    vpColVector s, sd(2);
    robot.setSd(sd);

    // us measure & Jacobian
    vpColVector us(16), us_d(16);us_d = 2;
    vpMatrix Jus;


    // error vectors
    vpColVector e1(2), e2, e(20);
    vpFeaturePoint p;

    // gains
    double lv = .5,
           lc = 0.5;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector v;
    while(ros::ok())
    {
        it++;
        cout << "-------------" << endl;

        // get robot and target positions to get position error
        target = robot.getTargetRelativePose();
        e1[0] = .5*lv*(target.x - .1);
        e1[1] = lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);
        putAt(e, e1, 0);

        // image and Jacobian
        robot.getImagePoint(s);
        p.set_xyZ(s[0], s[1], 3);
        e2 = -lc * (s-sd);
        J2 = p.interaction() *robot.getCamJacobian();
        putAt(J, J2, 2, 0);
        putAt(e, e2, 2);

        // us
        robot.getUSMeasureAndJacobian(us, Jus);
        putAt(J, Jus, 4, 0);
        //cout << "us: " << us.t() << endl;
        putAt(e, -0.01*(us - us_d), 4);

        // image weights
        H[2][2] = weightDouble(s[0], xy_act[0], xy_lim[0]);
        H[3][3] = weightDouble(s[1], xy_act[1], xy_lim[1]);
        // us weights
        for(int i=4;i<20;++i)
            H[i][i] = weight(-us[i-4], -0.5, -0.2);

        v = (H*J).pseudoInverse() * H * e;

        cout << "v: " << v.t() << endl;


        robot.setVelocity(v);
        ros::spinOnce();
        loop.sleep();
    }

}
