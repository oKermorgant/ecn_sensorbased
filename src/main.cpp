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

    // Jacobians
    vpMatrix J1(2,4), J2(2,4), J(20,4);
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
    vpMatrix J3(16,4);
    vpColVector us_lim(16);us_lim = 0.1;

    // error vectors
    vpColVector e1(2), e2(2), e(20);
    vpFeaturePoint p;

    // QP matrices
    vpMatrix C(24,4);
    vpColVector d(24);

    // wheel velocity
    double radius = .0975, base = .331, w_max = 4;
    vpMatrix Jw(4,4);
    vpColVector ew(4);
    Jw[0][0] = Jw[1][0] = 1/radius;
    Jw[0][1] = Jw[3][1] = -base/radius;
    Jw[1][1] = Jw[2][1] = base/radius;
    Jw[2][0] = Jw[3][0] = -1/radius;
    ew[0] = ew[1] = w_max;
    ew[2] = ew[3] = -w_max;


    vpMatrix J2lo(2,4);
    vpColVector im_lo(2), im_up(2), us_lo(16);

    // gains
    double lv = .5,
            lc = 5;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector v;

    okSolveQP qp;
    qp.addEquality(J1, e1, 1, "position");
    //qp.addInequality(J2, im_up, 0, "point upper");
    //qp.addInequality(J2lo, im_lo, 0, "point lower");
    qp.addInequality(J3, us_lo, 0, "us");
    //qp.addInequality(Jw, ew, 0, "wheels");
    qp.print();

    while(ros::ok())
    {
        if(robot.ok())
        {
            it++;
            cout << "-------------" << endl;

            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();
            e1[0] = lv*(target.x - .1);
            e1[1] = 10*lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);
            putAt(e, e1, 0);

            // image and Jacobian
            robot.getImagePoint(s);
            p.set_xyZ(s[0], s[1], 3);
            e2 = -lc * (s-sd);
            J2 = p.interaction() *robot.getCamJacobian();
            putAt(J, J2, 2, 0);
            putAt(e, e2, 2);

            // us
            robot.getUSMeasureAndJacobian(us, J3);
            putAt(J, J3, 4, 0);
            //cout << "us: " << us.t() << endl;
            putAt(e, -0.01*(us - us_d), 4);


            // image weights
            H[2][2] = weightDouble(s[0], xy_act[0], xy_lim[0]);
            H[3][3] = weightDouble(s[1], xy_act[1], xy_lim[1]);
            // us weights
            for(int i=4;i<20;++i)
                H[i][i] = weight(-us[i-4], -0.5, -0.2);

            //v = (H*J).pseudoInverse() * H * e;


            // QP approach
            // im up
            /*putAt(C, J2, 0,0);
            putAt(d, lc*(xy_lim - s), 0);
            // im low
            putAt(C, -J2, 2,0);
            putAt(d, -lc*(xy_lim + s), 2);
            // US
            putAt(C, -J3, 4,0);
            putAt(d, 1*(us_lim - us), 4);*/

            //okSolveQP::solveQPi(J1, e1, C, d, v);

            J2lo = -J2;
            im_up = lc*(xy_lim - s);
            im_lo = -lc*(xy_lim + s);
            J3 = -J3;
            us_lo = 0.01*(us_lim - us);
            qp.solveCascade(v);


            cout << "v: " << v.t() << endl;

            robot.setVelocity(v);
        }
        ros::spinOnce();
        loop.sleep();
    }

}
