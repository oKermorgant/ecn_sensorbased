#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    PioneerCam robot(nh);

    vpFeaturePoint p;

    ros::Rate loop(10);
    vpColVector v(4),s,vc(6), sd(2),q;
    sd[1] = -.1;
    robot.setSd(sd);

    //vc[3] = 0.1;

    vpMatrix J1(2,4), J2, P1, I4(4,4);I4.eye();
    J1[0][0] = J1[1][1] = 1;
    P1 = I4 - J1.pseudoInverse()*J1;
    vpColVector e1(2), e2, v1;

    double lv = .1, lc = .5;
    vpMatrix La(1,6);
    vector<int> deact = {};
    geometry_msgs::Pose2D target;

    double pan = 0,
            tilt = 0;
    int it = 0;
    bool pos = true;
    while(ros::ok())
    {
        it++;
        cout << "-------------" << endl;


        q = robot.getJoints();



        if(pos && vpMath::sqr(q[0]-pan) + vpMath::sqr(q[1]-tilt) > .1)
        {

            cout << "setPosition: q[0] = " << q[0]  << endl;
            cout << "setPosition: q[1] = " << q[1]  << endl;
            v[2] = pan-q[0];
            v[3] = tilt-q[1];
        }
        else
        {
            pos = false;





            // get robot and target positions
            target = robot.getTargetRelativePose();
            e1[0] = .5*lv*(target.x - .1);
            e1[1] = lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);

            // image
            s = robot.getImagePoint();
            p.set_xyZ(s[0], s[1], 3);

            e2 = -lc * (s-sd);
            //e2[1] *= 10;
            J2 = p.interaction() *robot.getCamJacobian();

            for(auto &j: deact)
            {
                for(int i=0;i<J2.getRows();++i)
                    J2[i][j] = 0;
            }

            v1 = J1.pseudoInverse() * e1;
            //   cout << "v1: " << v1.t() << endl;
            //   cout << "P1" << endl << P1 << endl;
            //   cout << "J2+e2: " << (J2.pseudoInverse()*e2).t() << endl;
            //   cout << "J2P1" << endl << J2*P1 << endl;

              v = v1 + P1*(J2*P1).pseudoInverse()*(e2 - J2*v1);

            //vc =  p.interaction().pseudoInverse() * e2;
            //vc[4] = .1;

            cout << "vc: " << vc.t() << endl;
           // v = J2.pseudoInverse() * vc;
        }

        cout << "v: " << v.t() << endl;


        robot.setVelocity(v);
        ros::spinOnce();
        loop.sleep();
    }

}
