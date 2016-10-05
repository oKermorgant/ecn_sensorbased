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
    vpColVector v,s,vc(6), sd(3);
    sd[1] = 0;
    sd[2] = 0.1;
    robot.setSd(sd);

    vc[3] = 0.1;

    vpMatrix J,L;
    double l = 0.5;
    vpMatrix La(1,6);
    vector<int> deact = {};

    while(ros::ok())
    {
        cout << "-------------" << endl;
        s = robot.getXY();
        La[0][2] = .5;//sqrt(s[2]);
        //s.resize(2,false);
        //s[1] = sd[1];
        p.set_xyZ(s[0], s[1], 3);
        L = vpMatrix::stack(p.interaction(),La);
        J = robot.getCamJacobian();
        cout << "s-sd: " << (s-sd).t() << endl;
        vc = -l*L.pseudoInverse()*(s-sd);
        cout << "vc: " << vc.t() << endl;

        for(auto j: deact)
        {
            for(int i=0;i<J.getRows();++i)
                J[i][j] = 0;
        }
         cout << "J: " << endl << J << endl;


        v = J.pseudoInverse()*vc;
        cout << "v: " << v.t() << endl;
        //   v = J.pseudoInverse()*vc;

        /*   cout << "J inv: " << endl << J.pseudoInverse() << endl;
        cout << "v: " << v.t() << endl;*/
        robot.setVelocity(v);
        ros::spinOnce();
        loop.sleep();
    }


}
