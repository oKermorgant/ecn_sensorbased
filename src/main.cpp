#include <ecn_sensorbased/pioneer_cam.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    PioneerCam robot(nh);

    ros::Rate loop(10);

    vpColVector v(4);
    v[0] = 0.5;
    v[1] = -0.5;
    v[2] = 0;
    v[3] = 0;

    int i = 0;
    while(ros::ok())

    {v[3] = .1;
        v[2] = .5;


        robot.setVelocity(v);
        ros::spinOnce();
        loop.sleep();
    }
}
