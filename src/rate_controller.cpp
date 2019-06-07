#include "rate_controller.hpp"


///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controlNode");
    ros::NodeHandle n;

    ros::WallDuration(3).sleep(); //wait for other nodes to get raised
    double ctrlRate;
    ros::param::get("ctrlRate",ctrlRate); //frame rate in Hz
    ros::Rate spinner(ctrlRate);

    RateMpcWrapper<float> rate_ctrl();
    spinner.sleep();
    ROS_INFO("controlNode up");

    while (ros::ok())
    {
        ros::spinOnce();
        // rate_ctrl.step();
        spinner.sleep();

    }

    return 0;

}
