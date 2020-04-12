/**
 * @file rosflight_publisher.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Translate the ctrlPWM topic into ROSFlight Command msgs
 * @date 2020-04-12
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <math_utils.hpp>

#include <geometry_msgs/Vector3Stamped.h>
#include <rosflight_msgs/Command.h>

#include <last_letter_msgs/SimPWM.h>

ros::Publisher pub;

/**
 * @brief Callback to the control surfaces message
 * 
 * @param data_bus The control surfaces message
 */
void rcCallback(last_letter_msgs::SimPWM)
{
    rosflight_msgs::Command new_msg;
    new_msg.mode = msg.MODE_PASS_THROUGH;
    new_msg.ignore = msg.IGNORE_NONE;
    new_msg.x = data_bus.rc_in[0];
    new_msg.y = data_bus.rc_in[1];
    new_msg.F = data_bus.rc_in[2];
    new_msg.z = data_bus.rc_in[3];

    pub.publish(new_msg);
}


///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "rosflight_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ctrlPWM", 1, rcCallback, this);
    pub = n.advertise<rosfliht_msgs::>("/command", 1);

    ROS_INFO("rosflight_publisher node up");

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}