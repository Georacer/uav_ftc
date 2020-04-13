/**
 * @file rosflight_publisher.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Translate the ctrlPWM topic into ROSFlight Command msgs
 * @date 2020-04-12
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <rosflight_msgs/Command.h>

#include <math_utils.hpp>
#include <uav_utils.hpp>
#include <last_letter_msgs/SimPWM.h>

ros::Publisher pub;

/**
 * @brief Callback to the control surfaces message
 * 
 * @param pwm_msg The control surfaces message
 */
void rcCallback(last_letter_msgs::SimPWM pwm_msg)
{
    rosflight_msgs::Command new_msg;
    new_msg.mode = new_msg.MODE_PASS_THROUGH;
    new_msg.ignore = new_msg.IGNORE_NONE;
    new_msg.x = PwmToFullRange(pwm_msg.value[0]);
    new_msg.y = PwmToFullRange(pwm_msg.value[1]);
    new_msg.F = PwmToHalfRange(pwm_msg.value[2]);
    new_msg.z = -PwmToFullRange(pwm_msg.value[3]); // Invert sign because of topology of DeFault UAV

    pub.publish(new_msg);
}


///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "rosflight_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ctrlPWM", 1, rcCallback);
    pub = n.advertise<rosflight_msgs::Command>("command", 1);

    ROS_INFO("rosflight_publisher node up");

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}