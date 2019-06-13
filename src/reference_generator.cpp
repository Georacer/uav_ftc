/**
 * @file reference_generator.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Read the joystick commands and convert them to a reference command
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <reference_generator.hpp>

/**
 * @brief Construct a new Reference Generator:: Reference Generator object
 * Is tasked with generating reference commands from user inputs
 * 
 * @param n The ROS NodeHandle of the ROS node instantiating this class
 */
ReferenceGenerator::ReferenceGenerator(ros::NodeHandle n)
{
    n_ = n;
    sub_ = n.subscribe("joy", 1, &ReferenceGenerator::joyCallback, this);
    pub_ = n.advertise<geometry_msgs::Vector3Stamped>("refCmds", 1);

    // Read the controller configuration parameters from the HID.yaml file
    XmlRpc::XmlRpcValue listInt, listDouble;
    int i;
    ROS_INFO("Reading throws directions");
    if (!ros::param::getCached("/HID/throws", listDouble))
    {
        ROS_FATAL("Invalid parameters for -/HID/throws- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listDouble.size(); ++i)
    {
        ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        throwIndex_[i] = listDouble[i];
    }
    ROS_INFO("Reading input axes");
    if (!ros::param::getCached("/HID/axes", listInt))
    {
        ROS_FATAL("Invalid parameters for -/HID/axes- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listInt.size(); ++i)
    {
        ROS_ASSERT(listInt[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        axisIndex_[i] = listInt[i];
    }
    ROS_INFO("Reading input buttons configuration");
    if (!ros::param::getCached("/HID/buttons", listInt))
    {
        ROS_FATAL("Invalid parameters for -/HID/buttons- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listInt.size(); ++i)
    {
        ROS_ASSERT(listInt[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        buttonIndex_[i] = listInt[i];
    }
    // Read the reference commands parameters from the server:w
    ROS_INFO("Reading reference scale configuration");
    if (!ros::param::getCached("refScale", listDouble))
    {
        ROS_FATAL("Invalid parameters for -/refScale- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listDouble.size(); ++i)
    {
        ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        scaling_(i) = listDouble[i];
    }
}

/**
 * @brief Callback to the joystick message, representing user input
 * 
 * @param joyMsg The joystick message
 */
void ReferenceGenerator::joyCallback(sensor_msgs::Joy joyMsg)
{
    for (int i = 0; i < 11; i++)
    {
        if (axisIndex_[i] != -1)
        { // if an axis is assigned in this channel
            inpChannels_[i] = 1.0 / throwIndex_[i] * joyMsg.axes[axisIndex_[i]];
        }
        else if (buttonIndex_[i] != -1)
        {
            inpChannels_[i] = 1.0 / throwIndex_[i] * joyMsg.buttons[buttonIndex_[i]];
        }
        else
        {
            inpChannels_[i] = 0.0;
        }
    }

    reference_ = convertInputs(inpChannels_);
    publishCmds(reference_);
}

/**
 * @brief Convert the user inputs to reference commands.
 * 
 * @param inputs A double array, containing the user input channels, in the -1,1 range
 * @return Eigen::Vector3d The angular rates (p, q, r) reference values
 */
Eigen::Vector3d ReferenceGenerator::convertInputs(double *inputs)
{
    Eigen::Vector3d reference;
    reference(0) = inputs[0] * scaling_(0); // roll channel
    reference(1) = inputs[1] * scaling_(1); // pitch channel
    reference(2) = inputs[3] * scaling_(2); // Yaw channel
    return reference;
}

/**
 * @brief Publish the reference values
 * 
 * @param reference The reference values
 */
void ReferenceGenerator::publishCmds(Eigen::Vector3d reference)
{
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = reference(0);
    msg.vector.y = reference(1);
    msg.vector.z = reference(2);
    pub_.publish(msg);
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "reference_generator");
    ros::NodeHandle n;

    ros::WallDuration(3).sleep(); //wait for other nodes to get raised

    ReferenceGenerator refGen(n);
    ROS_INFO("Reference Generator up");

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}