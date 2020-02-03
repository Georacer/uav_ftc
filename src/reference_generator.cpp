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

#include <math_utils.hpp>

#include <geometry_msgs/Vector3Stamped.h>

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
    // Read the control mode
    ROS_INFO("Reading the control mode setting");
    if (!ros::param::getCached("ctrlMode", ctrlMode_))
    {
        ROS_FATAL("Invalid parameter for -/ctrlMode- in param server!");
        ros::shutdown();
    }
    // Read the reference commands parameters from the server:w
    ROS_INFO("Reading reference scale configuration");
    if (!ros::param::getCached("referenceMin", listDouble))
    {
        ROS_FATAL("Invalid parameters for -/referenceMin- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listDouble.size(); ++i)
    {
        ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        referenceMin_(i) = listDouble[i];
    }
    if (!ros::param::getCached("referenceMax", listDouble))
    {
        ROS_FATAL("Invalid parameters for -/referenceMax- in param server!");
        ros::shutdown();
    }
    for (i = 0; i < listDouble.size(); ++i)
    {
        ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        referenceMax_(i) = listDouble[i];
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

    switch (ctrlMode_)
    {
    case 0:  // Direct passthrough from rawPWM topic
        ROS_ERROR("Raised reference_generator with ctrlMode parameter = 0");
        break;
    
    case 1: // Generate reference rates
        reference(0) = map(inputs[0], -1.0, 1.0, referenceMin_(0), referenceMax_(0) ); // roll channel
        reference(1) = map(inputs[1], -1.0, 1.0, referenceMin_(1), referenceMax_(1) ); // pitch channel
        reference(2) = map(inputs[3], -1.0, 1.0, referenceMin_(2), referenceMax_(2) ); // yaw channel
        break;
    
    case 2: // Generate reference trajectory
        // use map_centered because reference trajectories usually don't have symmetric limits
        reference(0) = map(inputs[2], -1.0, 1.0, referenceMin_(0), referenceMax_(0) ); // Airspeed channel
        reference(1) = map_centered(inputs[1], -1.0, 1.0, referenceMin_(1), referenceMax_(1) ); // Flight path angle channel
        reference(2) = map_centered(inputs[0], -1.0, 1.0, referenceMin_(2), referenceMax_(2) ); // Turn rate channel
        // reference(2) = referenceMax_(2)/inputs[0]; // Turn rate channel
        break;
    
    default:
        ROS_ERROR("Please specify a valid value for ctrlMode parameter");
        break;
    }
    // ROS_INFO("Created reference:");
    // std::cout << reference << std::endl;
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