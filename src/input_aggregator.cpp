/**
 * @file input_aggregator.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Provides mixing utilities to combine direct user control inputs and controller-generated inputs
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <input_aggregator.hpp>

#include <uav_utils.hpp>

/**
 * @brief Construct a new Input Aggregator:: Input Aggregator object
 * 
 * @param n The ROS NodeHandle of the ROS node instantiating this class
 */
InputAggregator::InputAggregator(ros::NodeHandle n)
{
    rawSub_ = n.subscribe("rawPWM", 1, &InputAggregator::rawCtrlsCallback, this);
    surfaceSub_ = n.subscribe("ctrlSurfaceCmds", 1, &InputAggregator::surfaceCtrlsCallback, this);
    throttleSub_ = n.subscribe("throttleCmd", 1, &InputAggregator::throttleCtrlsCallback, this);

    pub_ = n.advertise<last_letter_msgs::SimPWM>("ctrlPWM", 10);

    ros::param::get("ctrlMode", ctrlMode_);
};

/**
 * @brief Callback which stores the user control inputs 
 * 
 * @param msg The user control input message
 */
void InputAggregator::rawCtrlsCallback(last_letter_msgs::SimPWM msg)
{
    rawCtrls_ = msg;
    if (ctrlMode_ == 0) // Direct passthrough, trigger with raw controls
    {
        publishCtrls();
    }
}

/**
 * @brief Callback which stores the controller-issued control surface commands
 * 
 * @param msg The controller-issued commands
 */
void InputAggregator::surfaceCtrlsCallback(geometry_msgs::Vector3Stamped msg)
{
    surfaceCtrls_.vector.x = msg.vector.x;
    surfaceCtrls_.vector.y = msg.vector.y;
    surfaceCtrls_.vector.z = msg.vector.z;

    if (ctrlMode_ == 1 || ctrlMode_ == 2) // Publish as fast as you have control surface inputs
    {
        publishCtrls();
    }
}

/**
 * @brief Callback which stores the controller issued throttle commands
 * 
 * @param msg The throttle commands
 */
void InputAggregator::throttleCtrlsCallback(geometry_msgs::Vector3Stamped msg)
{
    throttleCtrls_ = msg;
}

/**
 * @brief Performs the input mixing. Relies on the ctrlMode parameter to pick a strategy.
 * 
 */
void InputAggregator::mixer()
{
    switch (ctrlMode_)
    {
    case 0:  // Direct passthrough from rawPWM topic
        mixedCtrls_ = rawCtrls_;
        break;
    
    case 1: // Use the control surface signals from the controller
        mixedCtrls_ = rawCtrls_; // Copy over all of the raw signals
        mixedCtrls_.header.stamp = ros::Time::now(); // Stamp the message
        // Pick selected channels and convert them to PWM values
        mixedCtrls_.value[0] = FullRangeToPwm(surfaceCtrls_.vector.x); // Copy over aileron
        mixedCtrls_.value[1] = FullRangeToPwm(surfaceCtrls_.vector.y); // Copy over elevator
        mixedCtrls_.value[3] = FullRangeToPwm(surfaceCtrls_.vector.z); // Copy over rudder
        break;
    
    case 2: // Use the control surface signals from the controller and throttle from the trajectory controller
        mixedCtrls_ = rawCtrls_; // Copy over all of the raw signals
        mixedCtrls_.header.stamp = ros::Time::now(); // Stamp the message
        // Pick selected channels and convert them to PWM values
        mixedCtrls_.value[0] = FullRangeToPwm(surfaceCtrls_.vector.x); // Copy over aileron
        mixedCtrls_.value[1] = FullRangeToPwm(surfaceCtrls_.vector.y); // Copy over elevator
        mixedCtrls_.value[3] = FullRangeToPwm(surfaceCtrls_.vector.z); // Copy over rudder
        mixedCtrls_.value[2] = HalfRangeToPwm(throttleCtrls_.vector.x); // Copy over throttle
        break;
    
    default:
        ROS_ERROR("Please specify a value for ctrlMode parameter");
        break;
    }
}

/**
 * @brief Publish an input message, based on the currently stored inputs
 * 
 */
void InputAggregator::publishCtrls()
{
    mixer(); // Mix control signals according to ctrlMode_ setting
    pub_.publish(mixedCtrls_);
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "input_aggregator");
    ros::NodeHandle n;

    InputAggregator input_aggregator(n);
    ros::WallDuration(3).sleep(); //wait for other nodes to get raised
    ROS_INFO("Input Aggregator node up");

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}