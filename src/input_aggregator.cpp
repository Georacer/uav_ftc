// Merge input commands for control surfaces and throttle and output them to the UAV control topic
#include <input_aggregator.hpp>

InputAggregator::InputAggregator(ros::NodeHandle n)
{
    rawSub_ = n.subscribe("rawPWM", 1, &InputAggregator::rawCtrlsCallback, this);
    surfaceSub_ = n.subscribe("ctrlSurfaceCmds", 1, &InputAggregator::surfaceCtrlsCallback, this);
    throttleSub_ = n.subscribe("throttleCmds", 1, &InputAggregator::throttleCtrlsCallback, this);

    pub_ = n.advertise<last_letter_msgs::SimPWM>("ctrlPWM", 10);

    ros::param::get("ctrlMode", ctrlMode_);
};

void InputAggregator::rawCtrlsCallback(last_letter_msgs::SimPWM msg)
{
    rawCtrls_ = msg;
    if (ctrlMode_ == 0) // Direct passthrough, trigger with raw controls
    {
        publishCtrls();
    }
}

void InputAggregator::surfaceCtrlsCallback(last_letter_msgs::SimPWM msg)
{
    surfaceCtrls_ = msg;
    if (ctrlMode_ == 1) // Publish as fast as you have control surface inputs
    {
        publishCtrls();
    }
}

void InputAggregator::throttleCtrlsCallback(last_letter_msgs::SimPWM msg)
{
    throttleCtrls_ = msg;
}

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
        mixedCtrls_.value[0] = surfaceCtrls_.value[0]; // Copy over aileron
        mixedCtrls_.value[1] = surfaceCtrls_.value[1]; // Copy over elevator
        mixedCtrls_.value[3] = surfaceCtrls_.value[2]; // Copy over rudder
    
    default:
        ROS_ERROR("Please specify a value for ctrlMode parameter");
        break;
    }
}

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