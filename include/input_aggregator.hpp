#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <uav_utils/uav_utils.hpp>
#include <last_letter_msgs/SimPWM.h>
#include <last_letter_msgs/SimStates.h>

class InputAggregator
{
private:
///////////
//Variables
ros::Subscriber rawSub_, surfaceSub_, throttleSub_;
ros::Publisher pub_;
int ctrlMode_ = 0;
last_letter_msgs::SimPWM rawCtrls_;
geometry_msgs::Vector3Stamped surfaceCtrls_, throttleCtrls_;
last_letter_msgs::SimPWM mixedCtrls_;

public:
////////// 
// Methods

InputAggregator(ros::NodeHandle n);
void rawCtrlsCallback(last_letter_msgs::SimPWM msg);
void surfaceCtrlsCallback(geometry_msgs::Vector3Stamped msg);
void throttleCtrlsCallback(geometry_msgs::Vector3Stamped msg);
void statesCallback(last_letter_msgs::SimStates msg);
void mixer();
void publishCtrls();
};