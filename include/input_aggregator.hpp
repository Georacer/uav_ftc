#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <rosgraph_msgs/Clock.h>

#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/SimPWM.h>
#include <uav_ftc/BusData.h>

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
////////////
// Variables
bool ctrl_stale{false}; // TODO: for future feature

////////// 
// Methods

InputAggregator(ros::NodeHandle n);
void rawCtrlsCallback(uav_ftc::BusData msg);
void surfaceCtrlsCallback(geometry_msgs::Vector3Stamped msg);
void throttleCtrlsCallback(geometry_msgs::Vector3Stamped msg);
void statesCallback(last_letter_msgs::SimStates msg);
void clockCallback(rosgraph_msgs::Clock msg);
void mixer();
void publishCtrls();
};