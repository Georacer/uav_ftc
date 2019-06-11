#include <ros/ros.h>

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
last_letter_msgs::SimPWM rawCtrls_, surfaceCtrls_, throttleCtrls_;
last_letter_msgs::SimPWM mixedCtrls_;

public:
////////// 
// Methods

InputAggregator(ros::NodeHandle n);
void rawCtrlsCallback(last_letter_msgs::SimPWM msg);
void surfaceCtrlsCallback(last_letter_msgs::SimPWM msg);
void throttleCtrlsCallback(last_letter_msgs::SimPWM msg);
void statesCallback(last_letter_msgs::SimStates msg);
void mixer();
void publishCtrls();
};