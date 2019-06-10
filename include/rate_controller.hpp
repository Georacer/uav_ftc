#include <ros/ros.h>
#include "rate_controller_wrapper.hpp"

#include <cstdlib>
#include <math.h>
#include <geometry_msgs/Vector3.h>

#include <mathutils/mathutils.hpp>
#include <uav_utils/uav_utils.hpp>
#include <last_letter_msgs/SimStates.h>

class RateController
{
public:
    ///////////
    //Variables
    last_letter_msgs::SimStates states;
    geometry_msgs::Vector3 airdata;
    uav_ftc::refRates refRates;
    double input[10]; // Stores control inputs, -1,1 range
    double output[10];  // Stores control outputs, -1,1 range
    ros::Time tprev;
    ros::Subscriber subState, subRef;
    ros::Publisher pubCtrl;

    /////////
    //Members
    RateMpcWrapper mpcController();

    ///////////
    //Functions
    void step();                                                 // Caller of rate_controller_wrapper
    void getStates(last_letter_msgs::SimStates measuredStates); // Callback to store measured states
    void getReference(uav_ftc::ref_rates reference);             // Callback to store reference command  //TODO: Declare and compile the msg
    void WriteOutput();                            // Send control signals to the control inputs aggregator

    //Constructor
    RateController(ros::NodeHandle n);
    //Destructor
    ~RateController();

private:
    ///////////
    //Variables

    ///////////
    //Functions
};
