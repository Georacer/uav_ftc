#include <ros/ros.h>
#include "mpc_wrapper.hpp"

#include <cstdlib>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

#include <mathutils/mathutils.hpp>
#include <uav_utils/uav_utils.hpp>
#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/SimPWM.h>

class RateController
{
private:
    ////////////
    // Variables
    last_letter_msgs::SimStates states_; // Complete aircraft state
    Eigen::Vector3f angularStates_;     // p, q, r measurements
    Eigen::Vector3f airdata_;
    Eigen::Vector3f refRates_, refInputs_;
    Eigen::Matrix<float, 3, 1> predicted_controls_; // Stores control outputs, -1,1 range
    ros::Time tprev;
    ros::Subscriber subState, subRef;
    ros::Publisher pubCtrl;
    float dt_ = 0.02;
    bool statesReceivedStatus_ = false;

    //////////
    // Members
    MpcWrapper<float> mpcController_;

public:

    ////////////
    // Functions
    void step();                                                // Caller of rate_controller_wrapper
    void getStates(last_letter_msgs::SimStates measuredStates); // Callback to store measured states
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command  //TODO: Declare and compile the msg
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    void writeOutput();                                         // Send control signals to the control inputs aggregator

    // Constructor
    RateController(ros::NodeHandle n);
    // Destructor
    ~RateController();
};
