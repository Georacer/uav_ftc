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

class TrajectoryController
{
private:
    ////////////
    // Variables
    last_letter_msgs::SimStates simStates_; // Complete aircraft state
    Eigen::Matrix<float, 5, 1> states_; // Va, alpha, beta, phi, theta
    Eigen::Vector3f airdata_;
    Eigen::Matrix<float, 9, 1> reference_; // Va, gamma, psi_dot, alpha, beta
    Eigen::Vector2f endReference_;
    Eigen::Vector3f referenceTrajectory_; // Va, gamma, R
    Eigen::Vector4f refInputs_; // Stores reference inputs
    Eigen::Vector4f predictedControls_; // Stores control outputs
    ros::Time tprev;
    ros::Subscriber subState, subRef;
    ros::Publisher pubCmdRates, pubCmdThrottle;
    float dt_ = 0.2;
    int numConstraints_ = 4;
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
    TrajectoryController(ros::NodeHandle n);
    // Destructor
    ~TrajectoryController();
};

float calcPsiDot(Eigen::Vector3f refTrajectory);