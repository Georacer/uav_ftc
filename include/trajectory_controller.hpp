#include <ros/ros.h>
#include "mpc_wrapper.hpp"

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/Environment.h>

#define NUM_STATES 5
#define NUM_REFS 9
#define NUM_END_REFS 3
#define NUM_INPUTS 4

class TrajectoryController
{
private:
    ////////////
    // Variables
    last_letter_msgs::SimStates simStates_; // Complete aircraft state
    Eigen::Matrix<float, NUM_STATES, 1> states_; // Va, alpha, beta, phi, theta
    Eigen::Vector3f airdata_;
    Eigen::Vector3d wind_body_;
    Eigen::Matrix<float, NUM_REFS, 1> reference_; // Va, gamma, psi_dot, alpha, beta
    Eigen::Matrix<float, NUM_END_REFS, 1> endReference_;
    Eigen::Vector3f referenceTrajectory_; // Va, gamma, R
    Eigen::Vector4f refInputs_; // Stores reference inputs
    Eigen::Vector4f predictedControls_; // Stores control outputs
    ros::Time tprev;
    ros::Subscriber subState, subRef, subEnvironment;
    ros::Publisher pubCmdRates, pubCmdThrottle;
    float dt_ = 0.2;
    int numConstraints_ = 4;
    bool statesReceivedStatus_ = false;
    float propD; // Propeller diameter
    float rho = 1.225; // Air density
    float omega_max; // Maximum engine rad/s
    float c_t_0, c_t_1, c_t_2; // Thrust polynomial coefficients

    //////////
    // Members
    MpcWrapper<float> mpcController_;

public:

    ////////////
    // Functions
    void step();                                                // Caller of rate_controller_wrapper
    void getStates(last_letter_msgs::SimStates measuredStates); // Callback to store measured states
    void getEnvironment(last_letter_msgs::Environment msg);     // Callback to store environment values
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command  //TODO: Declare and compile the msg
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    void writeOutput();                                         // Send control signals to the control inputs aggregator
    void getDefaultParameters(std::string uavName);             // Read necessary UAV parameters
    float calcOmega(const float thrust);                               // Calculate required RPM from desired thrust

    // Constructor
    TrajectoryController(ros::NodeHandle n);
    // Destructor
    ~TrajectoryController();
};

float calcPsiDotDes(Eigen::Vector3f refTrajectory);
float calcPsiDot(Eigen::Matrix<float, NUM_STATES, 1> states, Eigen::Matrix<float, NUM_INPUTS, 1> inputs);