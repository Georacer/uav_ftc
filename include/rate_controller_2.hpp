#include <ros/ros.h>
#include "mpc_wrapper.hpp"

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/Parameter.h>

// Declare the order of OnlineData in the solver code
// Used to easily access the online data array elements
// WARNING: enumerator order must be the same as declared in the solver code
enum class Parameter {
    Va = 0,
    alpha,
    beta,
    c_l_0,
    c_l_p,
    c_l_b,
    c_l_r,
    c_l_de,
    c_l_dr,
    c_m_0,
    c_m_a,
    c_m_q,
    c_m_de,
    c_n_0,
    c_n_b,
    c_n_p,
    c_n_r,
    c_n_da,
    c_n_dr
};

class RateController
{
private:
    ////////////
    // Variables
    last_letter_msgs::SimStates states_; // Complete aircraft state
    Eigen::Vector3f angularStates_;     // p, q, r measurements
    // Eigen::Vector3f airdata_;
    Eigen::Vector3f refRates_, refInputs_;
    Eigen::Matrix<float, kInputSize, 1> predicted_controls_; // Stores control outputs, -1,1 range
    ros::Time tprev;
    ros::Subscriber subState, subRef, subParam;
    ros::Publisher pubCtrl;
    float dt_ = 0.02;
    int numConstraints_ = 19; // TODO: Does this need fixing?
    bool statesReceivedStatus_ = false;

    //////////
    // Members
    MpcWrapper<float> mpcController_;

public:

    ////////////
    // Functions
    void step();                                                // Caller of rate_controller_wrapper
    void getStates(last_letter_msgs::SimStates measuredStates); // Callback to store measured states
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command
    void getDefaultParameters(std::string uavName);             // Read default uav parameters and pass them to the MPC
    void getParameters(last_letter_msgs::Parameter parameter);  // Callback to store estimated parameters
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    void writeOutput();                                         // Send control signals to the control inputs aggregator

    // Constructor
    RateController(ros::NodeHandle n);
    // Destructor
    ~RateController();
};
