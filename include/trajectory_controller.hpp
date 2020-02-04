#include <ros/ros.h>
#include "mpc_wrapper.hpp"

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

#include <uav_ftc/BusData.h>
#include <last_letter_msgs/Environment.h>
#include <last_letter_msgs/Parameter.h>

// Declare the order of OnlineData in the solver code
// Used to easily access the online data array elements
// WARNING: enumerator order must be the same as declared in the solver code
enum class Parameter {
    // Geometric data
    S=0,
    b,
    c,
    // Inertial data
    m,
    // Lift parameters
    c_lift_0,
    c_lift_a,
    // Drag parameters
    c_drag_0,
    c_drag_a,
    // Sideforce parameters
    c_y_0,
    c_y_b
};

class TrajectoryController
{
private:
    ////////////
    // Variables
    uav_ftc::BusData bus_data_; // Complete aircraft state
    Eigen::Matrix<float, kStateSize, 1> states_; // Va, alpha, beta, phi, theta
    Eigen::Vector3f airdata_;
    Eigen::Vector3d wind_body_;
    Eigen::Matrix<float, kRefSize, 1> reference_; // Va, gamma, psi_dot, alpha, beta
    Eigen::Matrix<float, kEndRefSize, 1> endReference_;
    Eigen::Vector3f referenceTrajectory_; // Va, gamma, R
    Eigen::Vector4f refInputs_; // Stores reference inputs
    Eigen::Vector4f predictedControls_; // Stores control outputs
    ros::Time tprev;
    ros::Subscriber subState, subRef, subEnvironment, subParam;
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
    void getStates(uav_ftc::BusData bus_data);                  // Callback to store measured states
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command
    void getParameters(last_letter_msgs::Parameter parameter);  // Callback to capture parameter changes
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    void writeOutput();                                         // Send control signals to the control inputs aggregator
    void getDefaultParameters(std::string uavName);             // Read necessary UAV parameters
    float calcOmega(const float thrust);                        // Calculate required RPM from desired thrust

    // Constructor
    TrajectoryController(ros::NodeHandle n);
    // Destructor
    ~TrajectoryController();
};

float calcPsiDotDes(Eigen::Vector3f refTrajectory);
float calcPsiDot(Eigen::Matrix<float, kStateSize, 1> states, Eigen::Matrix<float, kInputSize, 1> inputs);