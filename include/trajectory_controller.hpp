#include <ros/ros.h>
#include "mpc_wrapper.hpp"

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

#include <uav_ftc/BusData.h>
#include <uav_ftc/FlightEnvelopeEllipsoid.h>
#include <last_letter_msgs/Environment.h>
#include <last_letter_msgs/Parameter.h>
#include <math_utils.hpp>

// Declare the order of OnlineData in the solver code
// Used to easily access the online data array elements
// WARNING: enumerator order must be the same as declared in the solver code
enum class Parameter {
    // Inertial data
    m=0,
    // Geometric data
    S,
    b,
    c,
    // Lift parameters
    c_lift_0,
    c_lift_a,
    // Drag parameters
    c_drag_0,
    c_drag_a,
    // Sideforce parameters
    c_y_0,
    c_y_b,
    // Ellipsoid Flight Envelope parameters
    el_A,
    el_B,
    el_C,
    el_D,
    el_E,
    el_F,
    el_G,
    el_H,
    el_I,
    el_J
};


// Vector containing the corresponding names of the Online Data components
// Starts with UAV parameters and end with Flight Envelope parameters
std::vector<std::string> online_data_names {
    "m",
    "s",
    "b",
    "c",
    "c_L_0",
    "c_L_alpha",
    "c_D_0",
    "c_D_alpha",
    "c_Y_0",
    "c_Y_beta",
    "el_A",
    "el_B",
    "el_C",
    "el_D",
    "el_E",
    "el_F",
    "el_G",
    "el_H",
    "el_I",
    "el_J"
};


// Declare the order of constraints in the solver code
// Used to easily access and set the constraint limits
// WARNING: enumerator order must be the same as declared in the solver code
enum class Constraint {
    // Input constraints
    deltaF = 0,
    p,
    q,
    r,
    // State constraints
    alpha,
    beta,
    phi,
    theta
};


class TrajectoryController
{
private:
    ////////////
    // Variables
    uav_ftc::BusData bus_data_; // Complete aircraft state
    Eigen::Matrix<float, kStateSize, 1> states_; // Va, alpha, beta, phi, theta
    Eigen::Matrix<real_t, kOdSize, 1> trimOnlineData_; // Vector holding default Online Data values
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> defaultBoundsMin_; // Vector holding default min bounds
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> defaultBoundsMax_; // Vector holding default max bounds
    Eigen::Vector3f airdata_;
    Eigen::Vector3d wind_body_;
    Eigen::Matrix<float, kRefSize, 1> reference_; // Va, gamma, psi_dot, alpha, beta
    Eigen::Matrix<float, kEndRefSize, 1> endReference_;
    Eigen::Vector3f referenceTrajectory_; // Va, gamma, R
    Eigen::Vector4f refInputs_; // Stores reference inputs
    Eigen::Vector4f predictedControls_; // Stores control outputs
    Ellipsoid3DCoefficients_t default_fe_coeffs_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0};  // Assign coeffs which always evaluate true
    Ellipsoid3D fe_ellipsoid_;
    ros::Time tprev;
    ros::Subscriber subState, subRef, subEnvironment, subParam, subFE;
    ros::Publisher pubCmdRates, pubCmdThrottle;
    float dt_ = 0.2;
    int numConstraints_ = 4;  // Number of input constraints
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
    void getFlightEnvelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr&); // Callback to capture Flight Envelope ellipsoid parameters
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    void writeOutput();                                         // Send control signals to the control inputs aggregator
    void getDefaultParameters(std::string uavName);             // Read necessary UAV parameters
    bool getDefaultBounds(ros::NodeHandle pnh);                 // Read default state and input bounds
    float calcOmega(const float thrust);                        // Calculate required RPM from desired thrust

    // Constructor
    TrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh);
    // Destructor
    ~TrajectoryController();
};

float calcPsiDotDes(Eigen::Vector3f refTrajectory);
float calcPsiDot(Eigen::Matrix<float, kStateSize, 1> states, Eigen::Matrix<float, kInputSize, 1> inputs);