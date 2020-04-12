#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

#include <uav_ftc/BusData.h>
#include <uav_ftc/FlightEnvelopeEllipsoid.h>
#include <math_utils.hpp>
#include <uav_utils.hpp>


class Controller
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ////////////
    // Functions
    void step();                                                // Caller of rate_controller_wrapper
    void getStates(uav_ftc::BusData bus_data);                  // Callback to store measured states
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command
    void getFlightEnvelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr&); // Callback to capture Flight Envelope ellipsoid parameters
    void getDefaultParameters(std::string uavName);             // Read default uav parameters and pass them to the MPC
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    double calcOmega(const double thrust);                       // Calculate the rpms required to produce thrust
    void writeOutput();                                         // Send control signals to the control inputs aggregator

    // Constructor
    Controller(ros::NodeHandle n, ros::NodeHandle pnh);
    // Destructor
    ~Controller();

private:
    ////////////
    // Variables
    uav_ftc::BusData states_; // Complete aircraft state
    Eigen::Vector3d reference_; // Va, gamma, psi_dot references
    Eigen::Matrix<double, 4, 1> controls_; // Stores control outputs, -1,1 range
    double deltae_, deltaF_, deltar_;
    ros::Time tprev;
    ros::Subscriber subState, subRef, subFE;
    ros::Publisher pubCtrl, pubCmdThrottle;
    PID * pid_va_;
    PID * pid_gamma_;
    PID * pid_psi_dot_;
    Ellipsoid3DCoefficients_t default_fe_coeffs_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0};  // Assign coeffs which always evaluate true
    Ellipsoid3D fe_ellipsoid_;
    bool statesReceivedStatus_ = false;
    double propD; // Propeller diameter
    double rho = 1.225; // Air density
    double omega_max; // Maximum engine rad/s
    double c_t_0, c_t_1, c_t_2; // Thrust polynomial coefficients

    //////////
    // Members
};
