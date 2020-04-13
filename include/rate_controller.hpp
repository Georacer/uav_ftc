#include <ros/ros.h>
#include "mpc_wrapper.hpp"

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>
#include <iir/Butterworth.h>

#include <uav_ftc/BusData.h>
#include <last_letter_msgs/Parameter.h>

// Declare the order of OnlineData in the solver code
// Used to easily access the online data array elements
// WARNING: enumerator order must be the same as declared in the solver code
enum class Parameter {
    Va = 0,
    alpha,
    beta,
    t_prop,
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

// Vector containing the corresponding names of the Online Data components
// Starts with variables and states and ends with UAV parameters
std::vector<std::string> online_data_names {
    "Va",
    "alpha",
    "beta",
    "t_prop",
    "c_l_0",
    "c_l_pn",
    "c_l_beta",
    "c_l_rn",
    "c_l_deltaa",
    "c_l_deltar",
    "c_m_0",
    "c_m_alpha",
    "c_m_qn",
    "c_m_deltae",
    "c_n_0",
    "c_n_beta",
    "c_n_pn",
    "c_n_rn",
    "c_n_deltaa",
    "c_n_deltar"
};

class RateController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ////////////
    // Functions
    void step();                                                // Caller of rate_controller_wrapper
    void getStates(uav_ftc::BusData bus_data);                  // Callback to store measured states
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command
    void getDefaultWeights(ros::NodeHandle pnh);
    void getDefaultParameters(std::string uavName);             // Read default uav parameters and pass them to the MPC
    void getParameters(last_letter_msgs::Parameter parameter);  // Callback to store estimated parameters
    bool getDefaultBounds(ros::NodeHandle pnh);                 // Read default state and input bounds
    void readControls();                                        // Read the resulting predicted output from the RateMpcWrapper
    void writeOutput();                                         // Send control signals to the control inputs aggregator

    // Constructor
    RateController(ros::NodeHandle n, ros::NodeHandle pnh);
    // Destructor
    ~RateController();

private:
    ////////////
    // Variables
    uav_ftc::BusData states_; // Complete aircraft state
    Eigen::Vector3f angularStates_;     // p, q, r measurements
    Eigen::Matrix<real_t, kOdSize, 1> trimOnlineData_; // Vector holding default Online Data values
    double thrust_moment_{0}; // Downwards moment created by the thrust vector
    Eigen::Vector3f refRates_, refInputs_;
    Eigen::Matrix<float, kInputSize, 1> predicted_controls_; // Stores control outputs, -1,1 range
    float deltaa_max, deltae_max, deltar_max;
    ros::Time tprev;
    ros::Subscriber subState, subRef, subParam;
    ros::Publisher pubCtrl;
    float dt_ = 0.02;
    int numConstraints_ = 3;
    bool statesReceivedStatus_ = false;

    static const int filter_order_{2};
    const float f_sample_ = 100;
    const float f_cut_ = 10;
    Iir::Butterworth::LowPass<filter_order_> filt_va_, filt_alpha_, filt_beta_;
    Iir::Butterworth::LowPass<filter_order_> filt_p_, filt_q_, filt_r_;
    Iir::Butterworth::LowPass<filter_order_> filt_ty_;

    //////////
    // Members
    MpcWrapper<float> mpcController_;
};
