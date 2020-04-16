#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>
#include <iir/Butterworth.h>

#include <uav_utils.hpp>
#include <uav_ftc/BusData.h>


class RateController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ////////////
    // Functions
    void step();                                                // Caller of rate_controller_wrapper
    void getStates(uav_ftc::BusData bus_data);                  // Callback to store measured states
    void getReference(geometry_msgs::Vector3Stamped reference); // Callback to store reference command
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
    Eigen::Vector3d refRates_;
    Eigen::Vector3d controls_; // Stores control outputs, -1,1 range
    double deltaa_, deltae_, deltar_;
    ros::Time tprev;
    ros::Subscriber subState, subRef;
    ros::Publisher pubCtrl;
    float dt_ = 0.02;
    bool statesReceivedStatus_ = false;

    static const int filter_order_{2};
    const float f_sample_ = 100;
    const float f_cut_ = 10;
    Iir::Butterworth::LowPass<filter_order_> filt_p_, filt_q_, filt_r_;

    double pid_p_kp_{0.6}, pid_p_ki_{0.06}, pid_p_kd_{0};
    double pid_q_kp_{0.6}, pid_q_ki_{0.06}, pid_q_kd_{0};
    double pid_r_kp_{1.2}, pid_r_ki_{0.12}, pid_r_kd_{0};

    //////////
    // Members
    PID * pid_p_;
    PID * pid_q_;
    PID * pid_r_;
};
