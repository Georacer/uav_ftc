#include <Eigen/Eigen>
#include <ros/ros.h>

#include <uav_ftc/BusData.h>

#include "../../src/ekf.cpp"

enum data_source{
    DATA_SOURCE_HW,
    DATA_SOURCE_LL
};

class SubHandler
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uav_ftc::BusData bus_data;
    // Measurement availability flags
    bool flag_got_gps, flag_got_airdata;
    uav_ftc::BusData get_data() {};
    SubHandler();
};

class DataBus
{

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Settings {
        uint data_source;
        uint estimate_rps{0};
        uint estimate_airspeed{0};
        uint estimate_aoa{0};
        uint estimate_aos{0};
    };

    // Methods

    DataBus(ros::NodeHandle h, const Settings &opts); // Constructor
    ~DataBus();

    void ekf_init_();
    void ekf_build_measurement_matrix();
    void ekf_build_measurements();
    void ekf_build_inputs();
    void ekf_step(); // iterate the EKF

    void get_handler_data();
    void publish_data(); // Data publisher
    void set_pub_rate(double rate);
    void run(); // Spin constantly

    // Variables
    private:
    ros::NodeHandle n;
    ros::NodeHandle pnh;
    double pub_rate_;
    ros::Publisher data_pub_;
    ros::Publisher ekf_pub_;
    bool ekf_enable = true;
    uav_ftc::BusData bus_data_; // Contains aggregated output data
    SubHandler * sub_handler_;

    Ekf * ekf_;
    Eigen::Matrix<double, 10, 1> ekf_u_;
    Eigen::Matrix<double, 6, 1> ekf_y_;
    Eigen::Matrix<double, 6, 6> ekf_D_;
    uav_ftc::BusData ekf_data_;
};