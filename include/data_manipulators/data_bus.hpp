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
    uav_ftc::BusData bus_data;
    // Measurement availability flags
    bool flag_got_gps, flag_got_airdata;
    Eigen::Matrix<double, 6, 1> ekf_u_;
    Eigen::Matrix<double, 6, 1> ekf_y_;
    Eigen::Matrix<double, 6, 6> ekf_D_;

    uav_ftc::BusData get_data() {};
    SubHandler()
    {
        flag_got_gps = false;
        flag_got_airdata = false;
    };
    void ekf_build_measurement_matrix();
    void ekf_build_measurements();
    void ekf_build_inputs();


};

class DataBus
{
    // Variables
    private:
    double pub_rate_;
    ros::Publisher data_pub_;
    ros::Publisher ekf_pub_;
    Ekf * ekf_;
    uav_ftc::BusData ekf_data_;
    uav_ftc::BusData bus_data_; // Contains aggregated output data
    SubHandler * sub_handler_;

    public:

    // Methods

    DataBus(ros::NodeHandle h, uint data_source); // Constructor
    ~DataBus();
    void ekf_init_();
    void ekf_step(); // iterate the EKF
    void publish_data(); // Data publisher
    void set_pub_rate(double rate);
    void run(); // Spin constantly
};