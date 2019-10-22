#include <Eigen/Eigen>
#include <ros/ros.h>

#include <uav_ftc/BusData.h>

enum data_source{
    DATA_SOURCE_HW,
    DATA_SOURCE_LL
};

class SubHandler
{
    public:
    uav_ftc::BusData bus_data;

    uav_ftc::BusData get_data();
    SubHandler() {};

};

class DataBus
{
    // Variables
    private:
    double pub_rate_;
    ros::Publisher data_pub_;
    uav_ftc::BusData bus_data_; // Contains aggregated output data
    SubHandler sub_handler_;

    public:

    // Methods

    DataBus(ros::NodeHandle h, uint data_source); // Constructor
    void publish_data(); // Data publisher
    void set_pub_rate(double rate);
    void run(); // Spin constantly
};