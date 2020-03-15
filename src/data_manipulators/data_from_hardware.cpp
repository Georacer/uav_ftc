#include <Eigen/Eigen>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

#include <rosflight_msgs/RCRaw.h>
#include <rosflight_msgs/OutputRaw.h>
#include <rosflight_msgs/Attitude.h>
#include <rosflight_msgs/GNSS.h>

#include <minidaq/minidaq_peripherals.h>
#include "uav_utils.hpp"

#include "geodetic_conv.hpp"


using Eigen::Vector3d;
using Eigen::Quaterniond;
using geodetic_converter::GeodeticConverter;

class SubHandlerHW : public SubHandler
{
    public:
    ros::Subscriber sub_imu;
    ros::Subscriber sub_imu_temp;
    ros::Subscriber sub_attitude;
    ros::Subscriber sub_minidaq;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_gps_vel;
    ros::Subscriber sub_input;
    ros::Subscriber sub_output;

    SubHandlerHW(ros::NodeHandle n);
    GeodeticConverter geodetic_converter;

    private:
    Vector3d home_coords_;
    bool did_set_home_{false};
    double offset_qbar_;

    void cb_imu(sensor_msgs::Imu msg);
    void cb_imu_temp(sensor_msgs::Temperature msg);
    void cb_attitude(rosflight_msgs::Attitude msg);
    void cb_minidaq(minidaq::minidaq_peripherals msg);
    void cb_gps(sensor_msgs::NavSatFix msg);
    void cb_gps_velocity(geometry_msgs::TwistWithCovarianceStamped msg);
    void cb_input(rosflight_msgs::RCRaw msg);
    void cb_output(rosflight_msgs::OutputRaw msg);
    Vector3d coords_to_ned(Vector3d coords, Vector3d coords_init);

};

SubHandlerHW::SubHandlerHW(ros::NodeHandle n) : SubHandler()
{
    sub_imu = n.subscribe("imu/data", 1, &SubHandlerHW::cb_imu, this);
    sub_imu_temp = n.subscribe("imu/temperature", 1, &SubHandlerHW::cb_imu_temp, this);
    sub_attitude = n.subscribe("attitude", 1, &SubHandlerHW::cb_attitude, this);
    sub_minidaq = n.subscribe("minidaq_peripherals", 1, &SubHandlerHW::cb_minidaq, this); // MiniDAQ subscriber
	sub_gps = n.subscribe("gps/fix",1,&SubHandlerHW::cb_gps, this); // GPS subscriber
	sub_gps_vel = n.subscribe("gps/fix_velocity",1,&SubHandlerHW::cb_gps_velocity, this); // GPS velocity subscriber
	sub_input = n.subscribe("rc_raw",1,&SubHandlerHW::cb_input, this); // Input subscriber
	sub_output = n.subscribe("output_raw",1,&SubHandlerHW::cb_output, this); // Output subscriber

    n.param("offset_qbar", offset_qbar_, 0.0);

    bus_data.g = 9.81; // Hard-set gravity, since no other source exists;

    ROS_INFO("Built SubHandlerHW");
}

void SubHandlerHW::cb_imu(sensor_msgs::Imu msg)
{
    bus_data.acceleration_linear = msg.linear_acceleration; // Body frame, accelerometer reading
}

void SubHandlerHW::cb_imu_temp(sensor_msgs::Temperature msg)
{
    bus_data.temperature_imu = msg.temperature; // In degC
}

void SubHandlerHW::cb_attitude(rosflight_msgs::Attitude msg)
{
    // Copy over orientation internaly
    bus_data.orientation = msg.attitude;
    bus_data.velocity_angular = msg.angular_velocity; // Body frame
}

void SubHandlerHW::cb_minidaq(minidaq::minidaq_peripherals msg)
{
    double press_diff_mbar = double(msg.press_diff-0x666)/0x6ccc*25;
    bus_data.qbar = press_diff_mbar*100 - offset_qbar_; // Convert to Pascal and store
    double qbar_sign = 1;
    if (bus_data.qbar < 0) { qbar_sign = -1; }
    bus_data.rho = 1.225; // Air density
    bus_data.airspeed = qbar_sign*sqrt(2*fabs(bus_data.qbar)/bus_data.rho);

    bus_data.angle_of_attack = double(msg.aoa-2048.0)/4096.0*(2*M_PI);
    bus_data.angle_of_sideslip = double(msg.aos-2048.0)/4096.0*(2*M_PI);
    bus_data.temperature_air = msg.temperature*0.01;
    double press_abs_normalized = double(msg.press_abs-0x666)/0x6ccc;
    double press_abs_mbar = press_abs_normalized*(1100-600)+600;
    bus_data.pressure_absolute = press_abs_mbar*100; // Convert to Pascal
    bus_data.rps_motor = msg.rps; // Currently only for first motor
    // Disregard RC output information, available through ROSFlight topics

    // Estimate force-torque data based on rps?
    // bus_data.propulsion.force = 
    // bus_data.propulsion.torque = 

    flag_got_airdata = true;
}

void SubHandlerHW::cb_gps(sensor_msgs::NavSatFix msg)
{
    if (msg.status.status == msg.status.STATUS_FIX)
    {
        if (!did_set_home_)
        {
            geodetic_converter.initialiseReference(msg.latitude, msg.longitude, msg.altitude);
            did_set_home_ = true;
        }
        bus_data.coordinates.x = msg.latitude;
        bus_data.coordinates.y = msg.longitude;
        bus_data.coordinates.z = msg.altitude;

        double n, e, d;
        geodetic_converter.geodetic2Ned(msg.latitude, msg.longitude, msg.altitude,
                                        &n, &e, &d);

        geometry_msgs::Point temp_point;
        temp_point.x = n;
        temp_point.y = e;
        temp_point.z = d;
        bus_data.position = temp_point;

        // Use another message to capture velocity messages
        // bus_data.inertial_velocity.x = msg.pose.position.x;// NED frame
        // bus_data.inertial_velocity.y = msg.pose.position.y;// NED frame
        // bus_data.inertial_velocity.z = msg.pose.position.z;// NED frame

        // flag_got_gps = true; // EKF doesn't need GPS position
    }
    else
    {
        ROS_WARN_THROTTLE(10, "GPS didn't achieve 3D fix yet");
    }

}

void SubHandlerHW::cb_gps_velocity(geometry_msgs::TwistWithCovarianceStamped msg)
{
    if (did_set_home_)
    {
        bus_data.inertial_velocity.x = msg.twist.twist.linear.x;
        bus_data.inertial_velocity.y = msg.twist.twist.linear.y;
        bus_data.inertial_velocity.z = msg.twist.twist.linear.z;

        flag_got_gps = true;  // Let EKF know a new GPS velocity measurement is available
    }
    else
    {
        ROS_WARN_THROTTLE(10, "GPS didn't set home yet");
    }

}

void SubHandlerHW::cb_input(rosflight_msgs::RCRaw msg)
{
    for (uint i=0; i<8; ++i)
    {
        bus_data.rc_in[i] = msg.values[i] + 1000;
    }
}

void SubHandlerHW::cb_output(rosflight_msgs::OutputRaw msg)
{
    // ROSFlight output topic uses (-1,1) and (0,1) representation
    for (uint i=0; i<8; ++i)
    {
        bus_data.rc_out[i] = msg.values[i]*500 + 1500;
    }
    bus_data.rc_out[2] = msg.values[2]*1000 + 1000;
}

// Convert gps coordinates to local frame NED position
Vector3d coords_to_ned(Vector3d coords, Vector3d coords_init)
{
    Vector3d position_ned;

}