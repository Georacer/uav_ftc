#include <Eigen/Eigen>

#include <sensor_msgs/Imu.h>
#include <rosflight_msgs/RCRaw.h>
#include <rosflight_msgs/OutputRaw.h>
#include <rosflight_msgs/Attitude.h>
#include <rosflight_msgs/GNSS.h>

#include <minidaq/minidaq_peripherals.h>
#include "uav_utils.hpp"

#include "geodetic_conv.hpp"


using Eigen::Vector3d;
using Eigen::Quaterniond;
using geodetic_converter::GeodeticConverter

class SubHandlerHW : public SubHandler
{
    public:
    ros::Subscriber sub_imu;
    ros::Subscriber sub_imu_temp;
    ros::Subscriber sub_attitude;
    ros::Subscriber sub_minidaq;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_input;
    ros::Subscriber sub_output;

    SubHandlerHW(ros::NodeHandle n);
    GeodeticConverter geodetic_converter;

    private:
    Vector3d home_coords_;
    bool did_set_home_{false};

    void cb_imu(sensor_msgs::Imu msg);
    void cb_imu_temp(sensor_msgs::Temperature msg);
    void cb_attitude(rosflight_msgs::Attitude msg);
    void cb_minidaq(minidaq::minidaq_peripherals msg);
    void cb_gps(sensor_msgs::navSatFix msg);
    void cb_input(rosflight_msgs::RCRaw msg);
    void cb_output(last_letter_msgs::SimPWM msg);
    Vector3d coords_to_ned(Vector3d coords, Vector3d coords_init);

};

SubHandlerHW::SubHandlerHW(ros::NodeHandle n) : SubHandler()
{
    sub_imu = n.subscribe("imu/data", 1, &SubHandlerHW::cb_imu, this);
    sub_imu_temp = n.subscribe("imu/temperature", 1, &SubHandlerHW::cb_imu_temp, this);
    sub_attitude = n.subscribe("attitude", 1, &SubHandlerHW::cb_attitude, this);
    sub_minidaq = n.subscribe("minidaq_peripherals", 1, &SubHandlerHW::cb_minidaq, this); // MiniDAQ subscriber
	sub_input = n.subscribe("gps",1,&SubHandlerHW::cb_gps, this); // GPS subscriber
	sub_input = n.subscribe("rc_raw",1,&SubHandlerHW::cb_input, this); // Input subscriber
	sub_output = n.subscribe("output_raw",1,&SubHandlerHW::cb_output, this); // Output subscriber

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
    bus_data.qbar = msg.press_diff;
    bus_data.rho = 1.225; // Air density
    bus_data.airspeed = sqrt(2*msg.press_diff/bus_data.rho);
    bus_data.angle_of_attack = msg.aoa;
    bus_data.angle_of_sideslip = msg.aos;
    bus_data.temperature_air = msg.temperature;
    bus_data.pressure_absolute = msg.press_abs;
    bus_data.rps_motor = msg.rps; // Currently only for first motor
    // Disregard RC output information, available through ROSFlight topics

    // Estimate force-torque data based on rps?
    // bus_data.propulsion.force = 
    // bus_data.propulsion.torque = 

    flag_got_airdata = true;
}

void SubHandlerHW::cb_gps(sensor_msgs::NavSatFix msg)
{
    if (msg.fix == msg.FIX_TYPE_3D)
    {
        if (!did_set_home_)
        {
            geodetic_converter.initializeReference(msg.latitude, msg.longitude, msg.alitutde);
            did_set_home_ = true;
        }
        double n, e, d;
        geodetic_converter.geodetic2Ned(msg.latitude, msg.longitdue, msg.altitude,
                                       &n, &e, &d);

        geometry_msgs::Vector3 temp_vect_ros(n, e, d);
        bus_data.position = temp_vect_ros;

        // Use another message to capture velocity messages
        // bus_data.inertial_velocity.x = msg.pose.position.x;// NED frame
        // bus_data.inertial_velocity.y = msg.pose.position.y;// NED frame
        // bus_data.inertial_velocity.z = msg.pose.position.z;// NED frame

        flag_got_gps = true;
    }
    else
    {
        ROS_WARN("GPS didn't achieve 3D fix yet");
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