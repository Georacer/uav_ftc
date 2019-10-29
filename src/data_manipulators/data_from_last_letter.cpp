#include <Eigen/Eigen>

#include "uav_utils.hpp"

#include "last_letter_msgs/SimStates.h"
#include "last_letter_msgs/SimWrenches.h"
#include "last_letter_msgs/SimPWM.h"
#include "last_letter_msgs/Environment.h"

using Eigen::Vector3d;
using Eigen::Quaterniond;


class SubHandlerLL : public SubHandler
{
    public:
    ros::Subscriber sub_state, sub_state_dot, sub_environment, sub_wrench, sub_input, sub_output;

    SubHandlerLL(ros::NodeHandle n);

    private:
    Vector3d wind_body, vel_body_inertial, vel_body_relative;
    Quaterniond rotation_be;

    void cb_state(last_letter_msgs::SimStates msg);
    void cb_state_dot(last_letter_msgs::SimStates msg);
    void cb_environment(last_letter_msgs::Environment msg);
    void cb_wrench(last_letter_msgs::SimWrenches msg);
    void cb_input(last_letter_msgs::SimPWM msg);
    void cb_output(last_letter_msgs::SimPWM msg);
};

SubHandlerLL::SubHandlerLL(ros::NodeHandle n) : SubHandler()
{
    // Variable initialization
    wind_body = Vector3d::Zero();
    vel_body_inertial = Vector3d::Zero();
    vel_body_relative = Vector3d::Zero(); // Body velocity relative to air-mass
    rotation_be = Quaterniond::Identity();

	sub_state = n.subscribe("states",1,&SubHandlerLL::cb_state, this); // State subscriber
    ROS_INFO("Subscribed to state %s", sub_state.getTopic().c_str());
	sub_state_dot = n.subscribe("statesDot",1,&SubHandlerLL::cb_state_dot, this); // State derivatives subscriber
	sub_environment = n.subscribe("environment",1,&SubHandlerLL::cb_environment, this); // Environment subscriber
	sub_wrench = n.subscribe("wrenches",1,&SubHandlerLL::cb_wrench, this); // Input wrenches subscriber
	sub_input = n.subscribe("rawPWM",1,&SubHandlerLL::cb_input, this); // Input subscriber
	sub_output = n.subscribe("ctrlPWM",1,&SubHandlerLL::cb_output, this); // Output subscriber

    ROS_INFO("Built SubHandlerLL");
}

void SubHandlerLL::cb_state(last_letter_msgs::SimStates msg)
{
    geometry_msgs::Vector3 temp_vect_ros;
    Vector3d temp_vect_eig;

    temp_vect_ros.x = msg.geoid.latitude;
    temp_vect_ros.y = msg.geoid.longitude;
    temp_vect_ros.z = msg.geoid.altitude;
    bus_data.coordinates = temp_vect_ros;
    bus_data.position = msg.pose.position; // NED frame
    bus_data.orientation = msg.pose.orientation; 
    // Copy over orientation internaly
    rotation_be.x() = msg.pose.orientation.x;
    rotation_be.y() = msg.pose.orientation.y;
    rotation_be.z() = msg.pose.orientation.z;
    rotation_be.w() = msg.pose.orientation.w;
    bus_data.velocity_angular = msg.velocity.angular; // Body frame
    bus_data.rps_motor = msg.rotorspeed[0]; // Currently only for first motor

    temp_vect_eig = rotation_be * wind_body;
    convertRosVector3(temp_vect_eig, bus_data.wind); // NED frame
    // temp_vect_ros.x = temp_vect_eig.x();
    // temp_vect_ros.y = temp_vect_eig.y();
    // temp_vect_ros.z = temp_vect_eig.z();
    // bus_data.wind =  temp_vect_ros; // NED frame

    vel_body_inertial = Vector3d(msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z); // Body frame
    vel_body_relative = vel_body_inertial - wind_body;
    Vector3d airdata = getAirData(vel_body_relative);
    bus_data.airspeed = airdata.x();
    bus_data.qbar = 0.5*bus_data.rho*airdata.x()*airdata.x();
    bus_data.angle_of_attack = airdata.y();
    bus_data.angle_of_sideslip = airdata.z();

    flag_got_gps = true;
    flag_got_airdata = true;
}

void SubHandlerLL::cb_state_dot(last_letter_msgs::SimStates msg)
{
    bus_data.inertial_velocity.x = msg.pose.position.x;// NED frame
    bus_data.inertial_velocity.y = msg.pose.position.y;// NED frame
    bus_data.inertial_velocity.z = msg.pose.position.z;// NED frame
    bus_data.acceleration_angular = msg.velocity.angular; // Body frame
    bus_data.acceleration_linear = msg.velocity.linear; // Body frame
}

void SubHandlerLL::cb_environment(last_letter_msgs::Environment msg)
{
    wind_body.x() = msg.wind.x;
    wind_body.y() = msg.wind.y;
    wind_body.z() = msg.wind.z;
    bus_data.temperature_air = msg.temperature;
    bus_data.temperature_imu = msg.temperature;
    bus_data.pressure_absolute = msg.pressure;
    bus_data.rho = msg.density; // Air density
    bus_data.g = msg.gravity; // Gravity acceleration
}

void SubHandlerLL::cb_wrench(last_letter_msgs::SimWrenches msg)
{
    bus_data.propulsion.force = msg.propulsion.force;
    bus_data.propulsion.torque = msg.propulsion.torque;
}

void SubHandlerLL::cb_input(last_letter_msgs::SimPWM msg)
{
    for (uint i=0; i<8; ++i)
    {
        bus_data.rc_in[i] = msg.value[i];
    }
}

void SubHandlerLL::cb_output(last_letter_msgs::SimPWM msg)
{
    for (uint i=0; i<8; ++i)
    {
        bus_data.rc_out[i] = msg.value[i];
    }
}
