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

    SubHandlerLL(ros::NodeHandle n, const DataBus::Settings &opts);

    private:
    Vector3d wind_body, vel_body_inertial, vel_body_relative;
    Quaterniond rotation_be;
    // Variable estimation flags
    uint _estimate_rps{0};
    uint _estimate_airspeed{0};
    uint _estimate_aoa{0};
    uint _estimate_aos{0};
    // Variable estimates initialization
    double _aoa_estimated{0};
    double _aos_estimated{0};

    void cb_state(last_letter_msgs::SimStates msg);
    void cb_state_dot(last_letter_msgs::SimStates msg);
    void cb_environment(last_letter_msgs::Environment msg);
    void cb_wrench(last_letter_msgs::SimWrenches msg);
    void cb_input(last_letter_msgs::SimPWM msg);
    void cb_output(last_letter_msgs::SimPWM msg);
    double estimate_aoa_mahony(double Va, double q);
    double estimate_aoa_mine(double Va, double q);
    double estimate_aos(double Va);
};

SubHandlerLL::SubHandlerLL(ros::NodeHandle n, const DataBus::Settings &opts) : SubHandler()
{
    // Variable initialization
    wind_body = Vector3d::Zero();
    vel_body_inertial = Vector3d::Zero();
    vel_body_relative = Vector3d::Zero(); // Body velocity relative to air-mass
    rotation_be = Quaterniond::Identity();

    _estimate_rps = opts.estimate_rps;
    if (_estimate_rps) { ROS_INFO("Estimating RPS");}
    _estimate_airspeed = opts.estimate_airspeed;
    if (_estimate_airspeed==1) { ROS_INFO("Estimating airspeed using Mahony's filter");}
    if (_estimate_airspeed==2) { ROS_INFO("Estimating airspeed using Zogopoulos' filter");}
    _estimate_aoa = opts.estimate_aoa;
    if (_estimate_aoa) { ROS_INFO("Estimating AoA");}
    _estimate_aos = opts.estimate_aos;
    if (_estimate_aos) { ROS_INFO("Estimating AoS");}

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

    if (!_estimate_rps) {
        bus_data.rps_motor = msg.rotorspeed[0]; // Currently only for first motor
    }
    else {
        // Estimate rps based on last available motor command
        // Calculate based on omega vs deltat coefficient
        bus_data.rps_motor = 1050.0/2000.0*bus_data.rc_out[2] / (2*M_PI); 
    }

    temp_vect_eig = rotation_be * wind_body;
    convertRosVector3(temp_vect_eig, bus_data.wind); // NED frame
    // temp_vect_ros.x = temp_vect_eig.x();
    // temp_vect_ros.y = temp_vect_eig.y();
    // temp_vect_ros.z = temp_vect_eig.z();
    // bus_data.wind =  temp_vect_ros; // NED frame

    vel_body_inertial = Vector3d(msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z); // Body frame
    vel_body_relative = vel_body_inertial - wind_body;
    Vector3d airdata = getAirData(vel_body_relative);
    if (!_estimate_airspeed) {
        bus_data.airspeed = airdata.x();
    }
    else {
        bus_data.airspeed = vel_body_inertial.norm();//TODO: Take constant wind component into account
    }
    bus_data.qbar = 0.5*bus_data.rho*bus_data.airspeed*bus_data.airspeed;

    if (!_estimate_aoa) { // No AoA estimation
        bus_data.angle_of_attack = airdata.y();
    }
    else if (_estimate_aoa == 1) { // Mahony's estimator
        bus_data.angle_of_attack = estimate_aoa_mahony(bus_data.airspeed, bus_data.velocity_angular.y);
    }
    else if (_estimate_aoa == 2) { // My estimator
        bus_data.angle_of_attack = estimate_aoa_mine(bus_data.airspeed, bus_data.velocity_angular.y);
    }
    else {
        ROS_ERROR("Invalid AoA estimator flag: %d", _estimate_aoa);
    }

    if (!_estimate_aos) {
        bus_data.angle_of_sideslip = airdata.z();
    }
    else {
        bus_data.angle_of_sideslip = estimate_aos(bus_data.airspeed);
    }

    flag_got_gps = true;
    flag_got_airdata = true;
}

void SubHandlerLL::cb_state_dot(last_letter_msgs::SimStates msg)
{
    geometry_msgs::Vector3 temp_vect_ros;
    Vector3d temp_vect_eig, temp_vect_omega;

    bus_data.inertial_velocity.x = msg.pose.position.x;// NED frame
    bus_data.inertial_velocity.y = msg.pose.position.y;// NED frame
    bus_data.inertial_velocity.z = msg.pose.position.z;// NED frame
    bus_data.acceleration_angular = msg.velocity.angular; // Body frame
    // Construct accelerometers measurements
    // Build and subtract gravity acceleration
    temp_vect_eig = rotation_be.conjugate()*Vector3d(0, 0, bus_data.g);
    // Subtract corriolis acceleration
    convertEigenVector3(bus_data.velocity_angular, temp_vect_omega);
    temp_vect_eig -= temp_vect_omega.cross(vel_body_inertial);
    convertRosVector3(temp_vect_eig, temp_vect_ros);
    bus_data.acceleration_linear.x = msg.velocity.linear.x - temp_vect_ros.x; // Body frame, accelerometer reading
    bus_data.acceleration_linear.y = msg.velocity.linear.y - temp_vect_ros.y; // Body frame, accelerometer reading
    bus_data.acceleration_linear.z = msg.velocity.linear.z - temp_vect_ros.z; // Body frame, accelerometer reading
}

void SubHandlerLL::cb_environment(last_letter_msgs::Environment msg)
{
    wind_body.x() = msg.wind.x;
    wind_body.y() = msg.wind.y;
    wind_body.z() = msg.wind.z;
    bus_data.temperature_air = msg.temperature;
    bus_data.temperature_imu = msg.temperature;
    bus_data.pressure_absolute = msg.pressure*100; // Message pressure in mBar, convert to Pascal
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

double SubHandlerLL::estimate_aoa_mahony(double Va, double q)
{
    // Constant values suitable only for nominal model
    const double dt{0.0025}; // 400Hz simulation
    const double c1 = 3.82; 
    const double a0 = -0.0056;
    _aoa_estimated += (-c1/Va*_aoa_estimated + q + a0)*dt;
    return _aoa_estimated;
}

double SubHandlerLL::estimate_aoa_mine(double Va, double q)
{
    // Constant values suitable only for nominal model
    const double dt{0.0025}; // 400Hz simulation
    const double c1 = 3.82; 
    const double c2 = 0.897;
    const double c3 = 0.0551;
    const double g3 = 9.81; // approximate gravity z-component of wind-frame
    _aoa_estimated += ((-c1/Va-c2*Va)*_aoa_estimated + q + g3/Va - c3*Va)*dt;
    return _aoa_estimated;
}

double SubHandlerLL::estimate_aos(double Va)
{
    return 0;
}