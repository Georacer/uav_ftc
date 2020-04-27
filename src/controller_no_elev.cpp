/**
 * @file controller_no_ail.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Implements trajectory control using only a 3-channel model (rudder, elevator, throttle)
 * @date 2020-04-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "controller_no_elev.hpp"

#include <cstdlib>
#include <math.h>

#include <math_utils.hpp>
#include <uav_utils.hpp>
#include <prog_utils.hpp>

#include <geometry_msgs/Vector3.h>
#include <last_letter_msgs/SimPWM.h>

using Eigen::Vector3d;

/**
 * @brief Construct a new Controller object
 * 
 * @param n The ROS NodeHandle of the calling ROS node
 */
Controller::Controller(ros::NodeHandle n, ros::NodeHandle pnh)
    : fe_ellipsoid_(default_fe_coeffs_)  // Assign very large coeffs on squares to force always valid
{
    //Initialize states
    states_.velocity_angular.x = 0;
    states_.velocity_angular.y = 0;
    states_.velocity_angular.z = 0;
    states_.airspeed = 15.0;
    states_.angle_of_attack = 0.0;
    states_.angle_of_sideslip = 0.0;
    states_.orientation.x = 0.0;
    states_.orientation.y = 0.0;
    states_.orientation.z = 0.0;
    states_.orientation.w = 1.0;
    tprev = ros::Time::now();
    states_.header.stamp = tprev;

    // Initialize default reference
    reference_ << 15, 0, 0;

    // Read parameters from server
	std::string uavName;
	n.getParam("uav_name", uavName);
    getDefaultParameters(uavName);

    // Mandatory controller setup 
    double rate;
    pnh.getParam("rate", rate);
    double kp_beta, ki_beta, kd_beta;
    pnh.getParam("kp_beta", kp_beta);
    pnh.getParam("ki_beta", ki_beta);
    pnh.getParam("kd_beta", kd_beta);
    double kp_gamma, ki_gamma, kd_gamma;
    pnh.getParam("kp_gamma", kp_gamma);
    pnh.getParam("ki_gamma", ki_gamma);
    pnh.getParam("kd_gamma", kd_gamma);
    double kp_psi_dot, ki_psi_dot, kd_psi_dot;
    pnh.getParam("kp_psi_dot", kp_psi_dot);
    pnh.getParam("ki_psi_dot", ki_psi_dot);
    pnh.getParam("kd_psi_dot", kd_psi_dot);
    double kp_phi, ki_phi, kd_phi;
    pnh.getParam("kp_phi", kp_phi);
    pnh.getParam("ki_phi", ki_phi);
    pnh.getParam("kd_phi", kd_phi);
    double kp_p, ki_p, kd_p;
    pnh.getParam("kp_p", kp_p);
    pnh.getParam("ki_p", ki_p);
    pnh.getParam("kd_p", kd_p);
    double trim_deltaF = 0.6*9.81;
    pid_beta_ = new PID(kp_beta, ki_beta, kd_beta, 1.0, -1.0, 0.0, 1.0/rate, 0.1);
    pid_gamma_ = new PID(kp_gamma, ki_gamma, kd_gamma, 1.2*9.81, 0.0, trim_deltaF, 1.0/rate, 0.1);
    pid_psi_dot_ = new PID(kp_psi_dot, ki_psi_dot, kd_psi_dot, 60, -60.0, 0.0, 1.0/rate, 0.1);
    pid_phi_ = new PID(kp_phi, ki_phi, kd_phi, 1.0, -1.0, 0.0, 1.0/rate, 0.1);
    pid_p_ = new PID(kp_p, ki_p, kd_p, 1.0, -1.0, 0.0, 1.0/rate, 0.1);

    //Subscribe and advertize
    subState = n.subscribe("dataBus", 1, &Controller::getStates, this);
    subRef = n.subscribe("refTrajectory", 1, &Controller::getReference, this);
    subFE = n.subscribe<uav_ftc::FlightEnvelopeEllipsoid>("flight_envelope", 1, &Controller::getFlightEnvelope, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3Stamped>("ctrlSurfaceCmds", 1);
    pubCmdThrottle = n.advertise<geometry_msgs::Vector3Stamped>("throttleCmd", 1);
}

/**
 * @brief Destroy the Controller::Controller object
 * 
 */
Controller::~Controller()
{
    delete pid_beta_;
    delete pid_gamma_;
    delete pid_psi_dot_;
    delete pid_phi_;
    delete pid_p_;
}

/**
 * @brief Perform one step of the controller
 * 
 */
void Controller::step()
{
    // Don't do anything if no states measurements have been received yet
    if (!statesReceivedStatus_)
    {
        ROS_WARN("Controller tried to step without any measurements");
        return;
    }

    // Read state information
    Eigen::Quaterniond tempQuat(states_.orientation.w,
                                states_.orientation.x,
                                states_.orientation.y,
                                states_.orientation.z);
    Vector3d euler = quat2euler(tempQuat);
    double p = states_.velocity_angular.x;
    double q = states_.velocity_angular.y;
    double r = states_.velocity_angular.z;
    double va = states_.airspeed;
    double alpha = states_.angle_of_attack;
    double beta = states_.angle_of_sideslip;
    double sa = sin(alpha);
    double ca = cos(alpha);
    double sb = sin(beta);
    double cb = cos(beta);
    double sph = sin(euler.x());
    double cph = cos(euler.x());
    double sth = sin(euler.y());
    double cth = cos(euler.y());
    // Calculate derivative quantities
    double psi_dot = (q*sin(euler.x()) + r*cos(euler.x()))/cos(euler.y());
    double gamma = asin((ca*cb)*sth - (sph*sb + cph*sa*cb)*cth); // Stevens-Lewis 2003 3.4-2

    // Calculate errors
    double beta_err = beta;
    double gamma_err = reference_(1) - gamma;
    double psi_dot_err = reference_(2) - psi_dot;
    double phi_err = phi_des_ - euler.x()*180/M_PI;
    double p_err = p_des_ - p;

    // Generate control inputs
    deltar_ = pid_beta_->step(beta_err);
    deltaF_ = pid_gamma_->step(gamma_err);
    phi_des_ = pid_psi_dot_->step(psi_dot_err);
    p_des_ = pid_phi_->step(phi_err);
    deltaa_ = pid_p_->step(p_err);
    
    // Write the resulting controller output
    readControls();

    // Fill control commands and publish them
    // Will constrain outputs to [-1, 1]
    writeOutput();
}

///////////
//Utilities
///////////

/**
 * @brief Callback function to store the UAV states
 * Used to capture the angular rate measurements
 * 
 * @param inpStates The whole UAV state
 */
void Controller::getStates(uav_ftc::BusData bus_data)
{
    // Copy over all aircraft state
    states_ = bus_data;

    statesReceivedStatus_ = true;
}

/**
 * @brief Get the optimal input calculated by the MPC
 * The result is stored in the predicted_controls_ variable.
 * 
 */
void Controller::readControls()
{
    controls_ << deltaa_, 0, deltaF_, deltar_;
}

/**
 * @brief Calculate the angular velocity required (in rad/s) to produce require thrust
 * 
 * @param thrust Thrust requirement (in N)
 * @return the desired angular velocity
 */
double Controller::calcOmega(const double thrust)
{
    // Solve quadratic equation
    // Assumes 2nd-order thrust polynomial
    double Va = states_.airspeed;
    double a = c_t_0;
    double b = c_t_1*Va/propD;
    double c = c_t_2*Va*Va/propD/propD - thrust/rho/pow(propD, 4);
    double rps = (-b + sqrt(b*b - 4*a*c))/(2*a);
    return rps*2*M_PI;
}

/**
 * @brief Convert the optimal input to PPM values and publish them
 * 
 */
void Controller::writeOutput()
{
    geometry_msgs::Vector3Stamped channels;
    // MPC controls are in radians, must convert them to -1,1 range
    // std::cout << "Division result cmd:\n" << predicted_controls_(0)/deltaa_max << "\n"<< predicted_controls_(1)/deltaa_max << "\n"<< predicted_controls_(2)/deltaa_max << std::endl;
    channels.vector.x = constrain(controls_(0), -1.0, 1.0); // Pass aileron input
    channels.vector.y = constrain(controls_(1), -1.0, 1.0); // Pass elevator input
    channels.vector.z = constrain(controls_(3), -1.0, 1.0);   // Pass rudder input
    // std::cout << "Equivalent model cmd:\n" << channels.vector.x << "\n"<< channels.vector.y << "\n"<< channels.vector.z << std::endl;
    channels.header.stamp = ros::Time::now();
    pubCtrl.publish(channels);

    double thrust_req = controls_(2);
    double omega_req = calcOmega(thrust_req);
    if ((omega_req > omega_max) || (omega_req < 0))
    {
        ROS_WARN("omega_req value malformed: %g", omega_req);
        omega_req = constrain(omega_req, 0.0, omega_max);
    }
    geometry_msgs::Vector3Stamped throttleCmd;
    throttleCmd.vector.x = omega_req/omega_max;
    throttleCmd.header.stamp = ros::Time::now();
    pubCmdThrottle.publish(throttleCmd);
}

/**
 * @brief Callback to store the velocity commands
 * 
 * @param pRefRates The reference velocity commands Va, gamma, psi_dot
 */
void Controller::getReference(geometry_msgs::Vector3Stamped pRefTrajectory)
{
    Vector3d setpoint, projected_setpoint;
    setpoint << pRefTrajectory.vector.x,
                pRefTrajectory.vector.y,
                pRefTrajectory.vector.z;

    // Evaluate setpoint
    double point_value = fe_ellipsoid_.evaluate(setpoint.x(), setpoint.y(), setpoint.z());
    if (point_value>0)
    {
        // Project setpoint onto ellipsoid
        projected_setpoint = fe_ellipsoid_.project_point(setpoint);
        // std::cout << "*** Projected point " << setpoint.transpose() << " onto " << projected_setpoint.transpose() << std::endl;
    }
    else
    {
        projected_setpoint = setpoint;
        // std::cout << "Reference " << projected_setpoint << " already valid" << std::endl;
    }
    reference_ = projected_setpoint;
}

/**
 * @brief Read default uav parameters and fill the trimOnlineData_ vector
 * 
 * @param uavName The name of the uav to read from
 */
void Controller::getDefaultParameters(std::string uavName)
{
	// Read the uav configuration
    ROS_INFO("Loading uav configuration for %s", uavName.c_str());
	ConfigsStruct_t configStruct = loadModelConfig(uavName);

    // Get propeller diameter
    getParameter(configStruct.prop, "motor1/propDiam", propD);

    // Grab the thrust polynomial coefficients
    YAML::Node thrustPolyConfig = filterConfig(configStruct.prop, "motor1/thrust_poly/");
    uint polyNo;
    getParameter(thrustPolyConfig, "polyNo", polyNo); // Get polynomial order for debugging purposes
    std::vector<double> coeffVect;
    getParameterList(thrustPolyConfig, "coeffs", coeffVect);
    if (coeffVect.size() != (polyNo+1)) throw runtime_error("Parameter array coeffs size did not match polyNo");
    // Single-out coefficient values
    c_t_0 = coeffVect[0];
    c_t_1 = coeffVect[1];
    c_t_2 = coeffVect[2];

    getParameter(configStruct.prop, "motor1/omega_max", omega_max);
}

void Controller::getFlightEnvelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr& fe_msg)
{
    // TODO: could also pass box constraints provided in fe_msg
    // Update the ellipsoid coefficients for debugging purposes
    Ellipsoid3DCoefficients_t coeffs = {
        fe_msg->el_A,
        fe_msg->el_B,
        fe_msg->el_C,
        fe_msg->el_D,
        fe_msg->el_E,
        fe_msg->el_F,
        fe_msg->el_G,
        fe_msg->el_H,
        fe_msg->el_I,
        fe_msg->el_J,
    };
    fe_ellipsoid_.update_coeffs(coeffs);
}

///////////////
//Main function
///////////////
// Node parameters
// uav_name: name of the UAV, to access last_letter_models folder
// ~rate: Rate at which to publish actuator control inputs
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controlNode");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    double ctrlRate;
    if (!pnh.getParam("rate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find -rate- parameter");
        ros::shutdown();
    }
    ros::Rate spinner(ctrlRate);

    Controller rate_ctrl(n, pnh);
    spinner.sleep();
    ROS_INFO("ControlNode up");

    while (ros::ok())
    {
        ros::spinOnce();
        rate_ctrl.step();
        spinner.sleep();
    }

    return 0;
}
