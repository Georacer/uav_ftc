/**
 * @file rate_controller.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Defines the RateController class which controls the angular rates of the UAV. Also raises the corresponding ROS node.
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "rate_controller_pid.hpp"

#include <cstdlib>
#include <math.h>

#include <math_utils.hpp>
#include <prog_utils.hpp>

#include <geometry_msgs/Vector3.h>
#include <last_letter_msgs/SimPWM.h>

using Eigen::Vector3d;

/**
 * @brief Construct a new Rate Controller:: Rate Controller object
 * 
 * @param n The ROS NodeHandle of the calling ROS node
 */
RateController::RateController(ros::NodeHandle n, ros::NodeHandle pnh)
{
    //Initialize states
    states_.velocity_angular.x = 0;
    states_.velocity_angular.y = 0;
    states_.velocity_angular.z = 0;
    states_.airspeed = 15.0;
    states_.angle_of_attack = 0.034;
    states_.angle_of_sideslip = 0.0;
    states_.orientation.x = 0.0;
    states_.orientation.y = 0.0;
    states_.orientation.z = 0.0;
    states_.orientation.w = 1.0;
    tprev = ros::Time::now();
    states_.header.stamp = tprev;

    // Setup data filters
    // Angular Rates
    filt_p_.setup(f_sample_, f_cut_);
    filt_q_.setup(f_sample_, f_cut_);
    filt_r_.setup(f_sample_, f_cut_);

    // Initialize PID controllers
    double rate;
    pnh.getParam("rate", rate);
    pnh.getParam("kp_p", pid_p_kp_);
    pnh.getParam("ki_p", pid_p_ki_);
    pnh.getParam("kd_p", pid_p_kd_);
    pnh.getParam("kp_q", pid_q_kp_);
    pnh.getParam("ki_q", pid_q_ki_);
    pnh.getParam("kd_q", pid_q_kd_);
    pnh.getParam("kp_r", pid_r_kp_);
    pnh.getParam("ki_r", pid_r_ki_);
    pnh.getParam("kd_r", pid_r_kd_);
    pid_p_ = new PID(pid_p_kp_, pid_p_ki_, pid_p_kd_, 1.0, -1.0, 0, 1.0/rate, 0.1);
    ROS_INFO("Raised p_pid with gains: %f, %f, %f", pid_p_kp_, pid_p_ki_, pid_p_kd_);
    pid_q_ = new PID(pid_q_kp_, pid_q_ki_, pid_q_kd_, 1.0, -1.0, 0, 1.0/rate, 0.1);
    ROS_INFO("Raised q_pid with gains: %f, %f, %f", pid_q_kp_, pid_q_ki_, pid_q_kd_);
    pid_r_ = new PID(pid_r_kp_, pid_r_ki_, pid_r_kd_, 1.0, -1.0, 0, 1.0/rate, 0.1);
    ROS_INFO("Raised r_pid with gains: %f, %f, %f", pid_r_kp_, pid_r_ki_, pid_r_kd_);

    //Subscribe and advertize
    subState = n.subscribe("dataBus", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3Stamped>("ctrlSurfaceCmds", 1);
}

/**
 * @brief Destroy the Rate Controller:: Rate Controller object
 * 
 */
RateController::~RateController()
{
    delete pid_p_;
    delete pid_q_;
    delete pid_r_;
}

/**
 * @brief Perform one step of the MPC
 * 
 */
void RateController::step()
{
    // Don't do anything if no states measurements have been received yet
    if (!statesReceivedStatus_)
    {
        ROS_WARN("Controller tried to step without any measurements");
        return;
    }

    // Filter state data
    double p = filt_p_.filter(states_.velocity_angular.x);
    double q = filt_q_.filter(states_.velocity_angular.y);
    double r = filt_r_.filter(states_.velocity_angular.z);

    // Calculate errors
    double p_err = refRates_(0) - p;
    double q_err = refRates_(1) - q;
    double r_err = refRates_(2) - r;

    // Generate control inputs
    deltaa_ = pid_p_->step(p_err);
    deltae_ = pid_q_->step(q_err);
    deltar_ = pid_r_->step(r_err);

    // Write the resulting controller output
    readControls();

    // std::cout << "Got rates: " << p << q << r << std::endl;
    // std::cout << "Got reference: " << refRates_ << std::endl;
    // std::cout << "Made input: " << controls_ << std::endl;

    // Fill control commands and publish them
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
void RateController::getStates(uav_ftc::BusData bus_data)
{
    // Copy over all aircraft state
    states_ = bus_data;

    statesReceivedStatus_ = true;
}

/**
 * @brief Get the optimal input calculated by the MPC
 * The result is stored in the controls_ variable.
 * 
 */
void RateController::readControls()
{
    controls_ << deltaa_, deltae_, deltar_;
}

/**
 * @brief Convert the optimal input to PPM values and publish them
 * 
 */
void RateController::writeOutput()
{
    geometry_msgs::Vector3Stamped channels;
    // MPC controls are in radians, must convert them to -1,1 range
    // std::cout << "Division result cmd:\n" << controls_(0)/deltaa_max << "\n"<< controls_(1)/deltaa_max << "\n"<< controls_(2)/deltaa_max << std::endl;
    channels.vector.x = constrain(controls_(0), -1.0, 1.0); // Pass aileron input
    channels.vector.y = constrain(controls_(1), -1.0, 1.0); // Pass elevator input
    channels.vector.z = constrain(controls_(2), -1.0, 1.0);   // Pass rudder input
    // std::cout << "Equivalent model cmd:\n" << channels.vector.x << "\n"<< channels.vector.y << "\n"<< channels.vector.z << std::endl;
    channels.header.stamp = ros::Time::now();
    pubCtrl.publish(channels);
}

/**
 * @brief Callback to store the angular rate reference commands
 * 
 * @param pRefRates The reference angular rates p, q, r.
 */
void RateController::getReference(geometry_msgs::Vector3Stamped pRefRates)
{
    refRates_(0) = pRefRates.vector.x;
    refRates_(1) = pRefRates.vector.y;
    refRates_(2) = pRefRates.vector.z;
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

    RateController rate_ctrl(n, pnh);
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
