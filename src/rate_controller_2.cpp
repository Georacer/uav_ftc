/**
 * @file rate_controller.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Defines the RateController class which controls the angular rates of the UAV. Also raises the corresponding ROS node.
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "rate_controller_2.hpp"

#include <cstdlib>
#include <math.h>

#include <math_utils.hpp>
#include <uav_utils.hpp>
#include <prog_utils.hpp>

#include <geometry_msgs/Vector3.h>
#include <last_letter_msgs/SimPWM.h>

using Eigen::Vector3d;

/**
 * @brief Construct a new Rate Controller:: Rate Controller object
 * 
 * @param n The ROS NodeHandle of the calling ROS node
 */
RateController::RateController(ros::NodeHandle n) : mpcController_(dt_, numConstraints_)
{
    //Initialize states
    states_.velocity.angular.x = 0;
    states_.velocity.angular.y = 0;
    states_.velocity.angular.z = 0;
    states_.velocity.linear.x = 15.0;
    states_.velocity.linear.y = 0.034;
    states_.velocity.linear.z = 0.0;
    tprev = ros::Time::now();
    states_.header.stamp = tprev;

    // Initialize MPC wrapper
    // Set default model parameters
	string uavName;
	n.getParam("uav_name", uavName);
    // Set default/initial states
    Eigen::Matrix<real_t, 3, 1> trimState = Eigen::Matrix<real_t, 3, 1>::Zero();
    mpcController_.setTrimState(trimState);
    // Set default/initial inputs
    Eigen::Matrix<real_t, 3, 1> trimInput = Eigen::Matrix<real_t, 3, 1>::Zero();
    mpcController_.setTrimInput(trimInput);
    // Set default/initial online data
	Eigen::Quaterniond tempQuat(states_.pose.orientation.w, states_.pose.orientation.x, states_.pose.orientation.y, states_.pose.orientation.z);
    Eigen::Vector3d tempVect(quat2euler(tempQuat));
    Eigen::Matrix<real_t, 5, 1> trimOnlineData = (Eigen::Matrix<real_t, 5, 1>() << 
        states_.velocity.linear.x,
        states_.velocity.linear.y,
        states_.velocity.linear.z,
        tempVect.x(), // Pass phi
        tempVect.y() // Pass theta
        ).finished();
    mpcController_.setTrimOnlineData(trimOnlineData);
    // Initialize default reference
    refRates_ << 0.0f, 0.0f, 0.0f;
    refInputs_ << 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf reference(refRates_.size() + refInputs_.size());
    reference << refRates_, refInputs_;
    mpcController_.setDefaultRunningReference(reference);
    mpcController_.setDefaultEndReference(refRates_);
    // Mandatory controller setup 
    mpcController_.resetController();
    getDefaultParameters(uavName); // Restore aerodynamic parameters online data AFTER controller reset
    mpcController_.prepare();

    ROS_INFO("Initial rate MPC solver state:\n");
    mpcController_.printSolverState();

    //Subscribe and advertize
    subState = n.subscribe("states", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
    subParam = n.subscribe("parameter_changes", 100, &RateController::getParameters, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3Stamped>("ctrlSurfaceCmds", 100);
}

/**
 * @brief Destroy the Rate Controller:: Rate Controller object
 * 
 */
RateController::~RateController()
{
    delete &mpcController_;
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
    
    // Convert airdata triplet
    Vector3d airdata = getAirData(Vector3d(states_.velocity.linear.x,
                                            states_.velocity.linear.y,
                                            states_.velocity.linear.z));
    // Restrict airspeed to non-zero to avoid divide-by-zero errors
    if (airdata(0) < 1)
    {
        // ROS_WARN("Read airspeed: %f, setting to 1m/s", airdata_(0));
        airdata(0) = 1;
    }
    // refRates are already saved in the controller

    // Set the MPC reference state
    Eigen::VectorXf reference(refRates_.size() + refInputs_.size());
    reference << refRates_, refInputs_;
    mpcController_.setReference(reference, refRates_);

    // Set the airdata related online data
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::Va, airdata(0));
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::alpha, airdata(1));
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::beta, airdata(2));

    // Solve problem with given measurements
    mpcController_.update(angularStates_);

    // Write the resulting controller output
    readControls();

    // Shift the solver state by one and re-initialize
    // mpcController_.shift();
    mpcController_.prepare();

    // std::cout << "Got airdata: " << airdata_ << std::endl;
    // std::cout << "Got reference: " << refRates_ << std::endl;
    // std::cout << "Got measurements: " << angularStates_ << std::endl;
    // std::cout << "Made input: " << predicted_controls_ << std::endl;

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
void RateController::getStates(last_letter_msgs::SimStates inpStates)
{
    // Copy over all aircraft state
    states_ = inpStates;

    // Isolate angular velocity system measurements
    angularStates_(0) = inpStates.velocity.angular.x;
    angularStates_(1) = inpStates.velocity.angular.y;
    angularStates_(2) = inpStates.velocity.angular.z;

    statesReceivedStatus_ = true;
}

/**
 * @brief Get the optimal input calculated by the MPC
 * The result is stored in the predicted_controls_ variable.
 * 
 */
void RateController::readControls()
{
    mpcController_.getInput(0, predicted_controls_);
}

/**
 * @brief Convert the optimal input to PPM values and publish them
 * 
 */
void RateController::writeOutput()
{
    geometry_msgs::Vector3Stamped channels;
    // MPC controls are in radians, must convert them to -1,1 range
    double deltaa_max, deltae_max, deltar_max;
    ros::param::getCached("airfoil1/deltaa_max_nominal", deltaa_max);
    ros::param::getCached("airfoil1/deltae_max_nominal", deltae_max);
    ros::param::getCached("airfoil1/deltar_max_nominal", deltar_max);
    channels.vector.x = constrain(predicted_controls_(0) / deltaa_max, -1.0, 1.0); // Pass aileron input
    channels.vector.y = constrain(predicted_controls_(1) / deltae_max, -1.0, 1.0); // Pass elevator input
    channels.vector.z = constrain(predicted_controls_(2) / deltar_max, -1.0, 1.0);   // Pass rudder input
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

/**
 * @brief Callback to store incoming parameter changes
 * 
 * @param parameter Message containing a new paramter/value pair
 */
void RateController::getParameters(last_letter_msgs::Parameter parameter)
{
    std::string paramName = parameter.name;
    float paramValue = parameter.value;
    uint paramType = parameter.type;

    if (paramType != 4) return; // We only care about aerodynamic parameters

    ROS_INFO("Got new parameter %s with value %g", paramName.c_str(), paramValue);

    if (paramName.compare("c_l_0")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_0, paramValue); return; }
    if (paramName.compare("c_l_pn")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_p, paramValue); return; }
    if (paramName.compare("c_l_beta")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_b, paramValue); return; }
    if (paramName.compare("c_l_rn")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_r, paramValue); return; }
    if (paramName.compare("c_l_deltae")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_de, paramValue); return; }
    if (paramName.compare("c_l_deltar")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_dr, paramValue); return; }

    if (paramName.compare("c_m_0")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_0, paramValue); return; }
    if (paramName.compare("c_m_alpha")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_a, paramValue); return; }
    if (paramName.compare("c_m_qn")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_q, paramValue); return; }
    if (paramName.compare("c_m_deltae")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_de, paramValue); return; }

    if (paramName.compare("c_n_0")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_0, paramValue); return; }
    if (paramName.compare("c_n_beta")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_b, paramValue); return; }
    if (paramName.compare("c_n_pn")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_p, paramValue); return; }
    if (paramName.compare("c_n_rn")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_r, paramValue); return; }
    if (paramName.compare("c_n_deltaa")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_da, paramValue); return; }
    if (paramName.compare("c_n_deltar")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_dr, paramValue); return; }
}

/**
 * @brief Read default uav parameters and pass them to the MPC
 * 
 * @param uavName The name of the uav to read from
 */
void RateController::getDefaultParameters(std::string uavName)
{
	// Read the uav configuration
    ROS_INFO("Loading uav configuration for %s", uavName.c_str());
	ConfigsStruct_t configStruct = loadModelConfig(uavName);
    // std::cout << configStruct << std::endl;

    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_0, configStruct.aero["airfoil1/c_l_0"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_p, configStruct.aero["airfoil1/c_l_pn"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_b, configStruct.aero["airfoil1/c_l_beta"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_r, configStruct.aero["airfoil1/c_l_rn"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_de, configStruct.aero["airfoil1/c_l_deltaa"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_l_dr, configStruct.aero["airfoil1/c_l_deltar"].as<float>());

    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_0, configStruct.aero["airfoil1/c_m_0"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_a, configStruct.aero["airfoil1/c_m_alpha"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_q, configStruct.aero["airfoil1/c_m_qn"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_m_de, configStruct.aero["airfoil1/c_m_deltae"].as<float>());

    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_0, configStruct.aero["airfoil1/c_n_0"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_b, configStruct.aero["airfoil1/c_n_beta"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_p, configStruct.aero["airfoil1/c_n_pn"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_r, configStruct.aero["airfoil1/c_n_rn"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_da, configStruct.aero["airfoil1/c_n_deltaa"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_n_dr, configStruct.aero["airfoil1/c_n_deltar"].as<float>());
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controlNode");
    ros::NodeHandle n;

    // ros::WallDuration(0).sleep(); //wait for other nodes to get raised
    double ctrlRate;
    if (!ros::param::get("ctrlRateRate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find ctrlRateRate parameter");
        ros::shutdown();
    }
    ros::Rate spinner(ctrlRate);

    RateController rate_ctrl(n);
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
