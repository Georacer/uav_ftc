/**
 * @file rate_controller.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Defines the RateController class which controls the angular rates of the UAV. Also raises the corresponding ROS node.
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "rate_controller.hpp"

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

    // Initialize MPC wrapper

    // Set default model parameters

    // Set default/initial states
    Eigen::Matrix<real_t, 3, 1> trimState = Eigen::Matrix<real_t, 3, 1>::Zero();
    mpcController_.setTrimState(trimState);

    // Set default/initial inputs
    Eigen::Matrix<real_t, 3, 1> trimInput = Eigen::Matrix<real_t, 3, 1>::Zero();
    mpcController_.setTrimInput(trimInput);

    // Initialize default reference
    refRates_ << 0.0f, 0.0f, 0.0f;
    refInputs_ << 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf reference(refRates_.size() + refInputs_.size());
    reference << refRates_, refInputs_;
    mpcController_.setDefaultRunningReference(reference);
    mpcController_.setDefaultEndReference(refRates_);

    // Capture trim/default online data values
    trimOnlineData_(0) = states_.airspeed;
    trimOnlineData_(1) = states_.angle_of_attack;
    trimOnlineData_(2) = states_.angle_of_sideslip;
	string uavName;
	n.getParam("uav_name", uavName);
    getDefaultParameters(uavName);
    mpcController_.setTrimOnlineData(trimOnlineData_);

    // Mandatory controller setup 
    mpcController_.resetController();
    mpcController_.prepare();

    // ROS_INFO("Initial rate MPC solver state:\n");
    // mpcController_.printSolverState();

    //Subscribe and advertize
    subState = n.subscribe("dataBus", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
    subParam = n.subscribe("parameter_changes", 100, &RateController::getParameters, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3Stamped>("ctrlSurfaceCmds", 1);
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
    
    // Set the MPC reference state
    Eigen::VectorXf reference(refRates_.size() + refInputs_.size());
    reference << refRates_, refInputs_;
    mpcController_.setReference(reference, refRates_);

    // Convert airdata triplet
    Vector3d airdata = Vector3d(states_.airspeed,
                                states_.angle_of_attack,
                                states_.angle_of_sideslip);
    // Restrict airspeed to non-zero to avoid divide-by-zero errors
    if (airdata(0) < 1)
    {
        ROS_WARN_THROTTLE(1, "Read airspeed: %f, setting to 5m/s", airdata(0));
        airdata(0) = 5;
    }
    // refRates are already saved in the controller

    // Set the airdata related online data
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::Va, airdata(0));
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::alpha, airdata(1));
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::beta, airdata(2));

    // Solve problem with given measurements
    mpcController_.update(angularStates_);

    // Write the resulting controller output
    readControls();

    // std::cout << "Produced control surface cmds:\n" << predicted_controls_ << std::endl;

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
void RateController::getStates(uav_ftc::BusData bus_data)
{
    // Copy over all aircraft state
    states_ = bus_data;

    // Isolate angular velocity system measurements
    angularStates_(0) = bus_data.velocity_angular.x;
    angularStates_(1) = bus_data.velocity_angular.y;
    angularStates_(2) = bus_data.velocity_angular.z;

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
    // std::cout << "Division result cmd:\n" << predicted_controls_(0)/deltaa_max << "\n"<< predicted_controls_(1)/deltaa_max << "\n"<< predicted_controls_(2)/deltaa_max << std::endl;
    channels.vector.x = constrain(predicted_controls_(0) / deltaa_max, (float)-1.0, (float)1.0); // Pass aileron input
    channels.vector.y = constrain(predicted_controls_(1) / deltae_max, (float)-1.0, (float)1.0); // Pass elevator input
    channels.vector.z = constrain(predicted_controls_(2) / deltar_max, (float)-1.0, (float)1.0);   // Pass rudder input
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

    for (int i=3; i < kOdSize; i++) // Skip Va, alpha, beta
    {
        std:string name_to_match = online_data_names[i];
        if (paramName.compare(name_to_match))
        {
            mpcController_.setOnlineDataSingle(i, paramValue);
            return;
        }
    }
    // ROS_INFO("New parameter not included in Rate Controller MPC model");
}

/**
 * @brief Read default uav parameters and fill the trimOnlineData_ vector
 * 
 * @param uavName The name of the uav to read from
 */
void RateController::getDefaultParameters(std::string uavName)
{
	// Read the uav configuration
    ROS_INFO("Loading uav configuration for %s", uavName.c_str());
	ConfigsStruct_t configStruct = loadModelConfig(uavName);
    // std::cout << configStruct << std::endl;

    getParameter(configStruct.aero, "airfoil1/deltaa_max", deltaa_max);
    getParameter(configStruct.aero, "airfoil1/deltae_max", deltae_max);
    getParameter(configStruct.aero, "airfoil1/deltar_max", deltar_max);

    for (int i=3; i < kOdSize; i++) // Skip Va, alpha, beta
    {
        std:string param_name = "airfoil1/" + online_data_names[i];
        // ROS_INFO("Getting default value for %s", param_name.c_str());
        trimOnlineData_(i) = configStruct.aero[param_name].as<float>();
    }
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
