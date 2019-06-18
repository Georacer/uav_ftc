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

/**
 * @brief Construct a new Rate Controller:: Rate Controller object
 * 
 * @param n The ROS NodeHandle of the calling ROS node
 */
RateController::RateController(ros::NodeHandle n) : mpcController_()
{
    //Initialize states
    states_.velocity.angular.x = 0;
    states_.velocity.angular.y = 0;
    states_.velocity.angular.z = 0;
    states_.velocity.linear.x = 15.0;
    states_.velocity.linear.x = 0.5;
    states_.velocity.linear.x = 0.0;
    tprev = ros::Time::now();
    states_.header.stamp = tprev;

    // Initialize airdata container
    airdata_ << 15.0f, 0.03f, 0.0f; // Values allow a normal-ish response, avoiding divide-by-zero errors
    mpcController_.setOnlineData(airdata_);
    // Initialize system state
    angularStates_ << 0.0f, 0.0f, 0.0f;
    mpcController_.setInitialState(angularStates_);
    // Initialize reference
    refRates_ << 0.0f, 0.0f, 0.0f;
    mpcController_.setReferencePose(refRates_);

    //Subscribe and advertize
    subState = n.subscribe("states", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
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
    // Convert airdata triplet
    geometry_msgs::Vector3 airdataVec = getAirData(states_.velocity.linear);
    airdata_(0) = airdataVec.x;
    airdata_(1) = airdataVec.y;
    airdata_(2) = airdataVec.z;
    // Restrict airspeed to non-zero to avoid divide-by-zero errors
    if (airdata_(0) < 1)
    {
        airdata_(0) = 1;
    }

    // refRates are already saved in the controller

    // Set the MPC reference state
    mpcController_.setReferencePose(refRates_);

    // Solve problem with given measurements
    mpcController_.update(angularStates_, airdata_);

    // Write the resulting controller output
    readControls();

    // Shift the solver state by one and re-initialize
    mpcController_.shift();
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

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controlNode");
    ros::NodeHandle n;

    ros::WallDuration(3).sleep(); //wait for other nodes to get raised
    double ctrlRate;
    if (!ros::param::get("ctrlRate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find ctrlRate parameter");
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
