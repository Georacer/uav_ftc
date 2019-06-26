/**
 * @file trajectory_controller.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Defines the TrajectoryController class which controls the navigational states of the UAV. Also raises the corresponding ROS node.
 * @date 2019-06-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "trajectory_controller.hpp"

/**
 * @brief Construct a new Trajectory Controller:: Rate Controller object
 * 
 * @param n The ROS NodeHandle of the calling ROS node
 */
TrajectoryController::TrajectoryController(ros::NodeHandle n) : mpcController_(dt_, numConstraints_)
{
    // Initialize default reference
    float inf = std::numeric_limits<float>::infinity();
    referenceTrajectory_(0) = 15.0; // Set airspeed
    referenceTrajectory_(1) = 0.0; // Set flight path angle
    referenceTrajectory_(2) = inf; // Set turn radius
    float defaultRefPsi = calcPsiDot(referenceTrajectory_);

    //Initialize internally stored simulation states to default values
    simStates_.velocity.angular.x = 0;
    simStates_.velocity.angular.y = 0;
    simStates_.velocity.angular.z = 0;
    simStates_.velocity.linear.x = referenceTrajectory_(0);
    simStates_.velocity.linear.y = 0.034;
    simStates_.velocity.linear.z = 0.0;
    tprev = ros::Time::now();
    simStates_.header.stamp = tprev;

    // Initialize MPC wrapper
    // Set default/initial states
    Eigen::Matrix<real_t, 5, 1> trimState = Eigen::Matrix<real_t, 5, 1>::Zero();
    trimState(0) = referenceTrajectory_(0);
    trimState(1) = 0.034;
    mpcController_.setTrimState(trimState);

    // Set default reference inputs
    refInputs_ << 0.0f, 0.0f, 0.0f, 0.5f;
    
    // Set default/initial inputs
    mpcController_.setTrimInput(refInputs_);

    // Initialize default reference states
    reference_ << referenceTrajectory_(0), referenceTrajectory_(1), defaultRefPsi,
                 trimState(1), trimState(2), refInputs_;
    mpcController_.setDefaultRunningReference(reference_);
    endReference_ << trimState(0), trimState(1), 0.0f;
    mpcController_.setDefaultEndReference(endReference_);

    // Mandatory controller setup 
    mpcController_.resetController();
    mpcController_.prepare();

    //Subscribe and advertize
    subState = n.subscribe("states", 1, &TrajectoryController::getStates, this);
    subRef = n.subscribe("refTrajectory", 1, &TrajectoryController::getReference, this);
    pubCmdRates = n.advertise<geometry_msgs::Vector3Stamped>("refRates", 100);
    pubCmdThrottle = n.advertise<geometry_msgs::Vector3Stamped>("throttleCmd", 100);
}

/**
 * @brief Destroy the Rate Controller:: Rate Controller object
 * 
 */
TrajectoryController::~TrajectoryController()
{
    delete &mpcController_;
}

/**
 * @brief Perform one step of the MPC
 * 
 */
void TrajectoryController::step()
{
    // Don't do anything if no states measurements have been received yet
    if (!statesReceivedStatus_)
    {
        ROS_WARN("Controller tried to step without any measurements");
        return;
    }
    
    // Convert airdata triplet
    geometry_msgs::Vector3 airdataVec = getAirData(simStates_.velocity.linear);
    airdata_(0) = airdataVec.x;
    airdata_(1) = airdataVec.y;
    airdata_(2) = airdataVec.z;
    // Restrict airspeed to non-zero to avoid divide-by-zero errors
    if (airdata_(0) < 1)
    {
        ROS_WARN("Read airspeed: %f, setting to 1m/s", airdata_(0));
        airdata_(0) = 1;
    }

    // reference vectors are already saved in the controller

    // Set the MPC reference state
    mpcController_.setReference(reference_, endReference_);

    mpcController_.printSolverState();

    // Solve problem with given measurements
    Eigen::Matrix<float, 0, 1> dummyOnlineData;
    mpcController_.update(states_, dummyOnlineData);

    // Write the resulting controller output
    readControls();

    // Shift the solver state by one and re-initialize
    // mpcController_.shift(); // Shifting seems to destabilize solution
    // acado_initializeNodesByForwardSimulation();
    mpcController_.prepare();

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
void TrajectoryController::getStates(last_letter_msgs::SimStates inpStates)
{
    // Copy over all aircraft state
    simStates_ = inpStates;

    // Isolate relevant states
    geometry_msgs::Vector3 airdata = getAirData(inpStates.velocity.linear);
    states_(0) = airdata.x;
    states_(1) = airdata.y;
    states_(2) = airdata.z;
    geometry_msgs::Vector3 euler = quat2euler(inpStates.pose.orientation);
    states_(3) = euler.x;
    states_(4) = euler.y;

    statesReceivedStatus_ = true;
}

/**
 * @brief Get the optimal input calculated by the MPC
 * The result is stored in the predictedControls_ variable.
 * 
 */
void TrajectoryController::readControls()
{
    mpcController_.getInput(0, predictedControls_);
}

/**
 * @brief Convert the optimal input to PPM values and publish them
 * 
 */
void TrajectoryController::writeOutput()
{
    geometry_msgs::Vector3Stamped outputRates;
    outputRates.vector.x = predictedControls_(0);
    outputRates.vector.y = predictedControls_(1);
    outputRates.vector.z = predictedControls_(2);
    outputRates.header.stamp = ros::Time::now();
    pubCmdRates.publish(outputRates);

    geometry_msgs::Vector3Stamped throttleCmd;
    throttleCmd.vector.x = predictedControls_(3);
    throttleCmd.header.stamp = ros::Time::now();
    pubCmdThrottle.publish(throttleCmd);
}

/**
 * @brief Callback to store the angular rate reference commands
 * 
 * @param pRefRates The reference angular rates p, q, r.
 */
void TrajectoryController::getReference(geometry_msgs::Vector3Stamped pRefTrajectory)
{
    Eigen::Vector3f tempTrajectory;
    tempTrajectory << (float)pRefTrajectory.vector.x,
                                      (float)pRefTrajectory.vector.y,
                                      (float)pRefTrajectory.vector.z;
    reference_(0) = tempTrajectory(0); // Copy over airspeed
    reference_(1) = tempTrajectory(1); // Copy over gamma   
    float psiDotDes = calcPsiDot(tempTrajectory); // Calculate desired psi_dot
    reference_(2) = psiDotDes;
    reference_(3) = 0.0f;
    reference_(4) = 0.0f;
    reference_.segment(NUM_STATES, refInputs_.size()) = refInputs_;

    endReference_(0) = tempTrajectory(0);
    endReference_(1) = tempTrajectory(1);
    // Calculate an estimate for the final roll angle
    float g = 9.81;
    float endPhi = atan(psiDotDes*reference_(0)/g);
    endReference_(2) = endPhi;
}

/**
 * @brief Caclulate turn rate of the desired trajectory
 * 
 * @param refTrajectory Vector with desired airspeed, flight path angle and turn radius
 * @return float The resulting turn radius in radians per second
 */
float calcPsiDot(Eigen::Vector3f refTrajectory)
{
    return refTrajectory(0)/refTrajectory(2)*cos(refTrajectory(1));
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectoryControlNode");
    ros::NodeHandle n;

    ros::WallDuration(3).sleep(); //wait for other nodes to get raised
    double ctrlRate;
    if (!ros::param::get("ctrlTrajectoryRate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find ctrlTrajectoryRate parameter");
        ros::shutdown();
    }
    ros::Rate spinner(ctrlRate);

    TrajectoryController rate_ctrl(n);
    spinner.sleep();
    ROS_INFO("trajectoryControlNode up");

    while (ros::ok())
    {
        ros::spinOnce();
        rate_ctrl.step();
        spinner.sleep();
    }

    return 0;
}
