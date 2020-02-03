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

#include <cstdlib>
#include <math.h>

#include <math_utils.hpp>
#include <uav_utils.hpp>
#include <prog_utils.hpp>

#include <geometry_msgs/Vector3.h>
#include <last_letter_msgs/SimPWM.h>

using Eigen::Vector3d;
using Eigen::Vector3f;

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
    referenceTrajectory_(2) = 0.0; // Set turn rate (psi_dot)
    // float defaultRefPsi = calcPsiDotDes(referenceTrajectory_);

    //Initialize internally stored simulation states to default values
    simStates_.velocity.angular.x = 0;
    simStates_.velocity.angular.y = 0;
    simStates_.velocity.angular.z = 0;
    simStates_.velocity.linear.x = referenceTrajectory_(0);
    simStates_.velocity.linear.y = 0.034;
    simStates_.velocity.linear.z = 0.0;
    tprev = ros::Time::now();
    simStates_.header.stamp = tprev;

    wind_body_.setZero(); // Initialize wind readings

    // Initialize MPC wrapper
    // Set default/initial states
    Eigen::Matrix<real_t, 5, 1> trimState = Eigen::Matrix<real_t, 5, 1>::Zero();
    trimState(0) = referenceTrajectory_(0);
    trimState(1) = 0.034;
    mpcController_.setTrimState(trimState);

    // Set default reference inputs
    refInputs_ << 0.0f, 0.0f, 0.0f, 0.0f;
    
    // Set default/initial inputs
    mpcController_.setTrimInput(refInputs_);

    // Initialize default reference states
    reference_ << referenceTrajectory_(0), referenceTrajectory_(1), referenceTrajectory_(2),
                 trimState(1), trimState(2), refInputs_;
    mpcController_.setDefaultRunningReference(reference_);
    endReference_ << trimState(0), trimState(1), 0.0f;
    mpcController_.setDefaultEndReference(endReference_);

    // Mandatory controller setup 
    mpcController_.resetController();

    // Read propeller thrust curve
	std::string uavName;
	n.getParam("uav_name", uavName);
    getDefaultParameters(uavName); // Read necessary uav parameters

    mpcController_.prepare();


    //Subscribe and advertize
    subState = n.subscribe("states", 1, &TrajectoryController::getStates, this);
	subEnvironment = n.subscribe("environment",1,&TrajectoryController::getEnvironment, this); // Environment subscriber
    subRef = n.subscribe("refTrajectory", 1, &TrajectoryController::getReference, this);
    subParam = n.subscribe("parameter_changes", 100, &TrajectoryController::getParameters, this);
    pubCmdRates = n.advertise<geometry_msgs::Vector3Stamped>("refRates", 1);
    pubCmdThrottle = n.advertise<geometry_msgs::Vector3Stamped>("throttleCmd", 1);
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
    
    // // Convert airdata triplet
    // Already doing that in state callback
    // airdata_ = getAirdata(Vector3d(simStates_.velocity.linear.x,
    //                                simStates_.velocity.linear.y,
    //                                simStates_.velocity.linear.z));

    // Restrict airspeed to non-zero to avoid divide-by-zero errors
    if (airdata_(0) < 1)
    {
        ROS_WARN("Read airspeed: %f, setting to 1m/s", airdata_(0));
        airdata_(0) = 1;
    }

    // reference vectors are already saved in the controller

    // Set the MPC reference state
    mpcController_.setReference(reference_, endReference_);

    // Solve problem with given measurements
    // Eigen::Matrix<float, 0, 1> dummyOnlineData;
    // mpcController_.update(states_, dummyOnlineData);
    // std::cout << "Passed measurements to MPC:\n" << states_  << std::endl;
    mpcController_.update(states_);

    // ROS_INFO("Trajectory MPC solver state:\n");
    // mpcController_.printSolverState();

    // mpcController_.printSolverState();
    // std::cout << "Des / Achieved psi:\n" << reference_(2) << "/\t" << calcPsiDot(states_, predictedControls_) << std::endl;

    // Write the resulting controller output
    readControls();

    // Shift the solver state by one and re-initialize
    mpcController_.shift(); // Shifting seems to destabilize solution
    acado_initializeNodesByForwardSimulation();
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

    // Extract airdata
    Vector3d vel_body_inertial = Vector3d(simStates_.velocity.linear.x,
                                          simStates_.velocity.linear.y,
                                          simStates_.velocity.linear.z); // Body frame
    Vector3d vel_body_relative = vel_body_inertial - wind_body_;
    Vector3d tempVect = getAirData(vel_body_relative);
    airdata_ = tempVect.cast<float>();

    // Isolate relevant states
    states_(0) = airdata_.x();
    states_(1) = airdata_.y();
    states_(2) = airdata_.z();

    Eigen::Quaterniond tempQuat(inpStates.pose.orientation.w,
                                inpStates.pose.orientation.x,
                                inpStates.pose.orientation.y,
                                inpStates.pose.orientation.z);
    Vector3d euler = quat2euler(tempQuat);
    states_(3) = euler.x();
    states_(4) = euler.y();

    double q = simStates_.velocity.angular.y;
    double r = simStates_.velocity.angular.z;
    double psi_dot = (q*sin(euler.x()) + r*cos(euler.x()))/cos(euler.y());
    // std::cout req/act: " <<  reference_(2) << "/\t" << psi_dot << std::endl;

    statesReceivedStatus_ = true;
}

void TrajectoryController::getEnvironment(last_letter_msgs::Environment msg)
{
    wind_body_.x() = msg.wind.x;
    wind_body_.y() = msg.wind.y;
    wind_body_.z() = msg.wind.z;
    // bus_data.temperature_air = msg.temperature;
    // bus_data.temperature_imu = msg.temperature;
    // bus_data.pressure_absolute = msg.pressure;
    // bus_data.rho = msg.density; // Air density
    // bus_data.g = msg.gravity; // Gravity acceleration
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

    // Build throttle command
    geometry_msgs::Vector3Stamped throttleCmd;
    float thrust_req = predictedControls_(3);
    float omega_req = calcOmega(thrust_req);
    // std::cout << "Calculated omega_req: " << omega_req << std::endl;
    if ((omega_req > omega_max) || (omega_req < 0))
    {
        ROS_WARN("omega_req value malformed: %g", omega_req);
        omega_req = constrain(omega_req, (float)0.0, (float)omega_max);
        // std::cout << "Actually passed omega_req: " << omega_req << std::endl;
    }
    throttleCmd.vector.x = omega_req/omega_max;
    // std::cout << "Actually passed deltat: " << omega_req/omega_max << std::endl;
    throttleCmd.header.stamp = ros::Time::now();

    // std::cout << "Mid-MPC output:\n" << predictedControls_ << "\n" << throttleCmd.vector.x << std::endl;
    pubCmdThrottle.publish(throttleCmd);
}

/**
 * @brief Calculate the angular velocity required (in rad/s) to produce require thrust
 * 
 * @param thrust Thrust requirement (in N)
 * @return the desired angular velocity
 */
float TrajectoryController::calcOmega(const float thrust)
{
    // Solve quadratic equation
    // Assumes 2nd-order thrust polynomial
    float Va = airdata_.x();
    float a = c_t_0;
    float b = c_t_1*Va/propD;
    float c = c_t_2*Va*Va/propD/propD - thrust/rho/pow(propD, 4);
    float rps = (-b + sqrt(b*b - 4*a*c))/(2*a);
    return rps*2*M_PI;
}

/**
 * @brief Callback to store the velocity commands
 * 
 * @param pRefRates The reference velocity commands Va, gamma, psi_dot
 */
void TrajectoryController::getReference(geometry_msgs::Vector3Stamped pRefTrajectory)
{
    Eigen::Vector3f tempTrajectory;
    tempTrajectory << (float)pRefTrajectory.vector.x,
                                      (float)pRefTrajectory.vector.y,
                                      (float)pRefTrajectory.vector.z;
    reference_(0) = tempTrajectory(0); // Copy over airspeed
    reference_(1) = tempTrajectory(1); // Copy over gamma   
    // float psiDotDes = calcPsiDotDes(tempTrajectory); // Calculate desired psi_dot // Not needed anymore, reference is passed as psi_dot
    reference_(2) = tempTrajectory(2);
    reference_(3) = 0.0f;
    reference_(4) = 0.0f;
    reference_.segment(kStateSize, refInputs_.size()) = refInputs_;

    endReference_(0) = tempTrajectory(0);
    endReference_(1) = tempTrajectory(1);
    // Calculate an estimate for the final roll angle
    float g = 9.81;
    float endPhi = atan(reference_(2)*reference_(0)/g);
    endReference_(2) = endPhi;
}

/**
 * @brief Caclulate turn rate of the desired trajectory
 * 
 * @param refTrajectory Vector with desired airspeed, flight path angle and turn radius
 * @return float The resulting turn radius in radians per second
 */
float calcPsiDotDes(Eigen::Vector3f refTrajectory)
{
    return refTrajectory(0)/refTrajectory(2)*cos(refTrajectory(1));
}

float calcPsiDot(Eigen::Matrix<float, kStateSize, 1> states, Eigen::Matrix<float, kInputSize, 1> inputs)
{
    return (inputs(1)*sin(states(3)) + inputs(2)*cos(states(3)))/cos(states(4));
}

/**
 * @brief Callback to store incoming parameter changes
 * 
 * @param parameter Message containing a new paramter/value pair
 */
void TrajectoryController::getParameters(last_letter_msgs::Parameter parameter)
{
    std::string paramName = parameter.name;
    float paramValue = parameter.value;
    uint paramType = parameter.type;

    if (paramType == 4) // Aerodynamic parameters
    {
        ROS_INFO("Got new parameter %s with value %g", paramName.c_str(), paramValue);

        if (paramName.compare("s")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::S, paramValue); return; }
        if (paramName.compare("b")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::b, paramValue); return; }
        if (paramName.compare("r")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c, paramValue); return; }
        if (paramName.compare("c_L_0")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_lift_0, paramValue); return; }
        if (paramName.compare("c_L_alpha")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_lift_a, paramValue); return; }
        if (paramName.compare("c_D_0")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_drag_0, paramValue); return; }
        if (paramName.compare("c_D_alpha")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_drag_a, paramValue); return; }
        if (paramName.compare("c_Y_0")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_y_0, paramValue); return; }
        if (paramName.compare("c_Y_beta")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_y_b, paramValue); return; }
    }
    else if (paramType == 3) // Inertial parameters
    {
        ROS_INFO("Got new parameter %s with value %g", paramName.c_str(), paramValue);

        if (paramName.compare("m")) { mpcController_.setOnlineDataSingle((unsigned int) Parameter::m, paramValue); return; }
    }
    else // We don't care about other paramter types
    {
        return;
    }
}

/**
 * @brief Read default uav parameters
 * 
 * @param uavName The name of the uav to read from
 */
void TrajectoryController::getDefaultParameters(std::string uavName)
{
	// Read the uav configuration
    ROS_INFO("Loading uav configuration for %s", uavName.c_str());
	ConfigsStruct_t configStruct = loadModelConfig(uavName);
    // std::cout << configStruct << std::endl;

    // Get propeller diameter
    getParameter(configStruct.prop, "motor1/propDiam", propD);

    // Grab the thrust polynomial coefficients
    YAML::Node thrustPolyConfig = filterConfig(configStruct.prop, "motor1/thrust_poly/");
    uint polyNo;
    getParameter(thrustPolyConfig, "polyNo", polyNo); // Get polynomial order for debugging purposes
    std::vector<float> coeffVect;
    getParameterList(thrustPolyConfig, "coeffs", coeffVect);
    if (coeffVect.size() != (polyNo+1)) throw runtime_error("Parameter array coeffs size did not match polyNo");
    // Single-out coefficient values
    c_t_0 = coeffVect[0];
    c_t_1 = coeffVect[1];
    c_t_2 = coeffVect[2];

    getParameter(configStruct.prop, "motor1/omega_max", omega_max);

    // Pass required parameters to MPC controller
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::S, configStruct.aero["airfoil1/s"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::b, configStruct.aero["airfoil1/b"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c, configStruct.aero["airfoil1/c"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_lift_0, configStruct.aero["airfoil1/c_L_0"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_lift_a, configStruct.aero["airfoil1/c_L_alpha"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_drag_0, configStruct.aero["airfoil1/c_D_0"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_drag_a, configStruct.aero["airfoil1/c_D_alpha"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_y_0, configStruct.aero["airfoil1/c_Y_0"].as<float>());
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::c_y_b, configStruct.aero["airfoil1/c_Y_beta"].as<float>());

    mpcController_.setOnlineDataSingle((unsigned int) Parameter::m, configStruct.inertial["m"].as<float>());
}


///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectoryControlNode");
    ros::NodeHandle n;

    ros::WallDuration(1).sleep(); //wait for other nodes to get raised
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
