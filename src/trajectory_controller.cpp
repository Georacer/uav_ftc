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
    bus_data_.velocity_angular.x = 0;
    bus_data_.velocity_angular.y = 0;
    bus_data_.velocity_angular.z = 0;
    bus_data_.airspeed = referenceTrajectory_(0);
    bus_data_.angle_of_attack = 0.034;
    bus_data_.angle_of_sideslip = 0.0;
    tprev = ros::Time::now();
    bus_data_.header.stamp = tprev;

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

    // Capture trim/default online data values
	std::string uavName;
	n.getParam("uav_name", uavName);
    getDefaultParameters(uavName);
    mpcController_.setTrimOnlineData(trimOnlineData_);

    // Mandatory controller setup 
    mpcController_.resetController();
    mpcController_.prepare();

    //Subscribe and advertize
    subState = n.subscribe("dataBus", 1, &TrajectoryController::getStates, this);
    subRef = n.subscribe("refTrajectory", 1, &TrajectoryController::getReference, this);
    subParam = n.subscribe("parameter_changes", 100, &TrajectoryController::getParameters, this);
    subFE = n.subscribe<uav_ftc::FlightEnvelopeEllipsoid>("flight_envelope", 1, &TrajectoryController::getFlightEnvelope, this);
    pubCmdRates = n.advertise<geometry_msgs::Vector3Stamped>("refRates", 1);
    pubCmdThrottle = n.advertise<geometry_msgs::Vector3Stamped>("throttleCmd", 1);
}

/**
 * @brief Destroy the Rate Controller:: Rate Controller object
 * 
 */
TrajectoryController::~TrajectoryController()
{
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
 * @param bus_data The BusData message, containing all available measurements
 */
void TrajectoryController::getStates(uav_ftc::BusData bus_data)
{
    // Copy over all aircraft state
    bus_data_ = bus_data;

    // Extract airdata
    airdata_ = Vector3f(bus_data_.airspeed, bus_data_.angle_of_attack, bus_data_.angle_of_sideslip);

    // Isolate relevant states
    states_(0) = airdata_.x();
    states_(1) = airdata_.y();
    states_(2) = airdata_.z();

    Eigen::Quaterniond tempQuat(bus_data.orientation.w,
                                bus_data.orientation.x,
                                bus_data.orientation.y,
                                bus_data.orientation.z);
    Vector3d euler = quat2euler(tempQuat);
    states_(3) = euler.x();
    states_(4) = euler.y();

    double q = bus_data_.velocity_angular.y;
    double r = bus_data_.velocity_angular.z;
    double psi_dot = (q*sin(euler.x()) + r*cos(euler.x()))/cos(euler.y());
    // std::cout req/act: " <<  reference_(2) << "/\t" << psi_dot << std::endl;

    wind_body_.x() = bus_data.wind.x;
    wind_body_.y() = bus_data.wind.y;
    wind_body_.z() = bus_data.wind.z;

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

    const int inertial_start_idx = 0;
    const int aerodynamic_start_idx = inertial_start_idx + 1; // There is 1 inertial parameter
    const int ellipsoid_start_idx = aerodynamic_start_idx + 9; // There are 9 aerodynamic parameters

    if ((paramType == 3) || (paramType == 4)) // Inertial and aerodynamic parameters parameters
    {
        ROS_INFO("Got new parameter %s with value %g", paramName.c_str(), paramValue);

        for (int i=inertial_start_idx; i < ellipsoid_start_idx; i++) // Skip Va, alpha, beta
        {
            std:string name_to_match = online_data_names[i];
            if (paramName.compare(name_to_match))
            {
                mpcController_.setOnlineDataSingle(i, paramValue);
                return;
            }
        }
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

    // Setup trimOnlineData_ vector
    const int inertial_start_idx = 0;
    const int aerodynamic_start_idx = inertial_start_idx + 1; // There is 1 inertial parameter
    const int ellipsoid_start_idx = aerodynamic_start_idx + 9; // There are 9 aerodynamic parameters

    // Read inertial parameters
    std::string param_name = online_data_names[(unsigned int) Parameter::m];
    trimOnlineData_(0) = configStruct.inertial[param_name].as<float>();

    // Read aerodynamic parameters
    for (int i = aerodynamic_start_idx; i < ellipsoid_start_idx; i++)
    {
        std:string param_name = "airfoil1/" + online_data_names[i];
        // ROS_INFO("Getting default value for %s", param_name.c_str());
        trimOnlineData_(i) = configStruct.aero[param_name].as<float>();
    }

    // Build default ellipsoid parameters
    for (int i = ellipsoid_start_idx; i < kOdSize; i++)
    {
        trimOnlineData_(i) = 0; // Set all to zero...
    }
    trimOnlineData_(kOdSize-1) = -1;
    // ... and set constant term to negative
}

void TrajectoryController::getFlightEnvelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr& fe_msg)
{
    ROS_INFO("Received new Flight Envelope");
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_A, fe_msg->el_A);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_B, fe_msg->el_B);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_C, fe_msg->el_C);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_D, fe_msg->el_E);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_E, fe_msg->el_D);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_F, fe_msg->el_F);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_G, fe_msg->el_G);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_H, fe_msg->el_H);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_I, fe_msg->el_I);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_J, fe_msg->el_J);
    // TODO: could also pass box constraints provided in fe_msg
}


///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectoryControlNode");
    ros::NodeHandle n;

    // ros::WallDuration(1).sleep(); //wait for other nodes to get raised
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
