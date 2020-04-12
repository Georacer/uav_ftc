/**
 * @file trajectory_controller.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Defines the TrajectoryController class which controls the navigational states of the UAV. Also raises the corresponding ROS node.
 * @date 2019-06-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "trajectory_controller_no_va.hpp"

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
 * @param pnh The private ROS NodeHandle of the calling ROS node
 */
TrajectoryController::TrajectoryController(ros::NodeHandle n, ros::NodeHandle pnh) 
    : mpcController_(dt_, numConstraints_)
    , fe_ellipsoid_(default_fe_coeffs_)  // Assign very large coeffs on squares to force always valid
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

    // Setup data filters
    // Airdata
    filt_va_.setup(f_sample_, f_cut_);
    filt_alpha_.setup(f_sample_, f_cut_);
    filt_beta_.setup(f_sample_, f_cut_);
    // Angular Rates
    filt_q_.setup(f_sample_, f_cut_);
    filt_r_.setup(f_sample_, f_cut_);

    // Initialize MPC wrapper
    // Set default/initial states
    Eigen::Matrix<real_t, kStateSize, 1> trimState = Eigen::Matrix<real_t, kStateSize, 1>::Zero();
    mpcController_.setTrimState(trimState);

    // Set default reference inputs
    refInputs_ << 0.0f, 0.0f, 0.0f;
    
    // Set default/initial inputs
    mpcController_.setTrimInput(refInputs_);

    // Initialize default reference states
    reference_ << referenceTrajectory_(1),
                  referenceTrajectory_(2),
                  trimState(0),
                  trimState(1),
                  refInputs_,
                  0.0f;
    mpcController_.setDefaultRunningReference(reference_);
    endReference_ << trimState(0),
                     0.0f;
    mpcController_.setDefaultEndReference(endReference_);

    // Initialize default variable bounds
    getDefaultBounds(pnh);

    // Initialize default variable bounds
    getDefaultWeights(pnh);
    // Capture trim/default online data values
	std::string uavName;
	n.getParam("uav_name", uavName);
    getDefaultParameters(uavName);
    mpcController_.setTrimOnlineData(trimOnlineData_);

    // Mandatory controller setup 
    mpcController_.resetController();
    mpcController_.prepare();

    // Allocate new airspeed pid
    double kp_va, ki_va, kd_va;
    pnh.getParam("kp_va", kp_va);
    pnh.getParam("ki_va", ki_va);
    pnh.getParam("kd_va", kd_va);
    double rate;
    pnh.getParam("rate", rate); //frame rate in Hz
    pid_va_ = new PID(kp_va, ki_va, kd_va, 1.0, 0.0, 0.5, 1.0/rate, 0.1);

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
    delete pid_va_;
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
    if (airdata_(0) < 10)
    {
        ROS_WARN_THROTTLE(1, "Read airspeed: %f, setting to 10m/s", airdata_(0));
        airdata_(0) = 10;
    }

    // reference vectors are already saved in the controller

    // Set the MPC reference state
    mpcController_.setReference(reference_, endReference_);

    // Update online data variables
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::Va, airdata_(0));
    double thrust_estimate = estimateThrust(airdata_(0), deltat_); // Estimate thrust using previous readings and outputs
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::Fprop, thrust_estimate);

    // Solve problem with given measurements
    // Eigen::Matrix<float, 0, 1> dummyOnlineData;
    // mpcController_.update(states_, dummyOnlineData);
    // std::cout << "Passed measurements to MPC:\n" << states_  << std::endl;
    mpcController_.update(states_);

    // ROS_INFO("Trajectory MPC solver state:\n");
    // mpcController_.printSolverState();
    // std::cout << "Des / Achieved psi:\n" << reference_(2) << "/\t" << calcPsiDot(states_, predictedControls_) << std::endl;

    // Write the resulting MPC controller output
    readControls();

    // Step the airspeed PID
    stepAirspeedController();

    // Shift the solver state by one and re-initialize
    mpcController_.shift(); // Shifting seems to destabilize solution
    acado_initializeNodesByForwardSimulation();
    mpcController_.prepare();

    // Fill control commands and publish them
    writeOutput();
}

// Calculate an airspeed PID step
void TrajectoryController::stepAirspeedController()
{
    double va_err = referenceTrajectory_(0) - airdata_(0);
    deltat_ = pid_va_->step(va_err);
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
    airdata_ = Vector3f(
        filt_va_.filter(bus_data_.airspeed),
        filt_alpha_.filter(bus_data_.angle_of_attack),
        filt_beta_.filter(bus_data_.angle_of_sideslip)
    );

    // Isolate relevant states
    states_(0) = airdata_.y();
    states_(1) = airdata_.z();

    Eigen::Quaterniond tempQuat(bus_data.orientation.w,
                                bus_data.orientation.x,
                                bus_data.orientation.y,
                                bus_data.orientation.z);
    Vector3d euler = quat2euler(tempQuat);
    states_(2) = euler.x();
    states_(3) = euler.y();

    double q = filt_q_.filter(bus_data_.velocity_angular.y);
    double r = filt_r_.filter(bus_data_.velocity_angular.z);
    double psi_dot = (q*sin(euler.x()) + r*cos(euler.x()))/cos(euler.y());
    // std::cout req/act: " <<  reference_(2) << "/\t" << psi_dot << std::endl;
    // Calculate gamma
    double sa = sin(states_(1));
    double ca = cos(states_(1));
    double sb = sin(states_(2));
    double cb = cos(states_(2));
    double sph = sin(euler.x());
    double cph = cos(euler.x());
    double sth = sin(euler.y());
    double cth = cos(euler.y());
    double gamma = asin((ca*cb)*sth - (sph*sb + cph*sa*cb)*cth); // Stevens-Lewis 2003 3.4-2

    // Unused
    wind_body_.x() = bus_data.wind.x;
    wind_body_.y() = bus_data.wind.y;
    wind_body_.z() = bus_data.wind.z;

    // Calculate FE ellipsoid value
    double ellipsoid_value = fe_ellipsoid_.evaluate(states_(0), gamma, psi_dot);
    // ROS_INFO("***Ellipsoid value: %f", ellipsoid_value);
    // mpcController_.printSolverState();

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

    // Run airspeed controller loop

    geometry_msgs::Vector3Stamped throttleCmd;
    throttleCmd.vector.x = deltat_;
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

double TrajectoryController::estimateThrust(const double Va, const double deltat)
{
    // Estimate the RPS the motor is running at
    double p1 = -133.5;
    double p2 =  306.6;
    double p3 =   10.2;
    double rps_estimate = p1*deltat*deltat + p2*deltat + p3;

    // Create estimates of the propeller force and torque about the CG
    // Based on the APC Electric 10x7 propeller
    const double prop_d = 10*2.54/100;
    double J = Va/(rps_estimate*prop_d);
    double c_t = 0.2202*J*J*J -0.4277*J*J + 0.0756*J + 0.1054;
    double thrust_estimate = c_t*pow(rps_estimate,2)*pow(prop_d,4);

    return thrust_estimate;
}

/**
 * @brief Callback to store the velocity commands
 * 
 * @param pRefRates The reference velocity commands Va, gamma, psi_dot
 */
void TrajectoryController::getReference(geometry_msgs::Vector3Stamped pRefTrajectory)
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
    referenceTrajectory_ = projected_setpoint.cast<float>();
    // std::cout << "Projected setpoint: " << referenceTrajectory_.transpose() << std::endl;

    reference_(0) = referenceTrajectory_(1); // Copy over gamma   
    reference_(1) = referenceTrajectory_(2); // Copoy over psi_dot
    reference_(2) = 0.0f; // AoA reference
    reference_(3) = 0.0f; // AoS reference
    // Calculate an estimate for the final roll angle
    float g = 9.81;
    float endPhi = atan(reference_(1)*airdata_(0)/g);
    reference_(7) = endPhi;
    reference_.segment(kStateSize, refInputs_.size()) = refInputs_;
    // std::cout << "Generated reference: " << reference_.transpose() << std::endl;

    endReference_(0) = referenceTrajectory_(1);
    endReference_(1) = endPhi;
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
    const int variables_start_idx = inertial_start_idx + 1; // There is 1 inertial parameter
    const int aerodynamic_start_idx = variables_start_idx + 2; // There are 2 variables
    const int ellipsoid_start_idx = aerodynamic_start_idx + 9; // There are 9 aerodynamic parameters

    if ((paramType == 3) || (paramType == 4)) // Inertial and aerodynamic parameters parameters
    {
        ROS_INFO("Got new parameter %s with value %g", paramName.c_str(), paramValue);

        for (int i=inertial_start_idx; i < ellipsoid_start_idx-1; i++) // Skip unrelated parameters
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

void TrajectoryController::getDefaultWeights(ros::NodeHandle pnh) // Read default MPC weights
{
    Eigen::Matrix<float, kRefSize, kRefSize> default_W = Eigen::Matrix<float, kRefSize, kRefSize>::Identity(); 

    // Running cost weights
    double w_gamma;
    double w_psi_dot;
    double w_alpha;
    double w_beta;
    double w_p;
    double w_q;
    double w_r;
    double w_phi;
    pnh.param("wgamma", w_gamma, 1000.0);
    pnh.param("wpsidot", w_psi_dot, 10000.0);
    pnh.param("walpha", w_alpha, 0.01);
    pnh.param("wbeta", w_beta, 100.0);
    pnh.param("wdp", w_p, 1.0);
    pnh.param("wdq", w_q, 0.5);
    pnh.param("wdr", w_r, 10.0);
    pnh.param("wphi", w_phi, 0.01);

    default_W(0,0) = w_gamma;
    default_W(1,1) = w_psi_dot;
    default_W(2,2) = w_alpha;
    default_W(3,3) = w_beta;
    default_W(4,4) = w_p;
    default_W(5,5) = w_q;
    default_W(6,6) = w_r;
    default_W(7,7) = w_phi;

    Eigen::Matrix<float, kEndRefSize, kEndRefSize> default_WN = Eigen::Matrix<float, kEndRefSize, kEndRefSize>::Identity(); 

    // End cost weights
    double wn_gamma;
    double wn_phi;
    pnh.param("wngamma", wn_gamma, w_gamma);
    pnh.param("wnphi", wn_phi, 100.0);

    default_WN(0,0) = wn_gamma;
    default_WN(1,1) = wn_phi;

    mpcController_.setDefaultCosts(default_W, default_WN);
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
    const int variables_start_idx = inertial_start_idx + 1; // There is 1 inertial parameter
    const int aerodynamic_start_idx = variables_start_idx + 2; // There are 2 variables
    const int ellipsoid_start_idx = aerodynamic_start_idx + 9; // There are 9 aerodynamic parameters

    // Read inertial parameters
    std::string param_name = online_data_names[(unsigned int) Parameter::m];
    trimOnlineData_(0) = configStruct.inertial[param_name].as<float>();
    trimOnlineData_(1) = 15.0; // Trim airspeed
    trimOnlineData_(2) = 0.0; // Trim Fprop

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

bool TrajectoryController::getDefaultBounds(ros::NodeHandle pnh) // Read default state and input bounds
{
    // Input constraints
    double p_min;
    double q_min;
    double r_min;
    double p_max;
    double q_max;
    double r_max;
    pnh.param("pMin", p_min, -1.5);
    pnh.param("qMin", q_min, -1.5);
    pnh.param("rMin", r_min, -0.5);
    pnh.param("pMax", p_max, 1.5);
    pnh.param("qMax", q_max, 1.5);
    pnh.param("rMax", r_max, 0.5);

    // // State constraints
    // double alpha_min;
    // double beta_min;
    // double phi_min;
    // double theta_min;
    // double alpha_max;
    // double beta_max;
    // double phi_max;
    // double theta_max;
    // //TODO: Convert defaults to radians
    // pnh.param("alphaMin", alpha_min, -45.0);
    // pnh.param("betaMin", beta_min, -45.0);
    // pnh.param("phiMin", phi_min, -120.0);
    // pnh.param("thetaMin", theta_min, -120.0);
    // pnh.param("alphaMax", alpha_max, 45.0);
    // pnh.param("betaMax", beta_max, 45.0);
    // pnh.param("phiMax", phi_max, 120.0);
    // pnh.param("thetaMax", theta_max, 120.0);

    defaultBoundsMin_.resize(numConstraints_, Eigen::NoChange);
    defaultBoundsMin_ <<
        p_min,
        q_min,
        r_min
        // alpha_min,
        // beta_min,
        // phi_min,
        // theta_min
    ;
    mpcController_.setDefaultBoundsLower(defaultBoundsMin_);

    defaultBoundsMax_.resize(numConstraints_, Eigen::NoChange);
    defaultBoundsMax_ <<
        p_max,
        q_max,
        r_max
        // alpha_max,
        // beta_max,
        // phi_max,
        // theta_max
    ;
    mpcController_.setDefaultBoundsUpper(defaultBoundsMax_);
}

void TrajectoryController::getFlightEnvelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr& fe_msg)
{
    // ROS_INFO("Received new Flight Envelope");
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_A, fe_msg->el_A);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_B, fe_msg->el_B);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_C, fe_msg->el_C);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_D, fe_msg->el_D);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_E, fe_msg->el_E);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_F, fe_msg->el_F);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_G, fe_msg->el_G);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_H, fe_msg->el_H);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_I, fe_msg->el_I);
    mpcController_.setOnlineDataSingle((unsigned int) Parameter::el_J, fe_msg->el_J);
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
int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectoryControlNode");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    // ros::WallDuration(1).sleep(); //wait for other nodes to get raised
    double ctrlRate;
    if (!pnh.getParam("rate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find -~rate- parameter");
        ros::shutdown();
    }
    ros::Rate spinner(ctrlRate);

    TrajectoryController rate_ctrl(n, pnh);
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
