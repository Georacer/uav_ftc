#include "rate_controller.hpp"

///////////////////
//Class Constructor
RateController::RateController(ros::NodeHandle n) : mpcController()
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
    mpcController.setOnlineData(airdata_);
    // Initialize system state
    angularStates_ << 0.0f, 0.0f, 0.0f;
    mpcController.setInitialState(angularStates_);
    // Initialize reference
    refRates_ << 0.0f, 0.0f, 0.0f;
    mpcController.setReferencePose(refRates_);

    //Subscribe and advertize
    subState = n.subscribe("states", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3Stamped>("ctrlSurfaceCmds", 100);
}

///////////////////
//Class Destructor
RateController::~RateController()
{
    delete &mpcController;
}

////////////
// Main Step
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
    mpcController.setReferencePose(refRates_);

    // Solve problem with given measurements
    mpcController.solve(angularStates_, airdata_);

    // Write the resulting controller output
    readControls();

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

void RateController::getStates(last_letter_msgs::SimStates inpStates)
{
    // Copy over all aircraft state
    states_ = inpStates;

    // Isolate angular velocity system measurements
    angularStates_(0) = inpStates.velocity.angular.x;
    angularStates_(1) = inpStates.velocity.angular.y;
    angularStates_(2) = inpStates.velocity.angular.z;
}

void RateController::readControls()
{
    mpcController.getInput(0, predicted_controls_);
}

///////////////////////////////////////////////////////
// Controller outputs to us PPM values and publish them
void RateController::writeOutput()
{
    geometry_msgs::Vector3Stamped channels;
    // MPC controls are in radians, must convert them to -1,1 range
    double deltaa_max, deltae_max, deltar_max;
    ros::param::getCached("airfoil1/deltaa_max", deltaa_max);
    ros::param::getCached("airfoil1/deltae_max", deltae_max);
    ros::param::getCached("airfoil1/deltar_max", deltar_max);
    channels.vector.x = predicted_controls_(0) / deltaa_max; // Pass aileron input
    channels.vector.y = predicted_controls_(1) / deltae_max; // Pass elevator input
    channels.vector.z = predicted_controls_(2) / deltar_max;   // Pass rudder input
    std::cout << "Writing controls: " << channels.vector << std::endl;
    channels.header.stamp = ros::Time::now();
    pubCtrl.publish(channels);
}

/////////////////////////////////////////////////
// Store reference commands
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
