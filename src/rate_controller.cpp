#include "rate_controller.hpp"

///////////////////
//Class Constructor
RateController::RateController(ros::NodeHandle n) : mpcController()
{
    //Initialize states
    tprev = ros::Time::now();
    states_.header.stamp = tprev;

    //Subscribe and advertize
    subState = n.subscribe("states", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3Stamped>("ctrlSurfaceCmds", 100);

    // RateMpcController already initialized in declaration
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

    // refRates are already saved in the controller

    // Set the MPC reference state
    mpcController.setReferencePose(refRates_);

    // Solve problem with given measurements
    mpcController.solve(angular_states_, airdata_);

    // Write the resulting controller output
    readControls();

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
    angular_states_(0) = inpStates.velocity.angular.x;
    angular_states_(1) = inpStates.velocity.angular.y;
    angular_states_(2) = inpStates.velocity.angular.z;
}

void RateController::readControls()
{
    mpcController.getInput(0, predicted_controls_);
}

///////////////////////////////////////////////////////
// Controller outputs to us PPM values and publish them
void RateController::writeOutput()
{
    last_letter_msgs::SimPWM channels;
    channels.value[0] = FullRangeToPwm(predicted_controls_(0)); // Pass aileron input
    channels.value[1] = FullRangeToPwm(predicted_controls_(1)); // Pass elevator input
    channels.value[3] = FullRangeToPwm(predicted_controls_(2)); // Pass rudder input
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

// RateMpcWrapper<float> rate_ctrl(); // I think this is a mis-paste

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controlNode");
    ros::NodeHandle n;

    ros::WallDuration(3).sleep(); //wait for other nodes to get raised
    double ctrlRate;
    ros::param::get("ctrlRate", ctrlRate); //frame rate in Hz
    ros::Rate spinner(ctrlRate);

    RateController rate_ctrl(n);
    spinner.sleep();
    ROS_INFO("controlNode up");

    while (ros::ok())
    {
        ros::spinOnce();
        rate_ctrl.step();
        spinner.sleep();
    }

    return 0;
}
