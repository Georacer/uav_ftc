#include "rate_controller.hpp"

///////////////////
//Class Constructor
RateController::RateController(ros::NodeHandle n)
{
    //Initialize states
    tprev = ros::Time::now();
    states.header.stamp = tprev;

    //Subscribe and advertize
    subState = n.subscribe("states", 1, &RateController::getStates, this);
    subRef = n.subscribe("refRates", 1, &RateController::getReference, this);
    pubCtrl = n.advertise<geometry_msgs::Vector3>("ctrlSurfaceCmds", 100);

    // RateMpcController already initialized in declaration
}

///////////////////
//Class Destructor
RateController::~RateController()
{
    delete mpcController;
}

////////////
// Main Step
void RateController::step()
{
    // Copy over p, q, r
    geometry_msgs::Vector3 currentState = states.velocity.angular;

    // Convert airdata triplet
    airdata = getAirData(states.velocity.linear);

    // refRates are already saved

    // Set mpc OnlineData with airdata
    // Set mpc x0 with states
    // Set mpc reference with refRates

    float output[4];
    // Fill control commands and publish them
    writePWM(output);
}

///////////
//Utilities
///////////

void RateController::getStates(last_letter_msgs::SimStates inpStates)
{
    states = inpStates;
}

///////////////////////////////////////////////////////
// Controller outputs to us PPM values and publish them
void RateController::writeOutput()
{
    last_letter_msgs::SimPWM channels;
    channels.value[0] = FullRangeToPwm(output[0]);
    channels.value[1] = FullRangeToPwm(output[1]);
    channels.value[2] = HalfRangeToPwm(output[2]);
    channels.value[3] = FullRangeToPwm(output[3]);
    channels.value[9] = HalfRangeToPwm(output[9]);
    channels.header.stamp = ros::Time::now();
    pubCtrl.publish(channels);
}

/////////////////////////////////////////////////
// Store reference commands
void RateController::getReference(uav_ftc::refRates pRefRates)
{
    refRates = pRefRates;
}

RateMpcWrapper<float> rate_ctrl();

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
