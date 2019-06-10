#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>

class ReferenceGenerator
{
private:
    ////////////
    // Variables
    ros::NodeHandle n_;
    Eigen::Vector3d scaling_; // Reference commands scaling multiplier
    Eigen::Vector3d reference_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    // Parameters for conversion from joy message to channels
    int axisIndex_[11];
    int buttonIndex_[11];
    double throwIndex_[11];
    double inpChannels_[11]; // The recorded input channels

public:
    //////////
    // Methods
    ReferenceGenerator(ros::NodeHandle n);
    void joyCallback(sensor_msgs::Joy joyMsg); // Joystick input callback
    void publishCmds(Eigen::Vector3d ref);
    Eigen::Vector3d convertInputs(double *input);
};
