#include <ros/ros.h>
#include <Eigen/Eigen>

#include <uav_ftc/BusData.h>


class ReferenceGenerator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //////////
    // Methods
    ReferenceGenerator(ros::NodeHandle n);
    void rcCallback(uav_ftc::BusData joyMsg); // Joystick input callback
    void publishCmds(Eigen::Vector3d ref);
    Eigen::Vector3d convertInputs(double *input);

private:
    ////////////
    // Variables
    ros::NodeHandle n_;
    Eigen::Vector3d referenceMin_, referenceMax_; // Reference commands scaling multiplier
    Eigen::Vector3d reference_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double inpChannels_[8];
    int ctrlMode_;
};
