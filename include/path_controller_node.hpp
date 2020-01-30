#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <last_letter_msgs/SimStates.h>
#include "path_controller.hpp"

class PathControllerROS {
    public:
    PathControllerROS(ros::NodeHandle);
    void cb_update_uav_states(const last_letter_msgs::SimStates::ConstPtr& nav_msg);
    void cb_update_path(const visualization_msgs::MarkerArray::ConstPtr& path_msg);
    void step();

    private:
    InputConstraints get_input_constraints(const ros::NodeHandle) const;
    MatrixXd get_obstacles(ros::NodeHandle) const;
    PathControllerSettings get_controller_settings(ros::NodeHandle) const;
    PathController path_controller;
    WaypointMngr waypoint_mngr_;
    ros::Publisher pub_uav_cmd_;
    ros::Subscriber sub_uav_state_;
    ros::Subscriber sub_uav_path_;
    Vector4d uav_state_;
    bool did_receive_wps{false};
    bool did_receive_state{false};
};