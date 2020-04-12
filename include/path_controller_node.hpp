#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <math_utils.hpp>
#include <uav_ftc/BusData.h>
#include <uav_ftc/FlightEnvelopeEllipsoid.h>
#include "path_controller.hpp"

class PathControllerROS {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PathControllerROS(ros::NodeHandle nh, ros::NodeHandle pnh);
    void cb_update_uav_states(const uav_ftc::BusData::ConstPtr& nav_msg);
    void cb_update_path(const visualization_msgs::MarkerArray::ConstPtr& path_msg);
    void cb_update_flight_envelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr& fe_msg);
    void step();

    private:
    InputConstraints get_input_constraints(const ros::NodeHandle) const;
    MatrixXd get_obstacles(ros::NodeHandle) const;
    PathControllerSettings get_controller_settings(ros::NodeHandle nh, ros::NodeHandle pnh) const;
    PathController path_controller;
    WaypointMngr waypoint_mngr_;
    Ellipsoid3DCoefficients_t default_fe_coeffs_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0};  // Assign coeffs which always evaluate true
    Ellipsoid3D fe_ellipsoid_;
    ros::Publisher pub_uav_cmd_;
    ros::Subscriber sub_uav_state_;
    ros::Subscriber sub_uav_path_;
    ros::Subscriber sub_flight_envelope_;
    Vector4d uav_state_;
    bool did_receive_wps{false};
    bool did_receive_state{false};
};