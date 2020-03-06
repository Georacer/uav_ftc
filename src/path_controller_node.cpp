#include "path_controller_node.hpp"

#include <geometry_msgs/Vector3Stamped.h>

#include <math_utils.hpp>


constexpr int k_number_of_states = 4;
constexpr int k_number_of_inputs = 3;
constexpr int k_mpc_number_of_samples = 4;


PathControllerROS::PathControllerROS(ros::NodeHandle nh)
    : path_controller{get_controller_settings(nh)} {

    double wp_radius;
    ros::param::param<double>("waypointRadius", wp_radius, 10);
    waypoint_mngr_.set_goal_radius(wp_radius);

    //Create Publishers
    pub_uav_cmd_ = nh.advertise<geometry_msgs::Vector3Stamped>("refTrajectory", 10);
    //Create Subscribers
    sub_uav_state_ = nh.subscribe<uav_ftc::BusData>("dataBus", 100, &PathControllerROS::cb_update_uav_states, this);
    sub_uav_path_ = nh.subscribe<visualization_msgs::MarkerArray>("waypoints", 100,
                                                                  &PathControllerROS::cb_update_path, this);
    sub_flight_envelope_ = nh.subscribe<uav_ftc::FlightEnvelopeEllipsoid>(
        "flight_envelope", 1, &PathControllerROS::cb_update_flight_envelope, this);
}

/**
 * @brief Callback to read the state of the UAV
 * 
 * @param nav_msg 
 */
void PathControllerROS::cb_update_uav_states(const uav_ftc::BusData::ConstPtr& nav_msg){
    uav_state_(0)= nav_msg->position.x;
    uav_state_(1)= nav_msg->position.y;
    uav_state_(2)= nav_msg->position.z;

    geometry_msgs::Quaternion quat;
    quat = nav_msg->orientation;
    Quaterniond rot_quat(quat.w, quat.x, quat.y, quat.z);
    Vector3d rot_euler = quat2euler(rot_quat);
    
    uav_state_(3) = rot_euler.z();
    did_receive_state = true;
    
    return;
}

/**
 * @brief Callback to read the series of planned waypoints
 * 
 * @param path_msg 
 */
void PathControllerROS::cb_update_path(const visualization_msgs::MarkerArray::ConstPtr& path_msg){
    // Reveive waypoints only once. Do not support updating waypoints yet
    if (!did_receive_wps){
        int path_size = path_msg->markers.size();

        MatrixXd wp_matrix;
        wp_matrix.setZero(path_size, 3);
        for (int idx=0; idx!=path_size; ++idx){
            wp_matrix(idx, 0) = path_msg->markers[idx].pose.position.x;
            wp_matrix(idx, 1) = path_msg->markers[idx].pose.position.y;
            wp_matrix(idx, 2) = path_msg->markers[idx].pose.position.z;
        }
        waypoint_mngr_.set_waypoints(wp_matrix);
        did_receive_wps = true;
    }

    return;
}

void PathControllerROS::cb_update_flight_envelope(const uav_ftc::FlightEnvelopeEllipsoid::ConstPtr& fe_msg)
{
    ROS_INFO("Received new Flight Envelope");
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
    path_controller.set_fe_ellipsoid(coeffs);
    // TODO: could also pass box constraints provided in fe_msg
}

void PathControllerROS::step()
{
    // Only call the controller if we have a valid position and waypoint sequence
    if (did_receive_wps && did_receive_state) {
        Vector3d pos = uav_state_.segment<3>(0);
        Vector3d waypoint = waypoint_mngr_.next_waypoint(pos);
        path_controller.step(uav_state_, waypoint);

        // Grab resulting inputs
        Vector3d input = path_controller.input_result;
        
        // Publish them
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";

        msg.vector.x = input.z(); // Copy over airspeed
        msg.vector.y = input.y(); // Copy over gamma
        msg.vector.z = input.x(); // Copy over psi_dot

        pub_uav_cmd_.publish(msg);
    }
    return;
}

InputConstraints PathControllerROS::get_input_constraints(const ros::NodeHandle nh) const {
    ROS_INFO("Reading path controller constraints");
    InputConstraints input_constraints;
    XmlRpc::XmlRpcValue listDouble;

    if (!ros::param::getCached("referenceMin", listDouble))
    {
        ROS_FATAL("Invalid parameters for -referenceMin- in param server!");
        ros::shutdown();
    }
    input_constraints.va_min = listDouble[0];
    input_constraints.gamma_min = listDouble[1];
    input_constraints.psi_dot_min = listDouble[2];

    if (!ros::param::getCached("referenceMax", listDouble))
    {
        ROS_FATAL("Invalid parameters for -referenceMax- in param server!");
        ros::shutdown();
    }
    input_constraints.va_max = listDouble[0];
    input_constraints.gamma_max = listDouble[1];
    input_constraints.psi_dot_max = listDouble[2];
    return input_constraints;
}

/**
 * @brief Access the parameter server for obstacle information
 * 
 * @param n ROS nodehandle
 */
MatrixXd PathControllerROS::get_obstacles(ros::NodeHandle n) const {
    MatrixXd obstacle_array;

    // Read the controller configuration parameters from the obstacles.yaml file
    XmlRpc::XmlRpcValue listDouble;
    ROS_INFO("Reading obstacles");
    int obstacles_num;
    if (!ros::param::getCached("obstacle/obs_num", obstacles_num))
    {
        ROS_FATAL("Invalid parameters for -obs_num- in param server!");
        ros::shutdown();
    }
    obstacle_array.setZero(obstacles_num, 4);

    for (int i=0; i<obstacles_num; ++i)
    {
        std::string obs_name = "obstacle/obs" + std::to_string(i+1);
        if (!ros::param::getCached(obs_name.c_str(), listDouble))
        {
            ROS_FATAL("Invalid parameters for -%s- in param server!", obs_name.c_str());
            ros::shutdown();
        }
        for (int j = 0; j < listDouble.size(); ++j)
        {
            ROS_ASSERT(listDouble[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            obstacle_array(i,j) = listDouble[j];
        }
    }

    return obstacle_array;
}

PathControllerSettings PathControllerROS::get_controller_settings(ros::NodeHandle nh) const {
    ROS_INFO("Reading path controller settings");
    PathControllerSettings controller_settings;

    controller_settings.num_states = k_number_of_states;
    controller_settings.num_inputs = k_number_of_inputs;
    controller_settings.num_samples = k_mpc_number_of_samples;

    double ctrl_path_rate;
    if (!ros::param::getCached("ctrlPathRate", ctrl_path_rate))
    {
        ROS_FATAL("Invalid parameters for -ctrlPathRate- in param server!");
        ros::shutdown();
    }
    controller_settings.dt = 1.0/ctrl_path_rate;

    controller_settings.obstacles = get_obstacles(nh);
    controller_settings.input_constraints = get_input_constraints(nh);

    return controller_settings;
}


int main (int argc, char **argv)
{       
    ros::init (argc, argv, "uav_mpc_node");
    ros::NodeHandle nh;

    PathControllerROS path_controller_ros(nh);

    double ctrlRate;
    if (!ros::param::get("ctrlPathRate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find ctrlPathRate parameter");
        ros::shutdown();
    }
    ros::Rate spinner(ctrlRate);

    while (ros::ok()) {
        ros::spinOnce();
        ros::Time t_start = ros::Time::now();
        path_controller_ros.step();
        // missing start time
        ros::Time t_end =ros::Time::now();
        ros::Duration t_calc = t_end-t_start;
        printf("Path Controller calculations dt:%lf\n", t_calc.toSec());
        //ROS Rate
        spinner.sleep();
    }
    return 0;
}