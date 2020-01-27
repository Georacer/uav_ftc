#include "path_controller.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/SimPWM.h>


constexpr number_of_states = 4;
constexpr number_of_inputs = 3;
constexpr mpc_number_of_samples = 4;
constexpr number_of_constraints = 6;

InputConstraints getConstraints(const ros::NodeHandle nh)
{
    ROS_INFO("Reading path controller constraints");
    InputConstraints input_constraints;

    if (!ros::param::getCached("constraint/Va/min", input_constraints.va_min))
    {
        ROS_FATAL("Invalid parameters for -constraint/Va/min- in param server!");
        ros::shutdown();
    }
    if (!ros::param::getCached("constraint/Va/max", constraints_map[pcc::va_max]))
    {
        ROS_FATAL("Invalid parameters for -constraint/Va/max- in param server!");
        ros::shutdown();
    }
    if (!ros::param::getCached("constraint/gamma/min", constraints_map[pcc::gamma_min]))
    {
        ROS_FATAL("Invalid parameters for -constraint/gamma/min- in param server!");
        ros::shutdown();
    }
    if (!ros::param::getCached("constraint/gamma/max", constraints_map[pcc::gamma_max]))
    {
        ROS_FATAL("Invalid parameters for -constraint/gamma/max- in param server!");
        ros::shutdown();
    }
    if (!ros::param::getCached("constraint/psi_dot/min", constraints_map[pcc::psi_dot_min]))
    {
        ROS_FATAL("Invalid parameters for -constraint/psi_dot/min- in param server!");
        ros::shutdown();
    }
    if (!ros::param::getCached("constraint/psi_dot/max", constraints_map[pcc::psi_dot_max]))
    {
        ROS_FATAL("Invalid parameters for -constraint/psi_dot/max- in param server!");
        ros::shutdown();
    }
    return constraints_map;
}

void 
updateUAVStates(const last_letter_msgs::SimStates::ConstPtr& nav_msg){

        uav_q(0)= nav_msg->pose.position.x;
        uav_q(1)= nav_msg->pose.position.y;
        uav_q(2)= nav_msg->pose.position.z;

        geometry_msgs::Quaternion quat;
        quat = nav_msg->pose.orientation;
        Quaterniond rot_quat(quat.w, quat.x, quat.y, quat.z);
        Vector3d rot_euler = quat2euler(rot_quat);
     
        uav_q(3) = rot_euler.z();
        
        return;
}

void
updatePath(const nav_msgs::Path::ConstPtr& path_msg){
    int path_size = path_msg->poses.size();

    cout << path_size << endl;

    wpMatrix.setZero(3,path_size);
    
    for(int j = 0; j<path_size ;j++){
    wpMatrix(0, j) = path_msg->poses[j].pose.position.x;
        wpMatrix(1, j) = -path_msg->poses[j].pose.position.y;
        wpMatrix(2, j) = -path_msg->poses[j].pose.position.z;
    }
    return;
}

void
updateObs1(const geometry_msgs::PointStamped::ConstPtr& obs_msg){
    obs1(0) = obs_msg->point.x;
    obs1(1) = -obs_msg->point.y;
    obs1(2) = -obs_msg->point.z;

    return;
}

void
updateObs2(const geometry_msgs::PointStamped::ConstPtr& obs_msg){
    obs2(0) = obs_msg->point.x;
    obs2(1) = -obs_msg->point.y;
    obs2(2) = -obs_msg->point.z;

    return;
}


void
updateObs3(const geometry_msgs::PointStamped::ConstPtr& obs_msg){
    obs3(0) = obs_msg->point.x;
    obs3(1) = -obs_msg->point.y;
    obs3(2) = -obs_msg->point.z;

    return;
}

void
updateObs4(const geometry_msgs::PointStamped::ConstPtr& obs_msg){
    obs4(0) = obs_msg->point.x;
    obs4(1) = -obs_msg->point.y;
    obs4(2) = -obs_msg->point.z;

    return;
}

void
updateObs5(const geometry_msgs::PointStamped::ConstPtr& obs_msg){
    obs5(0) = obs_msg->point.x;
    obs5(1) = -obs_msg->point.y;
    obs5(2) = -obs_msg->point.z;

    return;
}

void
updateObs6(const geometry_msgs::PointStamped::ConstPtr& obs_msg){
    obs6(0) = obs_msg->point.x;
    obs6(1) = -obs_msg->point.y;
    obs6(2) = -obs_msg->point.z;

    return;
}

int
main (int argc, char **argv)
{       
    ros::init (argc, argv, "uav_mpc_node");
    ros::NodeHandle nh;

    //Create Publishers
    //ros::Publisher pub_uav_cmd = nh.advertise<sensor_msgs::Joy>("/aircraft_navigation/joy", 1000);
    ros::Publisher pub_uav_cmd = nh.advertise<last_letter_msgs::SimPWM>("/aircraft_navigation/ctrlPWM", 1000);
    //Create Subscribers
    ros::Subscriber state_uav_sub = nh.subscribe<last_letter_msgs::SimStates>("/aircraft_navigation/states", 1000, updateUAVStates);
    ros::Subscriber plan_uav_sub = nh.subscribe<nav_msgs::Path>("/aircraft_navigation/rrt_path", 1000, updatePath);
    ros::Subscriber plan_controls_uav_sub = nh.subscribe<std_msgs::Float32MultiArray>("/aircraft_navigation/rrt_path_controls", 1000, updatePathControls);
    ros::Subscriber obs1_sub = nh.subscribe<geometry_msgs::PointStamped>("/aircraft_navigation/obs1", 1000, updateObs1);
    ros::Subscriber obs2_sub = nh.subscribe<geometry_msgs::PointStamped>("/aircraft_navigation/obs2", 1000, updateObs2);
    ros::Subscriber obs3_sub = nh.subscribe<geometry_msgs::PointStamped>("/aircraft_navigation/obs3", 1000, updateObs3);
    ros::Subscriber obs4_sub = nh.subscribe<geometry_msgs::PointStamped>("/aircraft_navigation/obs4", 1000, updateObs4);
    ros::Subscriber obs5_sub = nh.subscribe<geometry_msgs::PointStamped>("/aircraft_navigation/obs3", 1000, updateObs5);
    ros::Subscriber obs6_sub = nh.subscribe<geometry_msgs::PointStamped>("/aircraft_navigation/obs4", 1000, updateObs6);
         
    //Initialize MPC Variables
    
    Q.setIdentity(number_of_states, number_of_states);
    R.setIdentity(number_of_inputs,number_of_inputs);
    P.setIdentity(number_of_states,number_of_states);

    Q(0,0) = 0.1;
    Q(1,1) = 0.1;
    Q(2,2) = 0.1;
    Q(3,3) = 0.04;
    
    R(0,0) = 0.002;
    R(1,1) = 0.002;
    R(2,2) = 0.002;
    
    P = 5.0*Q;
    
    /* Initialize Optimization*/
    double inputs_lb[number_of_inputs*mpc_number_of_samples];
    double inputs_ub[number_of_inputs*mpc_number_of_samples];
    
    double Rmin = 50.0;
    double Vamax = 25.0;
    double Vamin = 10.0;
    double gammamax = 20.0*M_PI/180.0;
    double gammamin = -20.0*M_PI/180.0;
    double psi_dot_max = Vamax/Rmin;
    double psi_dot_min = -psi_dot_max;

    for (int i = 0; i < number_of_inputs*mpc_number_of_samples ; i+=3){
        inputs_lb[i] = psi_dot_min; //turn rate rad/s
        inputs_lb[i+1] = gammamin; //flight path angle rad
        inputs_lb[i+2] = Vamin; //velocity Va m/s

        inputs_ub[i] = psi_dot_max; //turn rate rad/s
        inputs_ub[i+1] = gammamax; //flight path angle rad
        inputs_ub[i+2] = Vamax; //velocity Va m/s
    }

    // Setup controller
    PathControllerSettings s;
    s.num_inputs = number_of_inputs;
    s.num_samples = number_of_samples;
    s.num_constraints = number_of_constraints;
    s.inputs_lb = inputs_lb;
    s.inputs_ub = inputs_ub;
    PathController path_controller{path_controller_settings};
   
    double inputs[number_of_inputs*mpc_number_of_samples];  /* some initial guess */
    double minJ; /* the minimum objective value, upon return */
    for (int i = 0; i < number_of_inputs*mpc_number_of_samples ; i+=3){
        inputs[i] = 0.0; //turn rate rad/s
        inputs[i+1] = 0.0; //flight path angle rad
        inputs[i+2] = Vamin; //velocity Va m/s
    }

    int counter_cycle = 0;

    //PROSOXH NEED TO PLACE the appropriate initial positions
    uav_q(0) = 0.0;
    uav_q(1) = 0.0;
    uav_q(2) = -100.0;
    uav_q(3) = 0.0;

    obs1.setZero(3);
    obs2.setZero(3);
    obs3.setZero(3);
    obs4.setZero(3);

    int wpCounter = 0;

    double ctrlRate;
    if (!ros::param::get("ctrlPathRate", ctrlRate)) //frame rate in Hz
    {
        ROS_ERROR("Could not find ctrlPathRate parameter");
        ros::shutdown();
    }
    ros::Rate spinner(ctrlRate);

    while (ros::ok())
    {
    
        counter_cycle = counter_cycle +1;
        cout << counter_cycle << endl;
        double start =ros::Time::now().toSec(); 

        VectorXd curr_state(4);
        curr_state.setZero(4);
        curr_state << uav_q(0), uav_q(1), uav_q(2), uav_q(3);

        //Classic MPC

        if (counter_cycle >= 20) {
            
            n_des.head(3) = wpMatrix.col(wpCounter);
            n_des(3) = atan2(n_des(1) - curr_state(1),n_des(0) - curr_state(0));

            input_des = pathControlsMatrix.col(wpCounter);            
                
            cout << "Optimization Return Code: " << nlopt_optimize(opt, inputs, &minJ) << endl;

            double pos_check = pow( curr_state(0) - n_des(0), 2 ) + pow( curr_state(1) - n_des(1), 2) + pow( curr_state(2) - n_des(2), 2) ;
            double orient_check = curr_state(3) - n_des(3);
        
            float R = 80.0;

            if ((pos_check <= R*R) && fabs(orient_check) <= 0.5){
                    cout << "Changing WP"<< endl;
                    wpCounter += 1;
                        if (wpCounter == wpMatrix.cols()){
                            wpCounter = 0;
                        }
                    n_des.head(3) = wpMatrix.col(wpCounter);
            }

            cout << "STATE:" <<uav_q.transpose() << endl;
            cout << "NAVIGATING TO WP: " <<wpCounter<< " VALUE: " << n_des.transpose() << endl;
            
        }
        printf("found minimum at J(%g,%g,%g) = %g\n", inputs[0], inputs[1],inputs[2], minJ);
        double end =ros::Time::now().toSec();
        printf("Loop dt:%lf\n", end-start);
        double loop_dt = end-start;
        
        VectorXd pwm_cmds;
        pwm_cmds.setZero(3);
        pwm_cmds = getPwmCmd(inputs);

        last_letter_msgs::SimPWM pwm_msg;
        pwm_msg.header.stamp = ros::Time::now();
        for (int i = 0 ; i<12;i++){
            pwm_msg.value[i] = 0.0;
        }
        pwm_msg.value[0] = pwm_cmds(0);
        pwm_msg.value[1] = pwm_cmds(1);
        pwm_msg.value[2] = pwm_cmds(2);

        pub_uav_cmd.publish(pwm_msg);

        ros::spinOnce();
        //ROS Rate
        spinner.sleep();
    }
        
    nlopt_destroy(opt); 

    return 0;
}
