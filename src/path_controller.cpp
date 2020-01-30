#include "path_controller.hpp"

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>

#include <math_utils.hpp>


//RRT Planner
// MatrixXd pathControlsMatrix;

//Desired Inputs (From RRT)
VectorXd input_des(3);
//MPC Gain Matrices
MatrixXd Q;
MatrixXd R;
MatrixXd P;

VectorXd uav_q(4); // UAV States

VectorXd obs1(3); // Obstacles
VectorXd obs2(3);
VectorXd obs3(3);
VectorXd obs4(3);
VectorXd obs5(3);
VectorXd obs6(3);

PathController::PathController(const PathControllerSettings& s) {
    pc_settings_ = s;
    opt = nlopt_create(NLOPT_LN_COBYLA, pc_settings_.num_inputs*pc_settings_.num_samples); /* algorithm and dimensionality */
    
    //nlopt_set_population(opt, 10);

    /* Initialize Optimization*/
    double inputs_lb_arr[pc_settings_.num_inputs*pc_settings_.num_samples];
    double inputs_ub_arr[pc_settings_.num_inputs*pc_settings_.num_samples];
    
    for (int i = 0; i < pc_settings_.num_inputs*pc_settings_.num_samples ; i+=3){
        inputs_lb_arr[i] = s.input_constraints.psi_dot_min; //turn rate rad/s
        inputs_lb_arr[i+1] = s.input_constraints.gamma_min; //flight path angle rad
        inputs_lb_arr[i+2] = s.input_constraints.va_min; //velocity Va m/s

        inputs_ub_arr[i] = s.input_constraints.psi_dot_max; //turn rate rad/s
        inputs_ub_arr[i+1] = s.input_constraints.gamma_max; //flight path angle rad
        inputs_ub_arr[i+2] = s.input_constraints.va_max; //velocity Va m/s
    }
    nlopt_set_lower_bounds(opt, inputs_lb_arr);
    nlopt_set_upper_bounds(opt, inputs_ub_arr);

    nlopt_set_min_objective(opt, cost_function_wrapper, this);
    nlopt_set_ftol_abs(opt, 0.001);
    nlopt_set_xtol_abs1(opt, 0.001);

    input_target_ << 0, 0, 15;
    
    int num_constraints = pc_settings_.obstacles.rows();
    double constraints_tol[num_constraints*(pc_settings_.num_samples+1)];
    for (int k = 0; k < num_constraints*(pc_settings_.num_samples+1); k++)
        constraints_tol[k] = 0.01;
    // add constraints
    nlopt_add_inequality_mconstraint(opt,
                                     num_constraints*(pc_settings_.num_samples+1),
                                     constraints_wrapper,
                                     this,
                                     constraints_tol); //NO CONSTRAINTS

    //Initialize MPC Parameters
    
    Q_.setIdentity(pc_settings_.num_states, pc_settings_.num_states);
    Q(0,0) = 0.1;
    Q(1,1) = 0.1;
    Q(2,2) = 0.1;
    Q(3,3) = 0.04;

    R_.setIdentity(pc_settings_.num_inputs, pc_settings_.num_inputs);
    R(0,0) = 0.002;
    R(1,1) = 0.002;
    R(2,2) = 0.002;

    P_.setIdentity(pc_settings_.num_states, pc_settings_.num_states);
    P = 5.0*Q;
}

PathController::~PathController(){
    nlopt_destroy(opt); 
}

VectorXd PathController::uav_model(VectorXd U) {    
    double wn = 0.0;
    double we = 0.0;
    double wd = 0.0;
    
    double pndot = U(2)*cos(uav_q(3))*cos(U(1)) + wn ;
    double pedot = U(2)*sin(uav_q(3))*cos(U(1)) + we ;
    double hdot =  U(2)*sin(U(1)) - wd;
    double psidot = U(0);
    
    VectorXd ndot(pc_settings_.num_states);
    ndot << pndot, pedot, hdot, psidot;

    return ndot;
}

double PathController::cost_function(unsigned int n, const double* x, double* grad){
    MatrixXd inputs(pc_settings_.num_inputs, pc_settings_.num_samples);
    Map<MatrixXd>((double*)x, inputs.rows(), inputs.cols()) = inputs;

    //Trajectory of States 
    MatrixXd traj_n(pc_settings_.num_states, pc_settings_.num_samples+1);
    traj_n.setZero(pc_settings_.num_states, pc_settings_.num_samples+1);
    
    //Initialize state trajectory
    traj_n.col(0) = uav_state_;
    
    //cout << "TRAJ:" << traj_n.col(0) << endl;

    //Progate the model (kinematics)
    for (int k = 0; k < pc_settings_.num_samples; k++){

        //cout << "HERE 1" << endl;
        VectorXd ndot = uav_model(inputs.col(k));
        traj_n.col(k+1) = traj_n.col(k) + ndot*dt_;
        //traj_n.col(k+1) = uvmsModel(traj_n.col(k), inputs.col(k));
    }
  
    //Calculate Running Costs
    double Ji{0};
    for (int k = 0; k < pc_settings_.num_samples; ++k) {
       VectorXd x_err = traj_n.col(k) - state_target_.segment<3>(0);
       Ji += x_err.transpose()*Q_*x_err;
       VectorXd u_err = inputs.col(k) - input_target_;
       Ji += u_err.transpose()*R_*u_err;
    }
 
    //Calculate Terminal Costs
    VectorXd et = traj_n.col(pc_settings_.num_samples) - state_target_.segment<3>(0);
    double Jt{et.transpose()*P_*et};
    return Ji + Jt;
}

void PathController::constraints(unsigned int m, double* c, unsigned int n, const double* x, double* grad) {
    MatrixXd inputs(pc_settings_.num_inputs, pc_settings_.num_samples);
    Map<MatrixXd>((double*)x, inputs.rows(), inputs.cols()) = inputs;

    //Trajectory of States 
    MatrixXd traj_n(pc_settings_.num_states, pc_settings_.num_samples+1);
    traj_n.setZero(pc_settings_.num_states, pc_settings_.num_samples+1);
  
    //Initialize state trajectory
    traj_n.col(0) = uav_state_;
    
    // Propagate model state
    for (int k = 0; k < pc_settings_.num_samples; k++){
        VectorXd ndot = uav_model(inputs.col(k));
        traj_n.col(k+1) = traj_n.col(k) + ndot*dt_;
    }
    // Check constraint validity for each time sample
    for (int i = 0; i < pc_settings_.num_samples + 1; i++){ // Sample iterator
        MatrixXd obstacle_pos = pc_settings_.obstacles.leftCols<3>(); // Isolate obstacle positions
        VectorXd obstacle_rad = pc_settings_.obstacles.rightCols<1>(); // Isolate obstacle radii

        // Calculate squared distance of uav from obstacles surface
        int num_obstacles = pc_settings_.obstacles.rows();
        VectorXd cons_vector(num_obstacles); // Constraints vector for this sample
        for (int j=0; j<num_obstacles; ++j){
            Vector3d rel_pos = uav_state_.segment<3>(0) - obstacle_pos.row(j).transpose();
            cons_vector(j) = pow(obstacle_rad(j), 2) - rel_pos.transpose()*rel_pos;
        }

        // Convert squared distances to C-style array
        for (int j = 0; j<num_obstacles; ++j){ // Obstacle iterator
            c[num_obstacles*i+j] = cons_vector(j);
        }
    }
}

void PathController::set_waypoints(Matrix<double, Dynamic, 3> waypoints)
{
    waypoints_ = waypoints;
    did_receive_waypoints_ = true;
}

void PathController::step(Vector4d state){

    if (!did_receive_waypoints_) {
        setup_error err("waypoints");
        throw err;
    }

    uav_state_ = state;

    // Find current waypoint
    Vector3d pos = state.segment<3>(0);
    wp_target_ = waypoints_.row(get_current_wp_idx(pos)).transpose();

    // Declare input structure with initial guess
    int input_arr_size = pc_settings_.num_inputs*pc_settings_.num_samples;
    double inputs[input_arr_size];
    for (int i = 0; i < input_arr_size ; i+=3){
        inputs[i] = input_target_(0); //turn rate rad/s
        inputs[i+1] = input_target_(1); //flight path angle rad
        inputs[i+2] = input_target_(2); //velocity Va m/s
    }
    // Call solver
    nlopt_result res = nlopt_optimize(opt, inputs, &minJ);
    // cout << "Optimization Return Code: " << res;
    // printf("found minimum at J(%g,%g,%g) = %g\n", inputs[0], inputs[1],inputs[2], minJ);
    // cout << "STATE:" <<uav_q.transpose() << endl;
    // cout << "NAVIGATING TO WP: " <<wpCounter<< " VALUE: " << n_des.transpose() << endl;

    // Store first sesults
    input_result(0) = inputs[0];
    input_result(1) = inputs[1];
    input_result(2) = inputs[2];

    // Increment waypoint if needed
    double distance_to_wp = get_distance_from_target(pos.segment<2>(0), wp_target_.segment<2>(0));
    if (distance_to_wp < goal_radius_){
        if (wp_counter-1<waypoints_.rows()){
            wp_counter += 1;
        }
    }
}

/**
 * @brief Calculate the distance from the perpendicular line to the target.
 Essentially it's the projection of the current heading onto the target direction
 * 
 * @param start 
 * @param finish 
 * @param pos 
 * @return double 
 */
double PathController::get_distance_from_target(Vector2d pos, Vector2d start, Vector2d finish) const{
    // Build vector from prev wp to next wp
    Vector2d v_ab = start - finish;
    v_ab.normalize();
    // Build vector from current pos to next wp
    Vector2d v_pb = finish - pos;
    return v_ab.dot(v_pb);
}

double PathController::get_distance_from_target(Vector2d pos, Vector2d target) const{
    // Build vector from prev wp to next wp
    Vector2d v_at = pos - target;
    return v_at.norm();
}

int PathController::get_current_wp_idx(Vector3d pos) const{
    int num_wp = waypoints_.rows();
    for (int wp_idx = wp_counter+num_wp_lookahead; wp_idx<num_wp; ++num_wp){
        // Construct the previous waypoint
        int wp_prev_idx = wp_idx -1;
        Vector2d wp_prev;
        if (wp_prev_idx<0){
            wp_prev = pos.segment<2>(0);
        }
        else{
            wp_prev = waypoints_.row(wp_prev_idx).transpose().segment<2>(0);
        }
        // Pick the next waypoint
        Vector2d wp_next = waypoints_.row(wp_idx).transpose().segment<2>(0);
        // Find distance from next wp
        double distance = get_distance_from_target(wp_prev, wp_next, pos.segment<2>(0));
        // If we are before the next wp return its index
        if (distance>0){
            return wp_idx;
        }
    }
    // We are past the last waypoint, keep tracking it
    return num_wp-1;
}

double cost_function_wrapper(unsigned int n, const double* x, double* grad, void* data) {
    PathController* pc_ptr = static_cast<PathController*>(data);
    return pc_ptr->cost_function(n, x, grad);
}

void constraints_wrapper(unsigned int m, double* c, unsigned int n, const double* x, double* grad, void* data) {
    PathController* pc_ptr = static_cast<PathController*>(data);
    return pc_ptr->constraints(m, c, n, x, grad);
}


// void updatePathControls(const std_msgs::Float32MultiArray::ConstPtr& msg){

//     int nrows = msg->layout.dim[0].size;
//     int ncols = msg->layout.dim[1].size;
    
//     pathControlsMatrix.setZero(nrows, ncols);

//     for (int i=0;i<nrows;i++)
//         for (int j=0;j<ncols;j++)
//             pathControlsMatrix(i,j) = msg->data[i*ncols+j];

//     pathControlsMatrix.conservativeResize(pathControlsMatrix.rows(), pathControlsMatrix.cols()+1);
//     pathControlsMatrix.col(0).setZero();
//     return;
// }

// VectorXd getPwmCmd(double* input){

//     double Rmin = 50.0;
//     double Vamax = 25.0;
//     double Vamin = 10.0;
//     double gammamax = 20.0*M_PI/180.0;
//     double gammamin = -20.0*M_PI/180.0;
//     double psi_dot_max = Vamax/Rmin;
//     double psi_dot_min = -psi_dot_max;

//     double Va_dash = (Vamax + Vamin)/2.0;
//     double gamma_dash = (gammamax + gammamin)/2.0;
//     double psi_dot_dash = (psi_dot_max + psi_dot_min)/2.0;

//     double Va_span = Vamax - Vamin;
//     double gamma_span = gammamax - gammamin;
//     double psi_dot_span = (psi_dot_max - psi_dot_min);

//     double Va_pwm = ((input[2] - Va_dash)/(Va_span/2.0))*500.0 + 1500.0;
//     double gamma_pwm = ((-input[1] - gamma_dash)/(gamma_span/2.0))*500.0 + 1500.0; 
//     double psi_dot_pwm = ((input[0] - psi_dot_dash)/(psi_dot_span/2.0))*500.0 + 1500.0;

//     VectorXd pwm;
//     pwm.setZero(3);
//     pwm(0) = psi_dot_pwm;
//     pwm(1) = gamma_pwm;
//     pwm(2) = Va_pwm;

//     return pwm;
// }