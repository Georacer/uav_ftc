#include "path_controller.hpp"

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>

#include <nlopt.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#include <math_utils.hpp>


//RRT Planner
MatrixXd wpMatrix;
MatrixXd pathControlsMatrix;
//Global MPC Variables
double dt = 0.05; //This is the MPC dt

//Desired States (From RRT)
VectorXd n_des(4);
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

    opt = nlopt_create(NLOPT_LN_COBYLA, s.um_inputs*s.num_samples); /* algorithm and dimensionality */
    
    //nlopt_set_population(opt, 10);
    // Grab data pointer to array contents
    double* input_lower_bounds = s.&inputs_lb[0];
    double* input_upper_bounds = s.&inputs_ub[0];
    nlopt_set_lower_bounds(opt, input_lower_bounds);
    nlopt_set_upper_bounds(opt, input_upper_bounds);

    nlopt_set_min_objective(opt, this->costFunction, NULL);
    nlopt_set_ftol_abs(opt, 0.001);
    nlopt_set_xtol_abs1(opt, 0.001);
    
    double constraints_tol[s.num_constraints*(s.num_samples+1)];
    for (int k = 0; k < s.num_constraints*(s.num_samples+1); k++)
        constraints_tol[k] = 0.01;
    // add constraints
    nlopt_add_inequality_mconstraint(opt, s.num_constraints*(s.num_samples+1), constraints, NULL, constraints_tol); //NO CONSTRAINTS
}

VectorXd uavModel(VectorXd U){
    
    double wn = 0.0;
    double we = 0.0;
    double wd = 0.0;
    
    double pndot = U(2)*cos(uav_q(3))*cos(U(1)) + wn ;
    double pedot = U(2)*sin(uav_q(3))*cos(U(1)) + we ;
    double hdot =  U(2)*sin(U(1)) - wd;
    double psidot = U(0);
    
    VectorXd ndot(num_states);
    ndot << pndot, pedot, hdot, psidot;

    return ndot;
}

void updatePathControls(const std_msgs::Float32MultiArray::ConstPtr& msg){

    int nrows = msg->layout.dim[0].size;
    int ncols = msg->layout.dim[1].size;
    
    pathControlsMatrix.setZero(nrows, ncols);

    for (int i=0;i<nrows;i++)
        for (int j=0;j<ncols;j++)
            pathControlsMatrix(i,j) = msg->data[i*ncols+j];

    pathControlsMatrix.conservativeResize(pathControlsMatrix.rows(), pathControlsMatrix.cols()+1);
    pathControlsMatrix.col(0).setZero();
    return;
}

double 
costFunction (unsigned int n, const double* x, double* grad, void* data){
 
    MatrixXd inputs = Map<Matrix<double, num_inputs, mpc_num_samples> > ((double*) x);

    //Trajectory of States 
    MatrixXd traj_n(num_states,mpc_num_samples+1);
    traj_n.setZero(num_states,mpc_num_samples+1);
    
    //Initialize state trajectory
    traj_n.col(0) << uav_q(0), uav_q(1), uav_q(2), uav_q(3);
    
    //cout << "TRAJ:" << traj_n.col(0) << endl;

    //Progate the model (kinematics)
    for (int k = 0; k < mpc_num_samples; k++){

        //cout << "HERE 1" << endl;
        VectorXd ndot = uavModel(inputs.col(k));
        traj_n.col(k+1) = traj_n.col(k) + ndot*dt;
        //traj_n.col(k+1) = uvmsModel(traj_n.col(k), inputs.col(k));
    }
 
 //cout <<"*******************************"<<endl;
 //cout << "STATES Traj:\n" << traj_n<< endl;
  
  //Calculate Running Costs
    double Ji = 0.0;

    for (int k = 0; k < mpc_num_samples; k++) {
       VectorXd ek = traj_n.col(k) - n_des;
       Ji += ek.transpose()* Q* ek;
       Ji += (inputs.col(k)-input_des).transpose()*R* (inputs.col(k)-input_des);
    }
 
    //Calculate Terminal Costs
    double Jt;
    VectorXd et = traj_n.col(mpc_num_samples) - n_des;
    Jt = et.transpose()*P*et;
    return Ji + Jt;
}

void constraints (unsigned int m, double* c, unsigned int n, const double* x, double* grad, void* data) {
    MatrixXd inputs = Map<Matrix<double, num_inputs, mpc_num_samples> > ((double*) x);

    //Trajectory of States 
    MatrixXd traj_n(num_states,mpc_num_samples+1);
    traj_n.setZero(num_states,mpc_num_samples+1);
  
    //Initialize state trajectory
    traj_n.col(0) << uav_q(0), uav_q(1), uav_q(2), uav_q(3);
    
    for (int k = 0; k < mpc_num_samples; k++){
        VectorXd ndot = uavModel( inputs.col(k));
        traj_n.col(k+1) = traj_n.col(k) + ndot*dt;
    }
    for (int i = 0; i < mpc_num_samples + 1; i++){
        VectorXd cons_vector(num_constraints);


        double obs_radious = 80.0;
        double obs1_check = pow(obs_radious,2.0) - pow( traj_n(0,i) - obs1(0), 2.0 ) - pow( traj_n(1,i) - obs1(1), 2.0 ) - pow( traj_n(2,i) - obs1(2), 2.0 ); 
        double obs2_check = pow(obs_radious,2.0) - pow( traj_n(0,i) - obs2(0), 2.0 ) - pow( traj_n(1,i) - obs2(1), 2.0 ) - pow( traj_n(2,i) - obs2(2), 2.0 );
        double obs3_check = pow(obs_radious,2.0) - pow( traj_n(0,i) - obs3(0), 2.0 ) - pow( traj_n(1,i) - obs3(1), 2.0 ) - pow( traj_n(2,i) - obs3(2), 2.0 );
        double obs4_check = pow(obs_radious,2.0) - pow( traj_n(0,i) - obs4(0), 2.0 ) - pow( traj_n(1,i) - obs4(1), 2.0 ) - pow( traj_n(2,i) - obs4(2), 2.0 );
        double obs5_check = pow(obs_radious,2.0) - pow( traj_n(0,i) - obs5(0), 2.0 ) - pow( traj_n(1,i) - obs5(1), 2.0 ) - pow( traj_n(2,i) - obs5(2), 2.0 );
        double obs6_check = pow(obs_radious,2.0) - pow( traj_n(0,i) - obs6(0), 2.0 ) - pow( traj_n(1,i) - obs6(1), 2.0 ) - pow( traj_n(2,i) - obs6(2), 2.0 );
        
        cons_vector << obs1_check, obs2_check, obs3_check, obs4_check, obs5_check, obs6_check;

        for (int j = 0; j < cons_vector.size(); j++){
            c[cons_vector.size()*i+j] = cons_vector(j);
        }
    }
}

VectorXd getPwmCmd(double* input){

    double Rmin = 50.0;
    double Vamax = 25.0;
    double Vamin = 10.0;
    double gammamax = 20.0*M_PI/180.0;
    double gammamin = -20.0*M_PI/180.0;
    double psi_dot_max = Vamax/Rmin;
    double psi_dot_min = -psi_dot_max;

    double Va_dash = (Vamax + Vamin)/2.0;
    double gamma_dash = (gammamax + gammamin)/2.0;
    double psi_dot_dash = (psi_dot_max + psi_dot_min)/2.0;

    double Va_span = Vamax - Vamin;
    double gamma_span = gammamax - gammamin;
    double psi_dot_span = (psi_dot_max - psi_dot_min);

    double Va_pwm = ((input[2] - Va_dash)/(Va_span/2.0))*500.0 + 1500.0;
    double gamma_pwm = ((-input[1] - gamma_dash)/(gamma_span/2.0))*500.0 + 1500.0; 
    double psi_dot_pwm = ((input[0] - psi_dot_dash)/(psi_dot_span/2.0))*500.0 + 1500.0;

    VectorXd pwm;
    pwm.setZero(3);
    pwm(0) = psi_dot_pwm;
    pwm(1) = gamma_pwm;
    pwm(2) = Va_pwm;

    return pwm;
}