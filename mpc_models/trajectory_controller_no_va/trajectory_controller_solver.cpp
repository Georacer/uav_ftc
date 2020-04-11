#include "acado_toolkit.hpp"
#include "acado_code_generation.hpp"
#include "acado_gnuplot.hpp"

int main()
{
  // Use ACADO
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.
  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.
  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
*/

  ///////////////////
  // System variables
  ///////////////////
  DifferentialState alpha, beta, phi, theta;
  IntermediateState psi_dot, gamma;
  Control p, q, r;

  // Specify OnlineData which will be passed in real-time
  // CAUTION!!! This online data order must be reflected exactly in trajectory_controller.hpp
  // Inertial data
  OnlineData m;
  // Other system variables
  OnlineData Va;
  OnlineData Fprop;
  // Geometric data
  OnlineData S;
  OnlineData b;
  OnlineData c;
  // Lift parameters
  OnlineData c_lift_0;
  OnlineData c_lift_a;
  // Drag parameters
  OnlineData c_drag_0;
  OnlineData c_drag_a;
  // Sideforce parameters
  OnlineData c_y_0;
  OnlineData c_y_b;
  // Ellipsoid Flight Envelope parameters
  OnlineData el_A;
  OnlineData el_B;
  OnlineData el_C;
  OnlineData el_D;
  OnlineData el_E;
  OnlineData el_F;
  OnlineData el_G;
  OnlineData el_H;
  OnlineData el_I;
  OnlineData el_J;

  // Parameters which are to be set/overwritten at runtime
  const double t_start = 0.0;      // Initial time [s]
  const double t_end = 4.0;        // Time horizon [s]
  const double dt = 0.2;           // Discretization time [s]
  const int N = round(t_end / dt); // Number of computation nodes
  const double g0 = 9.81;          // Gravity acceleration [m/s^2]
  const double rho = 1.225;        // Air density [kg/m^3]
  const double Fmax = 1.2*g0;      // Maximum motor thrust (N)

  //////////////////////
  // Aircraft parameters
  //////////////////////

  ///////////////
  // System Model
  ///////////////
  // Force calculations
  Expression qbar = 0.5 * rho * Va * Va;
  Expression CLa = c_lift_0 + c_lift_a * alpha;
  Expression FL = qbar * S * CLa;
  Expression FD = qbar * S * (c_drag_0 + c_drag_a * alpha);
  Expression FY = qbar * S * (c_y_0 + c_y_b * beta);

  Expression sa = sin(alpha);
  Expression ca = cos(alpha);
  Expression sb = sin(beta);
  Expression cb = cos(beta);
  Expression sph = sin(phi);
  Expression cph = cos(phi);
  Expression sth = sin(theta);
  Expression cth = cos(theta);
  Expression g1 = g0 * (sa*cb*cph*cth + sb*sph*cth - sth*ca*cb);
  Expression g2 = g0 * (-sa*sb*cth + sb*sth*ca + sph*cb*cth);
  Expression g3 = g0 * (sa*sth + ca*cph*cth);
  Expression qw = -p * sb * ca + q *cb - r*sa * sb;
  Expression rw = -p * sa + r * ca;

  psi_dot = (q*sph + r*cph)/cth;
  // gamma = theta - alpha; Holds only for beta==0 AND phi==0
  gamma = asin((ca*cb)*sth - (sph*sb + cph*sa*cb)*cth); // Stevens-Lewis 2003 3.4-2

  // System dynamics
  DifferentialEquation f;
  f << dot(alpha) == 1 / (m * Va * cb) * (-Fprop * sa - FL + m * Va * qw + m * g3);
  f << dot(beta) == 1 / Va / m * (-Fprop * ca * sb + FY - m * Va * rw + m * g2);
  f << dot(phi) == p + tan(theta) * (q * sph + r * cph);
  f << dot(theta) == q * cph - r * sph;
  // f << 0 == -psi_dot + (q*sph + r*cph)/cth;
  // f << 0 == -gamma + theta - alpha;

  // Flight Envelope expression
  Expression fe_value = el_A*Va*Va
                      + el_B*gamma*gamma
                      + el_C*psi_dot*psi_dot
                      + 2*el_D*Va*gamma
                      + 2*el_E*Va*psi_dot
                      + 2*el_F*gamma*psi_dot
                      + 2*el_G*Va
                      + 2*el_H*gamma
                      + 2*el_I*psi_dot
                      + el_J;

  ////////
  // Costs
  ////////
  Function h, hN;
  // Running cost vector
  h << gamma // Prescribed flight path angle
    << psi_dot // Prescribed turn rate
    << alpha << beta // Minimize alpha and beta
    << p << q << r // Prescribed angular rates
    << phi; // Bank angle for coordinated turn

  // End cost vector, cannot depend on controls, only on state
  hN << gamma // Prescribed flight path angle
     << phi; // Bank angle for coordinated turn
    //  << psi_dot; // Prescribed turn radius, cannot describe it without using inputs

  // For code generation, references are set during run time
  BMatrix Q_sparse(h.getDim(), h.getDim());
  Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim());
  QN_sparse.setIdentity();

  ////////////////////////////////////
  // Define an optimal control problem
  ////////////////////////////////////
  OCP ocp(t_start, t_end, N);

  // Add system dynamics
  ocp.subjectTo(f);

  // Add constraints
  // Input constraints
  ocp.subjectTo(-1.5 <= p <= 1.5); // Constraining p to help with solution feasibility
  ocp.subjectTo(-1.5 <= q <= 1.5); // Constraining q to help with solution feasibility
  ocp.subjectTo(-0.5 <= r <= 0.5); // Constraining r to help with solution feasibility

  // State constraints
  ocp.subjectTo(-45.0*M_PI/180.0 <= alpha <= 45.0*M_PI/180.0); // Stall protection
  ocp.subjectTo(-45.0*M_PI/180.0 <= beta <= 45.0*M_PI/180.0); // Constraining beta to help with solution feasibility 
  ocp.subjectTo(-120.0*M_PI/180.0 <= phi <= 120.0*M_PI/180.0); // Constraining phi to help with solution feasibility
  ocp.subjectTo(-90.0*M_PI/180.0 <= theta <= 90.0*M_PI/180.0); // Constraining theta to help with solution feasibility

  // Flight Envelope constraint
  ocp.subjectTo( fe_value <= 0 );

  // Other feasibility constraints
  // ocp.subjectTo(0 <= phi*psi_dot); // Force a positive-G maneuvera the end of the horizon

  // Set Number of Online Data
  ocp.setNOD(22);

  // For code generation, references are set during run time
  ocp.minimizeLSQ(Q_sparse, h);
  ocp.minimizeLSQEndTerm(QN_sparse, hN);

  OCPexport mpc(ocp);

  ////////////////////////////
  // Configure code generation

  // Setting solution parameters, taken from https://github.com/uzh-rpg/rpg_mpc/blob/master/model/quadrotor_model_thrustrates.cpp
  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);    // is robust, stable
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // good convergence
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); // due to qpOASES
  mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);           // accurate
  mpc.set(NUM_INTEGRATOR_STEPS, N);
  // mpc.set(MAX_NUM_ITERATIONS, 2); // Not a valid option
  mpc.set(QP_SOLVER, QP_QPOASES); // free, source code
  mpc.set(HOTSTART_QP, YES);
  mpc.set(CG_USE_OPENMP, YES);                   // paralellization
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);    // Allow for variable constraints 
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES); // only used for time-varying costs
  mpc.set(USE_SINGLE_PRECISION, YES);            // Single precision

  // Do not generate tests or matlab-related interfaces.
  mpc.set(GENERATE_TEST_FILE, NO);
  mpc.set(GENERATE_MAKE_FILE, YES);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);
  mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

  // Finally, export everything.
  if (mpc.exportCode("trajectory_controller_mpc_solver") != SUCCESSFUL_RETURN)
    exit(EXIT_FAILURE);
  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}
