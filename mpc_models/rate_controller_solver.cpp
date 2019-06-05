#include "acado_optimal_control.hpp"
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

  const bool CODE_GEN = false;

  // System variables
  DifferentialState p, q, r;
  Control da, de, dr;
  DifferentialEquation f;
  Function h, hN;
  OnlineData Va, alpha, beta;

  // Parameters which are to be set/overwritten at runtime
  const double t_start = 0.0;      // Initial time [s]
  const double t_end = 2.0;        // Time horizon [s]
  const double dt = 0.01;          // Discretization time [s]
  const int N = round(t_end / dt); // Number of computation nodes
  const double g0 = 9.81;          // Gravity acceleration [m/s^2]
  const double rho = 1.225;        // Air density [kg/m^3]

  //////////////////////
  // Aircraft parameters
  //////////////////////
  double S = 0.45; // Wing area
  double b = 1.88; // Wingspan
  double c = 0.24; // Mean chord
  // Inertial charateristics
  double j_x = 0.8244;
  double j_y = 1.135;
  double j_z = 1.759;
  double j_xz = 0.1204;
  double G = j_x * j_z - j_xz * j_xz;
  double G1 = j_xz * (j_x - j_y + j_z) / G;
  double G2 = (j_z * (j_z - j_y) + j_xz * j_xz) / G;
  double G3 = j_z / G;
  double G4 = j_xz / G;
  double G5 = (j_z - j_x) / j_y;
  double G6 = j_xz / j_y;
  double G7 = ((j_y - j_y) * j_x + j_xz * j_xz) / G;
  double G8 = j_x / G;
  // roll moment parameters
  double c_l_0 = 0;
  double c_l_p = -1;
  double c_l_b = -0.12;
  double c_l_r = 0.14;
  double c_l_deltaa = 0.25;
  double c_l_deltar = -0.037;
  // Pitch moment parameters
  double c_m_0 = 0.045;
  double c_m_a = -0.7;
  double c_m_q = -20;
  double c_m_deltae = 1.0;
  // Yaw moment parameters
  double c_n_0 = 0;
  double c_n_b = 0.25;
  double c_n_p = 0.022;
  double c_n_r = -1;
  double c_n_deltaa = 0.00;
  double c_n_deltar = 0.1; // opposite sign than c_y_deltar;
  // Input parameters
  double deltaa_max = 0.3491;
  double deltae_max = 0.3491;
  double deltar_max = 0.3491;

  ///////////////
  // System Model
  ///////////////
  // Torque calculations
  IntermediateState l = 0.5 * rho * S * b * (c_l_0 + c_l_b * beta + b / 2 / Va * (c_l_p * p + c_l_r * r) + c_l_deltaa * da + c_l_deltar * dr);
  IntermediateState m = 0.5 * rho * S * c * (c_m_0 + c_m_a * alpha + c / 2 / Va * (c_m_q * q) + c_m_deltae * de);
  IntermediateState n = 0.5 * rho * S * b * (c_n_0 + c_n_b * beta + b / 2 / Va * (c_n_p * p + c_n_r * r) + c_n_deltaa * da + c_n_deltar * dr);

  // System dynamics
  f << dot(p) == G1 * p * q - G2 * q * r + G3 * l + G4 * n;
  f << dot(q) == G5 * p * r - G6 * (p * p - r * r) + m / j_y;
  f << dot(r) == G7 * p * q - G1 * q * r + G4 * l + G8 * n;

  //  // Explicit model without using IntermediateStates
  // f << dot(p) == G1 * p * q - G2 * q * r + G3 * (0.5 * rho * S * b * (c_l_0 + c_l_b * beta + b / 2 / Va * (c_l_p * p + c_l_r * r) + c_l_deltaa * da + c_l_deltar * dr)) + G4 * (0.5 * rho * S * b * (c_n_0 + c_n_b * beta + b / 2 / Va * (c_n_p * p + c_n_r * r) + c_n_deltaa * da + c_n_deltar * dr));
  // f << dot(q) == G5 * p * r - G6 * (p * p - r * r) + (0.5 * rho * S * c * (c_m_0 + c_m_a * alpha + c / 2 / Va * (c_m_q * q) + c_m_deltae * de)) / j_y;
  // f << dot(r) == G7 * p * q - G1 * q * r + G4 * (0.5 * rho * S * b * (c_l_0 + c_l_b * beta + b / 2 / Va * (c_l_p * p + c_l_r * r) + c_l_deltaa * da + c_l_deltar * dr)) + G8 * (0.5 * rho * S * b * (c_n_0 + c_n_b * beta + b / 2 / Va * (c_n_p * p + c_n_r * r) + c_n_deltaa * da + c_n_deltar * dr));

  ////////
  // Costs
  ////////
  // Running cost vector
  h << p << q << r
    << da << de << dr;

  // End cost vector
  hN << p << q << r;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0, 0) = 100; // p
  Q(1, 1) = 100; // q
  Q(2, 2) = 100; // r
  Q(3, 3) = 100; // da
  Q(4, 4) = 100; // de
  Q(5, 5) = 100; // dr

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0, 0) = Q(0, 0);
  QN(1, 1) = Q(1, 1);
  QN(2, 2) = Q(2, 2);

  // Set a reference for the analysis, if CODE_GEN == false
  DVector ref(h.getDim());
  ref.setZero();
  ref(0) = 0;
  ref(1) = 0;
  ref(2) = 0.2; // Yaw rate required at 0.2rad/s

  DVector refN(hN.getDim());
  refN.setZero();
  refN(0) = ref(0);
  refN(1) = ref(1);
  refN(2) = ref(2);

  ////////////////////////////////////
  // Define an optimal control problem
  ////////////////////////////////////
  OCP ocp(t_start, t_end, N);
  if (!CODE_GEN)
  {
    // For analysis, set references
    ocp.minimizeLSQ(Q, h, ref);
    ocp.minimizeLSQEndTerm(QN, hN, refN);
  }
  else
  {
    // For code generation, references are set during run time
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN);
  }

  // Add system dynamics
  ocp.subjectTo(f);
  // Add constraints
  ocp.subjectTo(-deltaa_max <= da <= deltaa_max);
  ocp.subjectTo(-deltae_max <= de <= deltae_max);
  ocp.subjectTo(-deltar_max <= dr <= deltar_max);
  // Set Number of Online Data
  ocp.setNOD(3);

  if (!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo(AT_START, p == 0);
    ocp.subjectTo(AT_START, q == 0);
    ocp.subjectTo(AT_START, r == 0);
    // Setup visualizations
    GnuplotWindow window1(PLOT_AT_EACH_ITERATION);
    window1.addSubplot(p, "roll rate (rad/s)");
    window1.addSubplot(q, "pitch rate (rad/s)");
    window1.addSubplot(r, "yaw rate (rad/s)");
    GnuplotWindow window2(PLOT_AT_EACH_ITERATION);
    window1.addSubplot(da, "aileron input (rad)");
    window1.addSubplot(de, "elevator input (rad)");
    window1.addSubplot(dr, "rudder input (rad)");

    // Define an algorithm to solve the problem
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
    algorithm.set(KKT_TOLERANCE, 1e-3);
    algorithm << window1;
    algorithm << window2;
    algorithm.solve();
  }
  else
  {
    OCPexport mpc(ocp);

    // Setting solution parameters, taken from https://github.com/uzh-rpg/rpg_mpc/blob/master/model/quadrotor_model_thrustrates.cpp
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);    // is robust, stable
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // good convergence
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); // due to qpOASES
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);           // accurate
    mpc.set(NUM_INTEGRATOR_STEPS, N);
    mpc.set(QP_SOLVER, QP_QPOASES); // free, source code
    mpc.set(HOTSTART_QP, YES);
    mpc.set(CG_USE_OPENMP, YES);                    // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);     // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES); // time-varying costs
    mpc.set(USE_SINGLE_PRECISION, YES);             // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if (mpc.exportCode("uav_ftc_codegen") != SUCCESSFUL_RETURN)
      exit(EXIT_FAILURE);
    mpc.printDimensionsQP();
  }
  return EXIT_SUCCESS;
}
