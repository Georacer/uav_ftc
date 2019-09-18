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

  const bool CODE_GEN = true;

  ///////////////////
  // System variables
  ///////////////////
  DifferentialState p, q, r;
  Control da, de, dr;

  // Specify OnlineData which will be passed in real-time
  OnlineData Va;
  OnlineData alpha;
  OnlineData beta;
  OnlineData phi;
  OnlineData theta;

  // Specify constraint coefficients
  // TODO: Generate valid data for here
  const double  a0[7] = {    -555.9,      389.0,      -13.8,     -366.7,    15421.6,     2325.4,      227.2};
  const double  a1[7] = {   -1540.8,    -1525.8,        4.6,    -7455.4,   -42658.9,    31082.0,      -85.0};
  const double  a2[7] = {    -144.5,      382.3,        1.0,     1023.0,     6942.2,      -75.6,     -322.7};
  const double  a3[7] = {    -407.0,      283.9,       10.7,    -1075.4,    16029.2,      314.2,       12.4};
  const double  a4[7] = {  -14728.9,    22378.3,     -477.8,   -32943.2,   693936.5,    72663.7,    14000.7};
  const double  a5[7] = {     -96.4,       10.2,       -1.7,      206.6,    -3595.9,     2303.0,     -176.0};
  const double  a6[7] = {    -489.3,     1328.9,      -25.9,     1518.0,    25999.6,     2418.1,      -58.5};
  const double  a7[7] = {   -3516.4,     3414.9,     -132.4,    -2750.0,   111831.8,    17995.6,     2334.6};
  const double  a8[7] = {    -180.9,       44.4,       -5.2,     -243.8,     1858.7,     1670.4,       66.4};
  const double  a9[7] = {     148.0,     -541.1,        1.6,     -212.2,   -12516.1,     1861.2,     -167.1};
  const double a10[7] = {     856.8,     -889.2,       27.9,     1047.0,   -36136.0,    -5201.4,     -693.6};
  const double a11[7] = {   -1679.8,     1182.2,     -140.0,    -1831.2,    76800.5,    14506.5,     1795.9};
  const double a12[7] = {    7518.7,    -7430.2,      238.4,     9662.8,  -309206.8,   -48255.2,    -5978.3};
  const double a13[7] = {    1591.2,    -1048.6,       24.9,     2533.5,   -33560.4,    -5964.2,    -1097.5};
  const double a14[7] = {   15182.3,   -14721.4,      484.3,    19657.6,  -612794.7,   -96307.6,   -12074.8};
  const double a15[7] = {     149.5,    -1702.7,        1.5,      249.9,    -4268.0,     -232.8,     -692.1};
  const double a16[7] = {    6127.2,    -6200.8,      202.3,     7849.7,  -252505.7,   -39599.3,    -4987.8};
  const double a17[7] = {     388.6,     -410.7,       12.9,      448.7,   -16599.7,    -2284.0,     -316.7};
  const double a18[7] = {     565.2,    -2162.5,       32.9,    -2669.2,   -57803.7,     9899.8,     -764.3};
  const double a19[7] = {    3347.2,    -3081.7,       92.3,     5035.8,   -85889.1,   -15290.4,    -2846.8};
  const double a20[7] = {     298.2,     -295.4,        9.8,      384.7,   -12098.3,    -1905.7,     -241.3};
  const double a21[7] = {   -1057.7,       11.4,      122.8,    -6460.4,    63529.9,    -5429.9,     -965.1};
  const double a22[7] = {    2688.0,    -2746.2,      100.3,     3072.8,  -109396.0,   -18282.7,    -2301.5};
  const double a23[7] = {    -113.9,      189.1,       -3.7,     -284.4,     5566.8,      520.2,      113.2};
  const double a24[7] = {   -3559.5,     5426.0,     -115.8,    -8021.4,   168468.0,    17597.4,     3399.7};
  const double a25[7] = {    2764.4,    -2822.0,       90.0,     3418.7,  -115100.3,   -16834.4,    -2231.6};
  const double a26[7] = {     107.7,     -134.3,        2.7,      100.3,    -3989.0,     -140.3,      -89.5};


  // Parameters which are to be set/overwritten at runtime
  const double t_start = 0.0;      // Initial time [s]
  const double t_end = 0.4;        // Time horizon [s]
  const double dt = 0.02;          // Discretization time [s]
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
  double G7 = ((j_x - j_y) * j_x + j_xz * j_xz) / G;
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
  DifferentialEquation f;
  // Torque calculations
  Expression qbar = 0.5 * rho * Va * Va;
  IntermediateState l = qbar * S * b * (c_l_0 + c_l_b * beta + b / 2 / Va * (c_l_p * p + c_l_r * r) + c_l_deltaa * da + c_l_deltar * dr);
  IntermediateState m = qbar * S * c * (c_m_0 + c_m_a * alpha + c / 2 / Va * (c_m_q * q) + c_m_deltae * de);
  IntermediateState n = qbar * S * b * (c_n_0 + c_n_b * beta + b / 2 / Va * (c_n_p * p + c_n_r * r) + c_n_deltaa * da + c_n_deltar * dr);

  // System dynamics, Beard notation
  f << dot(p) == G1 * p * q - G2 * q * r + G3 * l + G4 * n;
  f << dot(q) == G5 * p * r - G6 * (p * p - r * r) + m / j_y;
  f << dot(r) == G7 * p * q - G1 * q * r + G4 * l + G8 * n;

  ////////
  // Costs
  ////////
  Function h, hN;
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
  Q(3, 3) = 1;   // da
  Q(4, 4) = 1;   // de
  Q(5, 5) = 1;   // dr

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0, 0) = Q(0, 0);
  QN(1, 1) = Q(1, 1);
  QN(2, 2) = Q(2, 2);

  ////////////////////////////////////
  // Define an optimal control problem
  ////////////////////////////////////
  OCP ocp(t_start, t_end, N);
  // Add system dynamics
  ocp.subjectTo(f);
  // Add constraints
  ocp.subjectTo(-deltaa_max <= da <= deltaa_max);
  ocp.subjectTo(-deltae_max <= de <= deltae_max);
  ocp.subjectTo(-deltar_max <= dr <= deltar_max);

  // Add polytope state constraints
  ocp.subjectTo(-INFTY <=  a0[0]*phi +  a0[1]*theta +  a0[2]*Va +  a0[3]*alpha +  a0[4]*beta +  a0[5]*r +  a0[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a1[0]*phi +  a1[1]*theta +  a1[2]*Va +  a1[3]*alpha +  a1[4]*beta +  a1[5]*r +  a1[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a2[0]*phi +  a2[1]*theta +  a2[2]*Va +  a2[3]*alpha +  a2[4]*beta +  a2[5]*r +  a2[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a3[0]*phi +  a3[1]*theta +  a3[2]*Va +  a3[3]*alpha +  a3[4]*beta +  a3[5]*r +  a3[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a4[0]*phi +  a4[1]*theta +  a4[2]*Va +  a4[3]*alpha +  a4[4]*beta +  a4[5]*r +  a4[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a5[0]*phi +  a5[1]*theta +  a5[2]*Va +  a5[3]*alpha +  a5[4]*beta +  a5[5]*r +  a5[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a6[0]*phi +  a6[1]*theta +  a6[2]*Va +  a6[3]*alpha +  a6[4]*beta +  a6[5]*r +  a6[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a7[0]*phi +  a7[1]*theta +  a7[2]*Va +  a7[3]*alpha +  a7[4]*beta +  a7[5]*r +  a7[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a8[0]*phi +  a8[1]*theta +  a8[2]*Va +  a8[3]*alpha +  a8[4]*beta +  a8[5]*r +  a8[6] <= 0 );
  ocp.subjectTo(-INFTY <=  a9[0]*phi +  a9[1]*theta +  a9[2]*Va +  a9[3]*alpha +  a9[4]*beta +  a9[5]*r +  a9[6] <= 0 );
  ocp.subjectTo(-INFTY <= a10[0]*phi + a10[1]*theta + a10[2]*Va + a10[3]*alpha + a10[4]*beta + a10[5]*r + a10[6] <= 0 );
  ocp.subjectTo(-INFTY <= a11[0]*phi + a11[1]*theta + a11[2]*Va + a11[3]*alpha + a11[4]*beta + a11[5]*r + a11[6] <= 0 );
  ocp.subjectTo(-INFTY <= a12[0]*phi + a12[1]*theta + a12[2]*Va + a12[3]*alpha + a12[4]*beta + a12[5]*r + a12[6] <= 0 );
  ocp.subjectTo(-INFTY <= a13[0]*phi + a13[1]*theta + a13[2]*Va + a13[3]*alpha + a13[4]*beta + a13[5]*r + a13[6] <= 0 );
  ocp.subjectTo(-INFTY <= a14[0]*phi + a14[1]*theta + a14[2]*Va + a14[3]*alpha + a14[4]*beta + a14[5]*r + a14[6] <= 0 );
  ocp.subjectTo(-INFTY <= a15[0]*phi + a15[1]*theta + a15[2]*Va + a15[3]*alpha + a15[4]*beta + a15[5]*r + a15[6] <= 0 );
  ocp.subjectTo(-INFTY <= a16[0]*phi + a16[1]*theta + a16[2]*Va + a16[3]*alpha + a16[4]*beta + a16[5]*r + a16[6] <= 0 );
  ocp.subjectTo(-INFTY <= a17[0]*phi + a17[1]*theta + a17[2]*Va + a17[3]*alpha + a17[4]*beta + a17[5]*r + a17[6] <= 0 );
  ocp.subjectTo(-INFTY <= a18[0]*phi + a18[1]*theta + a18[2]*Va + a18[3]*alpha + a18[4]*beta + a18[5]*r + a18[6] <= 0 );
  ocp.subjectTo(-INFTY <= a19[0]*phi + a19[1]*theta + a19[2]*Va + a19[3]*alpha + a19[4]*beta + a19[5]*r + a19[6] <= 0 );
  ocp.subjectTo(-INFTY <= a20[0]*phi + a20[1]*theta + a20[2]*Va + a20[3]*alpha + a20[4]*beta + a20[5]*r + a20[6] <= 0 );
  ocp.subjectTo(-INFTY <= a21[0]*phi + a21[1]*theta + a21[2]*Va + a21[3]*alpha + a21[4]*beta + a21[5]*r + a21[6] <= 0 );
  ocp.subjectTo(-INFTY <= a22[0]*phi + a22[1]*theta + a22[2]*Va + a22[3]*alpha + a22[4]*beta + a22[5]*r + a22[6] <= 0 );
  ocp.subjectTo(-INFTY <= a23[0]*phi + a23[1]*theta + a23[2]*Va + a23[3]*alpha + a23[4]*beta + a23[5]*r + a23[6] <= 0 );
  ocp.subjectTo(-INFTY <= a24[0]*phi + a24[1]*theta + a24[2]*Va + a24[3]*alpha + a24[4]*beta + a24[5]*r + a24[6] <= 0 );
  ocp.subjectTo(-INFTY <= a25[0]*phi + a25[1]*theta + a25[2]*Va + a25[3]*alpha + a25[4]*beta + a25[5]*r + a25[6] <= 0 );
  ocp.subjectTo(-INFTY <= a26[0]*phi + a26[1]*theta + a26[2]*Va + a26[3]*alpha + a26[4]*beta + a26[5]*r + a26[6] <= 0 );


  // Set Number of Online Data
  ocp.setNOD(5);

  if (!CODE_GEN)
  {
    // Setting the values of online data does not seem to work
    // ocp.subjectTo(Va == 15);
    // ocp.subjectTo(alpha == 0.035);
    // ocp.subjectTo(beta == 0);

    // Set a reference for the analysis
    DVector ref(h.getDim());
    ref.setZero();
    // Set all angular rates to 0
    ref(0) = 1;
    ref(1) = 1;
    ref(2) = 1;

    DVector refN(hN.getDim());
    refN.setZero();
    refN(0) = ref(0);
    refN(1) = ref(1);
    refN(2) = ref(2);

    // Set the references into the problem
    ocp.minimizeLSQ(Q, h, ref);
    ocp.minimizeLSQEndTerm(QN, hN, refN);
  }
  else
  {
    // For code generation, references are set during run time
    ocp.minimizeLSQ(Q, h);
    ocp.minimizeLSQEndTerm(QN, hN);
  }

  ////////////////////
  // System simulation
  ////////////////////
  if (!CODE_GEN)
  {
    // Setup the simulated process
    OutputFcn identity;
    DynamicSystem dynamicSystem(f, identity);
    Process process(dynamicSystem, INT_RK45);

    // Setup the MPC
    RealTimeAlgorithm alg(ocp, dt);
    alg.set(MAX_NUM_ITERATIONS, 5);
    StaticReferenceTrajectory reference;
    Controller controller(alg, reference);

    // Setup simulation environment
    SimulationEnvironment sim(t_start, t_end, process, controller);
    // Set initial conditions
    DVector x0(3);
    x0(0) = 0.3;
    x0(1) = 0.2;
    x0(2) = 0.1;

    if (sim.init(x0) != SUCCESSFUL_RETURN)
    {
      exit(EXIT_FAILURE);
    }
    if (sim.run() != SUCCESSFUL_RETURN)
    {
      exit(EXIT_FAILURE);
    }

    // Plot the results at the end of the simulation
    VariablesGrid sampledProcessOutput;
    sim.getSampledProcessOutput(sampledProcessOutput);
    VariablesGrid feedbackControl;
    sim.getFeedbackControl(feedbackControl);
    GnuplotWindow window;
    window.addSubplot(sampledProcessOutput(0), "p");
    window.addSubplot(sampledProcessOutput(1), "q");
    window.addSubplot(sampledProcessOutput(2), "r");
    window.addSubplot(feedbackControl(0), "da");
    window.addSubplot(feedbackControl(1), "de");
    window.addSubplot(feedbackControl(2), "dr");
    window.plot();
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
    // mpc.set(MAX_NUM_ITERATIONS, 2); // Not a valid option
    mpc.set(QP_SOLVER, QP_QPOASES); // free, source code
    mpc.set(HOTSTART_QP, YES);
    mpc.set(CG_USE_OPENMP, YES);                   // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);   // Leave this to yes for compatibility with uav_ftc/mpc_wrapper
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, NO); // only used for time-varying costs
    mpc.set(USE_SINGLE_PRECISION, YES);            // Single precision

    // Do not generate tests or matlab-related interfaces.
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, YES);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if (mpc.exportCode("rate_controller_mpc_solver_2") != SUCCESSFUL_RETURN)
      exit(EXIT_FAILURE);
    mpc.printDimensionsQP();
  }
  return EXIT_SUCCESS;
}
