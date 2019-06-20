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
  DifferentialState Va, alpha, beta, phi, theta;
  IntermediateState psi_dot, gamma;
  // AlgebraicState psi_dot, gamma; // Simulation cannot handle DAEs
  Control p, q, r, deltat;

  // Parameters which are to be set/overwritten at runtime
  const double t_start = 0.0;      // Initial time [s]
  const double t_end = 4.0;        // Time horizon [s]
  const double dt = 0.2;          // Discretization time [s]
  const int N = round(t_end / dt); // Number of computation nodes
  const double g0 = 9.81;          // Gravity acceleration [m/s^2]
  const double rho = 1.225;        // Air density [kg/m^3]

  // Desired trajectory parameters, used for simulation
  double R = 100; // turn radius in meters
  double gamma_d = 0.1; // fligh path angle in radians
  double Va_d = 15;

  //////////////////////
  // Aircraft parameters
  //////////////////////
  double S = 0.45; // Wing area
  double b = 1.88; // Wingspan
  double c = 0.24; // Mean chord
  // Inertial charateristics
  double m = 2.0;
  // Lift parameters
  double c_lift_0 = 0.56;
  double c_lift_a = 6.9;
  // Drag parameters
  double c_drag_p = 0.1;
  double oswald = 0.9;
  // Sideforce parameters
  double c_y_0 = 0;
  double c_y_b = -0.98;
  // Propulsion parameters
  double k_p = 0.33;
  double k_m = 30;
  // Input parameters
  double deltat_max = 1.0;

  ///////////////
  // System Model
  ///////////////
  // Force calculations
  Expression qbar = 0.5 * rho * Va * Va;
  Expression CLa = c_lift_0 + c_lift_a * alpha;
  Expression FL = qbar * S * CLa;
  Expression FD = qbar * S * (c_drag_p + CLa * CLa / (M_PI * oswald * b * b / S));
  Expression FY = qbar * S * (c_y_0 + c_y_b * beta);
  Expression Fprop = 0.5 * rho * k_p * (k_m * deltat * k_m * deltat - Va * Va);
  // IntermediateState l = qbar * S * b * (c_l_0 + c_l_b * beta + b / 2 / Va * (c_l_p * p + c_l_r * r) + c_l_deltaa * da + c_l_deltar * dr);
  // IntermediateState m = qbar * S * c * (c_m_0 + c_m_a * alpha + c / 2 / Va * (c_m_q * q) + c_m_deltae * de);
  // IntermediateState n = qbar * S * b * (c_n_0 + c_n_b * beta + b / 2 / Va * (c_n_p * p + c_n_r * r) + c_n_deltaa * da + c_n_deltar * dr);
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
  gamma = theta - alpha;

  // System dynamics
  DifferentialEquation f;
  f << dot(Va) == Fprop * ca * cb / m - FD / m + g1;
  f << dot(alpha) == 1 / (m * Va * cb) * (-Fprop * sa - FL + m * Va * qw + m * g3);
  f << dot(beta) == 1 / Va / m * (-Fprop * ca * sb + FY - m * Va * rw + m * g2);
  f << dot(phi) == p + tan(theta) * (q * sph + r * cph);
  f << dot(theta) == q * cph - r * sph;
  // f << 0 == -psi_dot + (q*sph + r*cph)/cth;
  // f << 0 == -gamma + theta - alpha;

  ////////
  // Costs
  ////////
  Function h, hN;
  // Running cost vector
  h << Va // Prescribed airspeed
    << gamma // Prescribed flight path angle
    << psi_dot // Prescribed turn rate
    << alpha << beta // Minimize alpha and beta
    << p << q << r // Prescribed angular rates
    << deltat; // Minimize throttle

  // End cost vector, cannot depend on controls, only on state
  hN << Va // Presribed airspeed 
     << gamma; // Prescribed flight path angle
    //  << psi_dot; // Prescribed turn radius, cannot describe it without using inputs

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0, 0) = 100; // Va
  Q(1, 1) = 100; // Flight path angle
  Q(2, 2) = 100; // turn rate
  Q(3, 3) = 1;  // alpha
  Q(4, 4) = 1;  // beta
  Q(5, 5) = 1;   // p
  Q(6, 6) = 1;   // q
  Q(7, 7) = 1;   // r
  Q(8, 8) = 1;   // throttle

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0, 0) = Q(0, 0);
  QN(1, 1) = Q(1, 1);
  // QN(2, 2) = Q(2, 2);

  ////////////////////////////////////
  // Define an optimal control problem
  ////////////////////////////////////
  OCP ocp(t_start, t_end, N);
  // Add system dynamics
  ocp.subjectTo(f);
  // Add constraints
  ocp.subjectTo(0.0 <= deltat <= deltat_max);
  ocp.subjectTo(-5.0*M_PI/180.0 <= alpha <= 15.0*M_PI/180.0); // Stall protection
  ocp.subjectTo(-15.0*M_PI/180.0 <= beta <= 15.0*M_PI/180.0); // Constraining beta to help with solution feasibility 
  ocp.subjectTo(-30.0*M_PI/180.0 <= theta <= 45.0*M_PI/180.0); // Constraining theta to help with solution feasibility
  // Set Number of Online Data
  // ocp.setNOD(3); // No online data for this model
  if (!CODE_GEN)
  {
    // Set a reference for the analysis
    DVector ref(h.getDim());
    ref.setZero();
    // Calculation of reference state for no-wind coordinated turn
    double psi_dot_d = Va_d/R*cos(gamma_d);
    ref(0) = Va_d;
    ref(1) = gamma_d;
    ref(2) = psi_dot_d;
    ref(3) = 0;
    ref(4) = 0;
    ref(5) = 0;
    ref(6) = 0;
    ref(7) = 0;
    ref(8) = 0;

    DVector refN(hN.getDim());
    refN.setZero();
    refN(0) = ref(0);
    refN(1) = ref(1);
    // refN(2) = ref(2);

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

    // Set a reference for the analysis
    DVector ref_state(5);
    ref_state.setZero();
    // Calculation of reference state for no-wind coordinated turn
    double psi_dot_d = Va_d/R*cos(gamma_d);
    ref_state(0) = Va_d;
    ref_state(1) = gamma_d;
    ref_state(2) = psi_dot_d;
    ref_state(3) = 0;
    ref_state(4) = 0;
    DVector ref_inputs(4);
    ref_inputs.setZero();
    StaticReferenceTrajectory reference;
    reference.init(0.0, ref_state, emptyConstVector, ref_inputs);

    Controller controller(alg, reference);

    // Setup simulation environment
    SimulationEnvironment sim(t_start, t_end, process, controller);
    // Set initial conditions
    DVector x0(5);
    x0(0) = 13.0;
    x0(1) = 0.0;
    x0(2) = 0.0;
    x0(3) = 0.0;
    x0(4) = 0.0;

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
    std::cout << "Output states num rows: " << sampledProcessOutput.getNumRows() << std::endl;
    std::cout << "Output states num cols: " << sampledProcessOutput.getNumCols() << std::endl;
    VariablesGrid feedbackControl;
    sim.getFeedbackControl(feedbackControl);
    std::cout << "Control inputs num rows: " << feedbackControl.getNumRows() << std::endl;
    std::cout << "Control inputs num cols: " << feedbackControl.getNumCols() << std::endl;
    VariablesGrid intermediateStates;
    sim.getProcessIntermediateStates(intermediateStates);
    std::cout << "Intermediate states num rows: " << intermediateStates.getNumRows() << std::endl;
    std::cout << "Intermediate states num cols: " << intermediateStates.getNumCols() << std::endl;
    // VariablesGrid algebraicStates;
    // sim.getProcessAlgebraicStates(algebraicStates);

    GnuplotWindow window;
    // Plot states
    window.addSubplot(sampledProcessOutput(0), "Va");
    window.addSubplot(sampledProcessOutput(1), "alpha");
    window.addSubplot(sampledProcessOutput(2), "beta");
    window.addSubplot(sampledProcessOutput(3), "phi");
    window.addSubplot(sampledProcessOutput(4), "theta");
    // Plot control inputs
    window.addSubplot(feedbackControl(0), "p");
    window.addSubplot(feedbackControl(1), "q");
    window.addSubplot(feedbackControl(2), "r");
    window.addSubplot(feedbackControl(3), "deltat");
    window.plot();

    // Plot intermediate states
    GnuplotWindow window2;
    window2.addSubplot(intermediateStates(0), "i-state 0");
    window2.addSubplot(intermediateStates(1), "i-state 1");
    window2.addSubplot(intermediateStates(2), "i-state 2");
    window2.addSubplot(intermediateStates(3), "i-state 3");
    window2.addSubplot(intermediateStates(4), "i-state 4");
    window2.addSubplot(intermediateStates(5), "i-state 5");
    window2.addSubplot(intermediateStates(6), "i-state 6");
    window2.addSubplot(intermediateStates(7), "i-state 7");
    window2.addSubplot(intermediateStates(8), "i-state 8");
    window2.addSubplot(intermediateStates(9), "i-state 9");
    window2.plot();

    // Print functions contents
    std::cout << "Rolling cost contents: \n" << std::endl;
    h.print(std::cout);
    std::cout << "Final cost contents: \n" << std::endl;
    hN.print(std::cout);
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
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, YES);   // Currently we do no plan to alter the constraints
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, NO); // only used for time-varying costs
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
  }
  return EXIT_SUCCESS;
}
