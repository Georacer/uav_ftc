/**
 * @file mpc_wrapper.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Wrapper library for the rate controller MPC library
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "mpc_wrapper.hpp"

/**
 * @brief Construct a new Rate Mpc Wrapper< T>:: Rate Mpc Wrapper object
 * 
 * @tparam T 
 */
template <typename T>
MpcWrapper<T>::MpcWrapper(T dt, int numConstraints) : 
    #ifdef ACADO_HAS_ONLINEDATA
    // acado_online_data_(acadoVariables.od, kOdSize, kSamples+1, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(1,kOdSize)),
    acado_online_data_(acadoVariables.od, kOdSize, kSamples+1),
    #else
    acado_online_data_(&dummy_od_, 1, 1),
    #endif

    #ifdef ACADO_HAS_CONSTRAINTS
    acado_lower_bounds_(acadoVariables.lbValues, numConstraints, kSamples),
    acado_upper_bounds_(acadoVariables.ubValues, numConstraints, kSamples),
    #else
    acado_lower_bounds_(&dummy_bounds_, 1, 1),
    acado_upper_bounds_(&dummy_bounds_, 1, 1),
    #endif

    dummy_var_(0)
{
    #ifdef ACADO_HAS_ONLINEDATA
    ROS_INFO("Raised MPC node with online data enabled");
    #else
    ROS_INFO("Raised MPC node with online data disabled");
    #endif
    #ifdef ACADO_HAS_CONSTRAINTS
    ROS_INFO("Raised MPC node with constraints enabled");
    #else
    ROS_INFO("Raised MPC node with constraints disabled");
    #endif


    dt_ = dt;
    kConstraintSize_ = numConstraints;

    // Sanity check for corrent passing of bound values
    // std::cout << "Upper limits:\n" << acado_upper_bounds_ << std::endl;
    // std::cout << "Lower limits:\n" << acado_lower_bounds_ << std::endl;
    resetController();

    ROS_INFO("Raised an MPC with attributes:");
    ROS_INFO("# States: %d", kStateSize);
    ROS_INFO("# Inputs: %d", kInputSize);
    ROS_INFO("# OnlineData: %d", kOdSize);
    ROS_INFO("# Running reference states: %d", kRefSize);
    ROS_INFO("# End reference states: %d", kEndRefSize);
}

template <typename T>
bool MpcWrapper<T>::setTrimState(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
    kTrimState_ = state;
    return true;
}

template <typename T>
bool MpcWrapper<T>::setTrimInput(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input)
{
    kTrimInput_ = input;
    return true;
}

template <typename T>
bool MpcWrapper<T>::setTrimOnlineData(const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> onlineData)
{
    kTrimOnlineData_ = onlineData;
    return true;
}

template <typename T>
bool MpcWrapper<T>::setDefaultRunningReference(const Eigen::Ref<const Eigen::Matrix<T, kRefSize, 1>> reference)
{
    defaultReference_ = reference;
    return true;
}

template <typename T>
bool MpcWrapper<T>::setDefaultEndReference(const Eigen::Ref<const Eigen::Matrix<T, kEndRefSize, 1>> endReference)
{
    defaultEndReference_ = endReference;
    return true;
}

/**
 * @brief Set the online data used by the model
 * 
 * @tparam T float or double
 * @param online_data 
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::setOnlineData(
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data)
{
    // Copy over online data
    acado_online_data_.block(0, 0, kOdSize, ACADO_N + 1) = online_data.replicate(1, ACADO_N + 1).template cast<float>();

    return true;
}

/**
 * @brief Set one item of the Online Data array, for all the time horizon
 * 
 * @tparam T Data type
 * @param singleData The data value
 * @param index The data index in the OnlineData array (referring to the solver object)
 * @return true Successfully set the data
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::setOnlineDataSingle(const uint index, const T singleData)
{
    // Copy over online data
    acado_online_data_.block(index, 0, 1, ACADO_N + 1) = Eigen::MatrixXf::Constant(1, ACADO_N+1, singleData);

    return true;
}

/**
 * @brief Set the initial state of the system under optimization, essentially the measurement
 * 
 * @tparam T float or double
 * @param state The initial system state
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::setInitialState(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
    acado_initial_state_ = state.template cast<float>();

    return true;
}

/**
 * @brief Set the target pose the sysetm should achieve at the end of the horizon
 * 
 * @tparam T float or double
 * @param state The target values each state should achieve
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::setReference(
    const Eigen::Ref<const Eigen::Matrix<T, kRefSize, 1>> reference,
    const Eigen::Ref<const Eigen::Matrix<T, kEndRefSize, 1>> referenceEnd)
{
    // acado_reference_states_.block(0, 0, kRefSize, kSamples) =
    acado_reference_states_ =
        reference.replicate(1, kSamples).template cast<float>();

    // acado_reference_end_state_.segment(0, kEndRefSize) =
    acado_reference_end_state_ =
        referenceEnd.template cast<float>();
        
    return true;
}

/**
 * @brief The target trajectory the system should follow within the prediction horizon
 * 
 * @tparam T float or double
 * @param states One state vector for each time instance in the horizon plus the final state
 * @param inputs One input vector for each time instance in the horizon plus the final state
 * @return true 
 * @return false 
 */
template <typename T>
// WARNING!!! DOES NOT WORK CURRENTLY
bool MpcWrapper<T>::setReferenceTrajectory(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs)
{
    Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
    y(const_cast<float *>(acadoVariables.y));

    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        states.block(0, 0, kStateSize, kSamples).template cast<float>();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

    acado_reference_end_state_.segment(0, kStateSize) =
        states.col(kSamples).template cast<float>();

    return true;
}

/**
 * @brief Perform one step of the MPC
 * Initialize the first-pass estimate of the solution before searching for a solution
 * 
 * @tparam T float or double 
 * @param state The first-pass guess for the optimal state before optimizing the system
 * @param online_data The currently measured vector of airspeed, AoA, AoS
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::solve(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data)
{
    // Reset states and inputs
    if (controller_is_reset_)
    {
        acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();
        acado_inputs_ = kTrimInput_.replicate(1, kSamples).template cast<float>();
        controller_is_reset_ = false;
    }

    return update(state, online_data);
}

/**
 * @brief Perform one step of the MPC 
 * 
 * @tparam T float or double
 * @param state State vector measurement
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
    if (!acado_is_prepared_)
    {
        ROS_ERROR("MPC: Solver was triggered without preparation, abort!");
        return false;
    }

    if (controller_is_reset_)
    {
        // Provide more accurate state estimates
        acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();
        acado_initializeNodesByForwardSimulation();
        prepare();
        controller_is_reset_ = false;
        ROS_INFO("Prepared controller after set/reset");
    }

    // Pass measurement
    setInitialState(state);

    // Perform feedback step and reset preparation check.
    acado_feedbackStep();
    acado_is_prepared_ = false;

    // Check if predicted input is valid
    if (!checkInput())
    {
        // TODO: Maybe reset the MPC somehow?
        ROS_ERROR("MPC has crashed...");
        printSolverState();
        // resetController();
        ros::shutdown();
    }

    return true;
}

/**
 * @brief Perform one step of the MPC 
 * 
 * @tparam T float or double
 * @param state State vector measurement
 * @param online_data The full online data vector
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::update(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data)
{
    if (!acado_is_prepared_)
    {
        ROS_ERROR("MPC: Solver was triggered without preparation, abort!");
        return false;
    }

    if (kOdSize > 0)
    {
        // Pass airdata information
        setOnlineData(online_data);
    }

    // Call simple update for the rest of the functionality
    bool result = update(state);
    return result;
}

/**
 * @brief Shift the solver optimal states and inputs to initialize for the next step
 * 
 * @tparam T float or double
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::shift()
{
    acado_shiftStates(2, 0, 0);
    acado_shiftControls(0);
}

/**
 * @brief Call the ACADO preparation step
 * Must be triggered between iterations if not done in the update function
 * 
 * @tparam T float or double
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::prepare()
{

    acado_preparationStep();
    acado_is_prepared_ = true;

    return true;
}

/**
 * @brief Check if the resulting optimal input has NaN elements
 * 
 * @tparam T 
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::checkInput()
{
    if ((acado_inputs_.array().isNaN()).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

// Reset the solver if it crashes (output==Nan)
/**
 * @brief Reset the solver
 * Also perform one feedback step and check if the new optimal input has NaN elements
 * 
 * @tparam T 
 * @return true 
 * @return false 
 */
template <typename T>
bool MpcWrapper<T>::resetController()
{
    // Clear solver memory
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    // Initialize the solver
    acado_initializeSolver();

    // Initialize states and inputs
    acado_states_ = kTrimState_.replicate(1, kSamples + 1).template cast<float>();
    acado_inputs_ = kTrimInput_.replicate(1, kSamples).template cast<float>();

    if (kOdSize > 0)
    {
        // Initialize online data.
        setOnlineData(kTrimOnlineData_);
    }

    // Initialize the states and controls.
    setReference(defaultReference_, defaultEndReference_);

    // Initialize solver.
    // prepare();
    controller_is_reset_ = true;
}

/**
 * @brief Get the predicted state at a time instance
 * 
 * @tparam T float or double 
 * @param node_index The index of the desired time instance
 * @param return_state The vector onto which to write the state
 */
template <typename T>
void MpcWrapper<T>::getState(const int node_index,
                                 Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state)
{
    return_state = acado_states_.col(node_index).cast<T>();
}

/**
 * @brief Get all the predicted states trajectory
 * 
 * @tparam T 
 * @param return_states The matrix onto which to return the trajectories
 */
template <typename T>
void MpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states)
{
    return_states = acado_states_.cast<T>();
}

/**
 * @brief Get the optimal input at a time instance
 * 
 * @tparam T float or double
 * @param node_index The index of the desired time instance
 * @param return_input The vector onto which to return the input
 */
template <typename T>
void MpcWrapper<T>::getInput(const int node_index,
                                 Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input)
{
    return_input = acado_inputs_.col(node_index).cast<T>();
}

/**
 * @brief Get the whole optimal input trajectory
 * 
 * @tparam T float or double
 * @param return_inputs The matrix onto which to return the trajectories
 */
template <typename T>
void MpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs)
{
    return_inputs = acado_inputs_.cast<T>();
}

template <typename T>
void MpcWrapper<T>::printSolverState()
{
    std::cout << "Online Data: \n" << acado_online_data_ << std::endl;
    std::cout << "Reference:   \n" << acado_reference_states_ << std::endl;
    std::cout << "Measurements:\n" << acado_initial_state_ << std::endl;
    std::cout << "States:      \n" << acado_states_ << std::endl;
    std::cout << "Input:       \n" << acado_inputs_ << std::endl;
}

template class MpcWrapper<float>;
template class MpcWrapper<double>;
