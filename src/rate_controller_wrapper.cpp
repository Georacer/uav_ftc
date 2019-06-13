/**
 * @file rate_controller_wrapper.cpp
 * @author George Zogopoulos-Papaliakos (gzogop@mail.ntua.gr)
 * @brief Wrapper library for the rate controller MPC library
 * @date 2019-06-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "rate_controller_wrapper.hpp"

/**
 * @brief Construct a new Rate Mpc Wrapper< T>:: Rate Mpc Wrapper object
 * 
 * @tparam T 
 */
template <typename T>
RateMpcWrapper<T>::RateMpcWrapper()
{
    // Clear solver memory
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    // Initialize the solver
    acado_initializeSolver();

    // Initialize online data.
    float Va = 15;
    float alpha = 0.034;
    float beta = 0.0;
    Eigen::Matrix<T, kOdSize, 1> online_data(Va, alpha, beta);
    setOnlineData(online_data);

    // Initialize the states and controls.
    const Eigen::Matrix<T, kStateSize, 1> trim_state =
        (Eigen::Matrix<T, kStateSize, 1>() << 0.0, 0.0, 0.0).finished();
    setReferencePose(trim_state);

    // Initialize solver.
    acado_preparationStep();
    acado_is_prepared_ = true;
}

/**
 * @brief Set the online data used by the model
 * 
 * @tparam T float or double
 * @param online_data airspeed, AoA, AoS
 * @return true 
 * @return false 
 */
template <typename T>
bool RateMpcWrapper<T>::setOnlineData(
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data)
{
    // Copy over airspeed
    acado_online_data_.block(0, 0, kOdSize, ACADO_N + 1) = online_data.replicate(1, ACADO_N + 1).template cast<float>();

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
bool RateMpcWrapper<T>::setInitialState(
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
bool RateMpcWrapper<T>::setReferencePose(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        state.replicate(1, kSamples).template cast<float>();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kTrimInput_.replicate(1, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        state.template cast<float>();

    acado_initializeNodesByForwardSimulation();

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
bool RateMpcWrapper<T>::setTrajectory(
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
bool RateMpcWrapper<T>::solve(
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
 * @param state The first-pass guess for the optimal state before optimizing the system
 * @param online_data The currently measured vector of airspeed, AoA, AoS
 * @param do_preparation Peform the ACADO preparation step after the optimization step
 * @return true 
 * @return false 
 */
template <typename T>
bool RateMpcWrapper<T>::update(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data,
    bool do_preparation)
{
    if (!acado_is_prepared_)
    {
        ROS_ERROR("MPC: Solver was triggered without preparation, abort!");
        return false;
    }

    // Pass airdata information
    setOnlineData(online_data);

    // Pass measurement
    setInitialState(state);

    // Perform feedback step and reset preparation check.
    acado_feedbackStep();
    acado_is_prepared_ = false;

    std::cout << "Got airdata: " << acado_online_data_ << std::endl;
    std::cout << "Got reference: " << acado_reference_states_ << std::endl;
    std::cout << "Got measurements: " << acado_initial_state_ << std::endl;
    std::cout << "Expected states: " << acado_states_ << std::endl;
    std::cout << "Made input: " << acado_inputs_ << std::endl;

    /* Optional: shift the initialization (look at acado_common.h). */
    acado_shiftStates(2, 0, 0);
    acado_shiftControls(0);

    // Check if predicted input is valid
    if (!checkInput())
    {
        ROS_ERROR("MPC has crashed, resetting...");
        resetController();
        ros::shutdown();
    }

    // Prepare if the solver if wanted
    if (do_preparation)
    {
        acado_preparationStep();
        acado_is_prepared_ = true;
    }


    return true;
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
bool RateMpcWrapper<T>::prepare()
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
bool RateMpcWrapper<T>::checkInput()
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
bool RateMpcWrapper<T>::resetController()
{
    // Clear solver memory
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    // Initialize the solver
    acado_initializeSolver();

    // Initialize online data.
    float Va = 15;
    float alpha = 0.034;
    float beta = 0.0;
    Eigen::Matrix<T, kOdSize, 1> online_data(Va, alpha, beta);
    setOnlineData(online_data);

    // Initialize the states and controls.
    const Eigen::Matrix<T, kStateSize, 1> trim_state =
        (Eigen::Matrix<T, kStateSize, 1>() << 0.0, 0.0, 0.0).finished();
    setReferencePose(trim_state);

    controller_is_reset_ = true;

    if (checkInput())
    {
        ROS_INFO("MPC has been reset successfully");
        ROS_INFO("Trying to get a new solution");
        setInitialState(trim_state);
        std::cout << "Running optimzation anew:" << std::endl;
        acado_feedbackStep();
        // std::cout << "Got airdata: " << acado_online_data_ << std::endl;
        // std::cout << "Got reference: " << acado_reference_states_ << std::endl;
        // std::cout << "Got measurements: " << acado_initial_state_ << std::endl;
        // std::cout << "Expected states: " << acado_states_ << std::endl;
        // std::cout << "Made input: " << acado_inputs_ << std::endl;

        return true;
    }
    else
    {
        ROS_ERROR("Could not reset MPC");
        return false;
    }
}

/**
 * @brief Get the predicted state at a time instance
 * 
 * @tparam T float or double 
 * @param node_index The index of the desired time instance
 * @param return_state The vector onto which to write the state
 */
template <typename T>
void RateMpcWrapper<T>::getState(const int node_index,
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
void RateMpcWrapper<T>::getStates(
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
void RateMpcWrapper<T>::getInput(const int node_index,
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
void RateMpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs)
{
    return_inputs = acado_inputs_.cast<T>();
}

template class RateMpcWrapper<float>;
template class RateMpcWrapper<double>;
