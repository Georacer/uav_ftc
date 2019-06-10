#include "rate_controller_wrapper.hpp"

// Default Constructor.
template <typename T>
RateMpcWrapper<T>::RateMpcWrapper()
{
    // Clear solver memory.
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    // Initialize the solver.
    acado_initializeSolver();

    // Initialize the states and controls.
    const Eigen::Matrix<T, kStateSize, 1> trim_state =
        (Eigen::Matrix<T, kStateSize, 1>() << 0.0, 0.0, 0.0).finished();

    // Initialize states x and xN and input u.
    acado_initial_state_ = trim_state.template cast<float>();

    acado_states_ = trim_state.replicate(1, kSamples + 1).template cast<float>();

    acado_inputs_ = kTrimInput_.replicate(1, kSamples).template cast<float>();

    // Initialize references y and yN.
    // Fill out reference states
    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        trim_state.replicate(1, kSamples).template cast<float>();
    // Fill out reference inputs
    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kTrimInput_.replicate(1, kSamples);

    // Fill out reference end-states
    acado_reference_end_state_.segment(0, kStateSize) =
        trim_state.template cast<float>();

    // Initialize online data.
    float Va = 15;
    float alpha = 0.034;
    float beta = 0.0;
    Eigen::Matrix<T, kOdSize, 1> online_data(Va, alpha, beta);

    setOnlineData(online_data);

    // Initialize solver.
    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();
    acado_is_prepared_ = true;
}

// Set online data
template <typename T>
bool RateMpcWrapper<T>::setOnlineData(
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data)
{
    // Copy over airspeed
    acado_online_data_.block(0, 0, kOdSize, ACADO_N + 1) = online_data.replicate(1, ACADO_N + 1).template cast<float>();

    return true;
}

// Set initial state
template <typename T>
bool RateMpcWrapper<T>::setInitialState(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
    acado_initial_state_ = state.template cast<float>();
}

// Set a reference pose.
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

// Set a reference trajectory.
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

// Reset states and inputs and calculate new solution.
template <typename T>
bool RateMpcWrapper<T>::solve(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data)
{
    acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();

    acado_inputs_ = kTrimInput_.replicate(1, kSamples).template cast<float>();

    return update(state, online_data);
}

// Calculate new solution from last known solution.
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

    // Prepare if the solver if wanted
    if (do_preparation)
    {
        acado_preparationStep();
        acado_is_prepared_ = true;
    }

    return true;
}

// Prepare the solver.
// Must be triggered between iterations if not done in the update funciton.
template <typename T>
bool RateMpcWrapper<T>::prepare()
{
    acado_preparationStep();
    acado_is_prepared_ = true;

    return true;
}

// Get a specific state.
template <typename T>
void RateMpcWrapper<T>::getState(const int node_index,
                                 Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state)
{
    return_state = acado_states_.col(node_index).cast<T>();
}

// Get all states.
template <typename T>
void RateMpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states)
{
    return_states = acado_states_.cast<T>();
}

// Get a specific input.
template <typename T>
void RateMpcWrapper<T>::getInput(const int node_index,
                                 Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input)
{
    return_input = acado_inputs_.col(node_index).cast<T>();
}

// Get all inputs.
template <typename T>
void RateMpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs)
{
    return_inputs = acado_inputs_.cast<T>();
}

template class RateMpcWrapper<float>;
template class RateMpcWrapper<double>;
