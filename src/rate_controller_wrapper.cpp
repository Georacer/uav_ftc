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

    //   // Initialize Cost matrix W and WN.
    //   if(!(acado_W_.trace()>0.0))
    //   {
    //     acado_W_ = W_.replicate(1, kSamples).template cast<float>();
    //     acado_W_end_ = WN_.template cast<float>();
    //   }

    // Initialize online data.
    float Va = 15;
    float alpha = 0.034;
    float beta = 0.0;
    Eigen::Matrix<T, kOdSize, 1> online_data(Va, alpha, beta);
    //   Eigen::Matrix<T, 3, 1> p_B_C(0, 0, 0);
    //   Eigen::Quaternion<T> q_B_C(1, 0, 0, 0);
    //   Eigen::Matrix<T, 3, 1> point_of_interest(0, 0, -1000);

    setOnlineData(online_data);

    // Initialize solver.
    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();
    acado_is_prepared_ = true;
}

// // Constructor with cost matrices as arguments.
// template <typename T>
// RateMpcWrapper<T>::RateMpcWrapper(
//   const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
//   const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R)
// {
//   setCosts(Q, R);
//   RateMpcWrapper();
// }

// // Set cost matrices with optional scaling.
// template <typename T>
// bool RateMpcWrapper<T>::setCosts(
//   const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
//   const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
//   const T state_cost_scaling, const T input_cost_scaling)
// {
//   if(state_cost_scaling < 0.0 || input_cost_scaling < 0.0 )
//   {
//     ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
//     return false;
//   }
//   W_.block(0, 0, kCostSize, kCostSize) = Q;
//   W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
//   WN_ = W_.block(0, 0, kCostSize, kCostSize);

//   float state_scale{1.0};
//   float input_scale{1.0};
//   for(int i=0; i<kSamples; i++)
//   {
//     state_scale = exp(- float(i)/float(kSamples)
//       * float(state_cost_scaling));
//     input_scale = exp(- float(i)/float(kSamples)
//       * float(input_cost_scaling));
//     acado_W_.block(0, i*kRefSize, kCostSize, kCostSize) =
//       W_.block(0, 0, kCostSize, kCostSize).template cast<float>()
//       * state_scale;
//     acado_W_.block(kCostSize, i*kRefSize+kCostSize, kInputSize, kInputSize) =
//       W_.block(kCostSize, kCostSize, kInputSize, kInputSize
//         ).template cast<float>() * input_scale;
//   }
//   acado_W_end_ = WN_.template cast<float>() * state_scale;

//   return true;
// }

// // Set the input limits.
// template <typename T>
// bool RateMpcWrapper<T>::setLimits(T min_thrust, T max_thrust,
//     T max_rollpitchrate, T max_yawrate)
// {
//   if(min_thrust <= 0.0 || min_thrust > max_thrust)
//   {
//     ROS_ERROR("MPC: Minimal thrust is not set properly, not changed.");
//     return false;
//   }

//   if(max_thrust <= 0.0 || min_thrust > max_thrust)
//   {
//     ROS_ERROR("MPC: Maximal thrust is not set properly, not changed.");
//     return false;
//   }

//   if(max_rollpitchrate <= 0.0)
//   {
//     ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
//     return false;
//   }

//   if(max_yawrate <= 0.0)
//   {
//     ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
//     return false;
//   }

//   // Set input boundaries.
//   Eigen::Matrix<T, 4, 1> lower_bounds = Eigen::Matrix<T, 4, 1>::Zero();
//   Eigen::Matrix<T, 4, 1> upper_bounds = Eigen::Matrix<T, 4, 1>::Zero();
//   lower_bounds << min_thrust,
//     -max_rollpitchrate, -max_rollpitchrate, -max_yawrate;
//   upper_bounds << max_thrust,
//     max_rollpitchrate, max_rollpitchrate, max_yawrate;

//   acado_lower_bounds_ =
//     lower_bounds.replicate(1, kSamples).template cast<float>();

//   acado_upper_bounds_ =
//     upper_bounds.replicate(1, kSamples).template cast<float>();
//   return true;
// }

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

    std::cout << "Ready to perform solver step" << std::endl;
    std::cout << "Initial State: " << acado_initial_state_ << std::endl;
    std::cout << "Online data: " << acado_online_data_ << std::endl;
    std::cout << "Reference: " << acado_reference_states_ << std::endl;

    // Perform feedback step and reset preparation check.
    acado_feedbackStep();
    acado_is_prepared_ = false;

    std::cout << "Resulting input: " << acado_inputs_ << std::endl;

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
