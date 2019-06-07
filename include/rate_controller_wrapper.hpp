#include <Eigen/Eigen>
#include <ros/ros.h>

#include "rate_controller_mpc_solver/acado_auxiliary_functions.h"
#include "rate_controller_mpc_solver/acado_common.h"

static constexpr int kSamples = ACADO_N;              // number of samples
static constexpr int kStateSize = ACADO_NX;           // number of states
static constexpr int kRefSize = ACADO_NY;             // number of reference states
static constexpr int kEndRefSize = ACADO_NYN;         // number of end reference states
static constexpr int kInputSize = ACADO_NU;           // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs, we do not use them in our case
static constexpr int kOdSize = ACADO_NOD;             // number of online data

const float dt = 0.05; // Be careful to set same as _solver.cpp

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

template <typename T>
class RateMpcWrapper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RateMpcWrapper(); // No-argument constructor
                      //   RateMpcWrapper(  // Constructor with custom Q and R costs
                      //     const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
                      //     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R);

    //   bool setCosts(  // We do not plan to add varying costs any time now...
    //     const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    //     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    //     const T state_cost_scaling = 0.0, const T input_cost_scaling = 0.0);

    //   bool setLimits(T min_thrust, T max_thrust,
    //     T max_rollpitchrate, T max_yawrate);  // Currently we haven't set variable limits

    bool MpcWrapper<T>::setOnlineData(
        const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data);
    bool setReferencePose(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
    bool setTrajectory(
        const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
        const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs);

    bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
    bool update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
                bool do_preparation = true);
    bool prepare();

    void getState(const int node_index,
                  Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state);
    void getStates(
        Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states);
    void getInput(const int node_index,
                  Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input);
    void getInputs(
        Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_input);
    T getTimestep() { return dt_; }

private:
    Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
        acado_reference_states_{acadoVariables.y};

    Eigen::Map<Eigen::Matrix<float, kEndRefSize, 1, Eigen::ColMajor>>
        acado_reference_end_state_{acadoVariables.yN};

    Eigen::Map<Eigen::Matrix<float, kStateSize, 1, Eigen::ColMajor>>
        acado_initial_state_{acadoVariables.x0};

    Eigen::Map<Eigen::Matrix<float, kStateSize, kSamples + 1, Eigen::ColMajor>>
        acado_states_{acadoVariables.x};

    Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
        acado_inputs_{acadoVariables.u};

    Eigen::Map<Eigen::Matrix<float, kOdSize, kSamples + 1, Eigen::ColMajor>>
        acado_online_data_{acadoVariables.od};

    // We do not have variable weight matrices
    //   Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>>
    //     acado_W_{acadoVariables.W};

    //   Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>>
    //     acado_W_end_{acadoVariables.WN};

    Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
        acado_lower_bounds_{acadoVariables.lbValues};

    Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
        acado_upper_bounds_{acadoVariables.ubValues};

    // Not sure what this is, perhaps a local W matrix initialization?
    Eigen::Matrix<T, kRefSize, kRefSize> W_ = (Eigen::Matrix<T, kRefSize, 1>() << 10 * Eigen::Matrix<T, 3, 1>::Ones(),
                                               100 * Eigen::Matrix<T, 4, 1>::Ones(),
                                               10 * Eigen::Matrix<T, 3, 1>::Ones(),
                                               Eigen::Matrix<T, 2, 1>::Zero(),
                                               1, 10, 10, 1)
                                                  .finished()
                                                  .asDiagonal();

    Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ =
        W_.block(0, 0, kEndRefSize, kEndRefSize);

    bool acado_is_prepared_{false};
    const T dt_{dt};
    const Eigen::Matrix<real_t, kInputSize, 1> kTrimInput = // Not meaningful in our case
        (Eigen::Matrix<real_t, kInputSize, 1>() << 0.0, 0.0, 0.0).finished();
};
