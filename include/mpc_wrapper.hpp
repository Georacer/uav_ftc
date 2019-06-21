#include <Eigen/Eigen>
#include <ros/ros.h>

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int kSamples = ACADO_N;              // number of samples
static constexpr int kStateSize = ACADO_NX;           // number of states
static constexpr int kRefSize = ACADO_NY;             // number of reference states
static constexpr int kEndRefSize = ACADO_NYN;         // number of end reference states
static constexpr int kInputSize = ACADO_NU;           // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs, we do not use them in our case
static constexpr int kOdSize = ACADO_NOD;             // number of online data

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

template <typename T>
class MpcWrapper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MpcWrapper(T dt); // No-argument constructor

    bool setTrimState(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
    bool setTrimInput(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input);
    bool setTrimOnlineData(const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> onlineData);
    bool setDefaultReference(const Eigen::Ref<const Eigen::Matrix<T, kRefSize, 1>> reference);
    bool setDefaultEndReference(const Eigen::Ref<const Eigen::Matrix<T, kEndRefSize, 1>> endReference);
    bool setOnlineData(const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> onlineData);
    bool setInitialState(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
    bool setReference(const Eigen::Ref<const Eigen::Matrix<T, kRefSize, 1>> reference,
                      const Eigen::Ref<const Eigen::Matrix<T, kEndRefSize, 1>> referenceEnd);
    bool setTrajectory(
        const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
        const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs);
    bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
               const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data);
    bool update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
                const Eigen::Ref<const Eigen::Matrix<T, kOdSize, 1>> online_data);
    bool shift();
    bool prepare();
    bool checkInput();
    bool resetController();
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

    Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
        acado_lower_bounds_{acadoVariables.lbValues};

    Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
        acado_upper_bounds_{acadoVariables.ubValues};

    bool acado_is_prepared_{false};
    bool controller_is_reset_{false};
    T dt_; // Currently unused
    Eigen::Matrix<real_t, kInputSize, 1> kTrimInput_ =
        Eigen::Matrix<float, kInputSize, 1>::Zero();
    Eigen::Matrix<real_t, kStateSize, 1> kTrimState_ =
        Eigen::Matrix<real_t, kStateSize, 1>::Zero();
    Eigen::Matrix<real_t, kOdSize, 1> kTrimOnlineData_ =
        Eigen::Matrix<real_t, kOdSize, 1>::Zero();
    Eigen::Matrix<real_t, kRefSize, 1> defaultReference_ =
        Eigen::Matrix<real_t, kRefSize, 1>::Zero();
    Eigen::Matrix<real_t, kEndRefSize, 1> defaultEndReference_ =
        Eigen::Matrix<real_t, kEndRefSize, 1>::Zero();
};
