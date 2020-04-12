#include <exception>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <nlopt.h>

#include <math_utils.hpp>

namespace {
    using namespace std;
    using namespace Eigen;
}

class setup_error: public exception {
    public:
    setup_error(const string& s)
        : s_{s}
    {}
    virtual const char* what() const throw()
    {
        string msg = "Called functionality without proper prior setup of: " + s_;
        return msg.c_str();
    }
    private:
    string s_;
};

struct InputConstraints{
    double va_min;
    double va_max;
    double gamma_min;
    double gamma_max;
    double psi_dot_min;
    double psi_dot_max;
};

struct PathControllerSettings{
    int num_states;
    int num_inputs;
    int num_samples;
    double dt;
    MatrixXd obstacles; // N x 4 (North, East, Down, Radius)
    InputConstraints input_constraints;
};

class PathController {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PathController(const PathControllerSettings& s);
    ~PathController();
    VectorXd uav_model(Vector4d state, Vector3d inputs);
    void set_wind(double w_n, double w_e, double w_d);
    void set_fe_ellipsoid(const Ellipsoid3DCoefficients_t);
    MatrixXd propagate_model(MatrixXd inputs);
    double cost_function(unsigned int n, const double* x, double* grad);
    MatrixXd obstacle_constraints(const MatrixXd trajectory);
    VectorXd flight_envelope_constraints(const MatrixXd inputs);
    void constraints(unsigned int m, double* c, unsigned int n, const double* x, double* grad);
    void step(Vector4d uav_state, Vector3d waypoint);
    Vector3d input_result; // Ouptut from the optimizer

    private:
    PathControllerSettings pc_settings_;
    Ellipsoid3DCoefficients_t default_fe_coeffs_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0};  // Assign coeffs which always evaluate true
    Ellipsoid3D fe_ellipsoid_;
    Vector4d uav_state_;
    double wind_n_{0}, wind_e_{0}, wind_d_{0}; // Wind components, NED frame
    Vector4d state_target_;
    Vector3d input_target_;
    nlopt_opt opt;
    double minJ; // Last optimization cost
    // MPC parameters
    MatrixXd P_;
    MatrixXd Q_;
    MatrixXd R_;
    double dt_; // Integration interval
};

class WaypointMngr {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WaypointMngr()
    {};
    ~WaypointMngr()
    {};
    void set_waypoints(const Matrix<double, Dynamic, 3>&);
    void set_num_wp_lookahead(const int);
    void set_goal_radius(const double);
    Vector3d next_waypoint(const Vector3d& pos);

    private:
    double get_distance_from_target(Vector2d pos, Vector2d start, Vector2d finish) const;
    double get_distance_from_target(Vector2d pos, Vector2d target) const;
    int get_current_wp_idx(Vector3d pos) const;
    Matrix<double, Dynamic, 3> waypoints_; // N x 3, NED coordinates
    int wp_counter_{1};
    int num_wp_lookahead_{5};
    double goal_radius_{10};
    bool did_receive_waypoints_{false};
};

// Static wrappers for optimization functions residing within the PathController objects
double cost_function_wrapper(unsigned int n, const double* x, double* grad, void* data);
void constraints_wrapper(unsigned int m, double* c, unsigned int n, const double* x, double* grad, void* data);