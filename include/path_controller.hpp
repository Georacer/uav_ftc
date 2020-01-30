#include <exception>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <nlopt.h>

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
    PathController(const PathControllerSettings& s);
    ~PathController();
    VectorXd uav_model(VectorXd U);
    double cost_function(unsigned int n, const double* x, double* grad);
    void constraints(unsigned int m, double* c, unsigned int n, const double* x, double* grad);
    void set_waypoints(Matrix<double, Dynamic, 3>);
    void step(Vector4d uav_state);
    int num_wp_lookahead{5};
    Vector3d input_result; // Ouptut from the optimizer

    private:
    double get_distance_from_target(Vector2d pos, Vector2d start, Vector2d finish) const;
    double get_distance_from_target(Vector2d pos, Vector2d target) const;
    int get_current_wp_idx(Vector3d pos) const;
    PathControllerSettings pc_settings_;
    int wp_counter{0};
    Vector4d uav_state_;
    Vector4d state_target_;
    Vector3d wp_target_;
    Vector3d input_target_;
    Matrix<double, Dynamic, 3> waypoints_; // N x 3, NED coordinates
    bool did_receive_waypoints_{false};
    double goal_radius_{10};
    nlopt_opt opt;
    double minJ; // Last optimization cost
    // MPC parameters
    MatrixXd P_;
    MatrixXd Q_;
    MatrixXd R_;
    double dt_; // Integration interval
};

// Static wrappers for optimization functions residing within the PathController objects
double cost_function_wrapper(unsigned int n, const double* x, double* grad, void* data);
void constraints_wrapper(unsigned int m, double* c, unsigned int n, const double* x, double* grad, void* data);