
namespace {
    using namespace std;
    using namespace Eigen;
}

struct InputConstraints{
    double va_min;
    double va_max;
    double gamma_min;
    double gamma_max;
    double psi_dot_min;
    double psi_dot_max;
};

struct ControllerSettings{
    int num_inputs;
    int num_samples;
    int num_constraints;
    InputConstraints input_constraints;
};

class PathController {
    public:
    PathController(ControllerSettings s);

    private:
    nlopt_opt opt;

}