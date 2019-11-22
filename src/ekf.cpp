#include "math_utils.hpp"
#include "Eigen/Eigen"


using std::cos;
using std::sin;
using std::asin;
using std::atan2;
using std::sqrt;

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Quaterniond;

class UavEkfModel 
{

    public: 
    // states: u_r, v_r, w_r, windN, windE, windD
    // inputs: body accells, body rates, euler angles
    // measurements: Earth-frame inertial velocities, qbar, alpha, beta
    Matrix<double,10,1> u; // last applied model inputs (accells, rates, orientation quaterion)
    Quaterniond rotation_quat;
    Vector3d eul_angles;

    UavEkfModel() {};

    void set_inputs(VectorXd p_u)
    {
        u = p_u;
        // Update euler angles
        rotation_quat.coeffs() = u.segment<4>(6);
        eul_angles = quat2euler(rotation_quat);
    }

    Matrix<double,6,1> get_state_derivatives(Matrix<double,6,1> x_curr)
    {
        Vector3d accel = u.segment<3>(0);
        Vector3d rates = u.segment<3>(3);
        Vector3d vel = x_curr.segment<3>(0);

        MatrixXd x_dot = Matrix<double,6,1>::Zero();

        Vector3d vel_dot = accel;// + rates.cross(vel); // Stevens-Lewis 2016 eq.1.7-21
        x_dot.block<3,1>(0,0) = vel_dot;

        return x_dot;
    }

    Matrix<double,6,6> get_state_matrix(Matrix<double,6,1> x_curr)
    {
        Vector3d rates = u.segment<3>(3);

        MatrixXd F = Matrix<double,6,6>::Zero();
        F(0,1) = rates(2); // partial u/v
        F(1,0) = -F(0,1); // partial v/u
        F(0,2) = -rates(1); // partial u/w
        F(2,0) = -F(0,2); // partial w/u
        F(1,2) = rates(0); // partial v/w
        F(2,1) = -F(1,2); // partial w/v

        return F;
    }

    Matrix<double,6,6> get_h_matrix(Matrix<double,6,1> x_curr)
    {
        // Construct matrix of partial measurement / state
        MatrixXd H = Matrix<double,6,6>::Zero();

        H.row(0) = derivative_partial_numerical(h1, x_curr, eul_angles);
        H.row(1) = derivative_partial_numerical(h2, x_curr, eul_angles);
        H.row(2) = derivative_partial_numerical(h3, x_curr, eul_angles);
        H.row(3) = derivative_partial_numerical(h4, x_curr);
        H.row(4) = derivative_partial_numerical(h5, x_curr);
        H.row(5) = derivative_partial_numerical(h6, x_curr);

        return H;
    }

    Matrix<double,6,1> get_h_estimate(Matrix<double,6,1> x_curr)
    {
        Matrix<double,6,1> h = Matrix<double,6,1>::Zero();

        Vector3d vel_rel = x_curr.segment<3>(0);
        Vector3d vel_wind = x_curr.segment<3>(3);
        Vector3d vel_inertial = rotation_quat*vel_rel + vel_wind;
        // Vector3d airdata = getAirData(vel_rel); // This is wrong because databus_last_letter generates Va from u_r only

        h.block<3,1>(0,0) = vel_inertial;
        // h.block<3,1>(3,0) = airdata;

        // h(0) = h1(x_curr, eul_angles);
        // h(1) = h2(x_curr, eul_angles);
        // h(2) = h3(x_curr, eul_angles);
        h(3) = h4(x_curr); // Airspeed calculation
        h(4) = h5(x_curr); // aoa calculation
        h(5) = h6(x_curr); // aos calculation

        return h;
    }

    private:

    static double h1(Matrix<double,6,1> x, Vector3d eul_angles)
    {
        double phi = eul_angles(0);
        double theta = eul_angles(1);
        double psi = eul_angles(2);
        double cf = cos(phi);
        double sf = sin(phi);
        double cth = cos(theta);
        double sth = sin(theta);
        double cy = cos(psi);
        double sy = sin(psi);
        double u = x(0);
        double v = x(1);
        double w = x(2);
        double WN = x(3);

        double hk1 = u*(cth*cy)+v*(sf*sth*cy-cf*sy)+(cf*sth*cy+sf*sy)*w + WN; // Equal to pos_n_dot
        return hk1;
    }

    static double h2(Matrix<double,6,1> x, Vector3d eul_angles)
    {
        double phi = eul_angles(0);
        double theta = eul_angles(1);
        double psi = eul_angles(2);
        double cf = cos(phi);
        double sf = sin(phi);
        double cth = cos(theta);
        double sth = sin(theta);
        double cy = cos(psi);
        double sy = sin(psi);
        double u = x(0);
        double v = x(1);
        double w = x(2);
        double WE = x(4);

        double hk2 = u*(cth*sy)+v*(sf*sth*sy+cf*cy)+w*(cf*sth*sy-sf*cy) + WE; // Equal to pos_e_dot
        return hk2;
    }


    static double h3(Matrix<double,6,1> x, Vector3d eul_angles)
    {
        double phi = eul_angles(0);
        double theta = eul_angles(1);
        double psi = eul_angles(2);
        double cf = cos(phi);
        double sf = sin(phi);
        double cth = cos(theta);
        double sth = sin(theta);
        double cy = cos(psi);
        double sy = sin(psi);
        double u = x(0);
        double v = x(1);
        double w = x(2);
        double WD = x(5);

        double hk3 = -u*sth+v*(sf*cth)+w*(cf*cth) + WD; // Equal to pos_d_dot
        return hk3;
    }

    static double h4(Matrix<double,6,1> x)
    {
        // WARNING: This returns zero and zero derivative at 0 airspeed
        double u = x(0);
        double v = x(1);
        double w = x(2);
        double rho = 1.225;
        
        double hk4 = 0.5*rho*u*u; // This is what is generated by databus_last_letter
        // double hk4 = 0.5*rho*(u*u + v*v + w*w);
        return hk4;
    }

    static double h5(Matrix<double,6,1> x)
    {
        double u = x(0);
        double w = x(2);
        
        double hk5 = atan2(w,u);
        return hk5;
    }

    static double h6(Matrix<double,6,1> x)
    {
        double u = x(0);
        double v = x(1);
        double w = x(2);
        double Vat = sqrt(u*u + v*v + w*w);
        
        double hk6 = asin(v/Vat);
        return hk6;
    }
};

class Ekf 
{
    public:
    VectorXd x; // Filter state
    VectorXd x_prev; // State storage
    VectorXd x_dot; // Filter state derivative
    VectorXd h; // Filter output
    VectorXd e; // Filter measurement error
    MatrixXd K; // Optimal gain
    MatrixXd P; // Error covariance matrix
    VectorXd x_err; // State error variance
    MatrixXd F; // State transition matrix
    MatrixXd Q; // Process noise
    MatrixXd R; // Measurement noise
    MatrixXd D; // Measurement availability matrix

    double dt; // sampling time
    UavEkfModel model; // Process model

    Ekf(VectorXd x0, MatrixXd P0, MatrixXd p_Q, MatrixXd p_R, double p_dt)
    {
        x = x0;
        P = P0;
        Q = p_Q;
        R = p_R;
        dt = p_dt;
        model = UavEkfModel();
        K.resizeLike(P0);
        K.setZero();

        D = Eigen::MatrixXd::Identity(R.rows(), R.cols()); // Not actually used
    }

    void set_dt(double p_dt)
    {
        dt = p_dt;
    }

    VectorXd iterate(VectorXd u, VectorXd y, MatrixXd D)
    // Filter iteration
    {
        // std::cout << "u: " << u.transpose() << std::endl;
        // std::cout << "y: " << y.transpose() << std::endl;
        // If u_r is less than 1 then it is likely that the airplane is stationary
        // or falling backwards. Having zero airspeed will crash the EKF due to
        // zero division and rank-reduced matrix inversion.
        // Thus, a small airspeed is always enforced, breaking generality
        if (x(0)<1)
        {
            x(0) = 1;
        }
        time_update(u);
        // std::cout << "post-tu\n" << x.transpose() << std::endl;
        // Propagate filter state
        if (D.trace()>0)
        // If a new measurement is available, do a measurement update
        {
            measurement_update(y, D);
        }
        // std::cout << "post-mu\n" << x.transpose() << std::endl;
        return x;
    }

    private:

    void time_update(VectorXd u)
    // Perform one update of the model
    {
        model.set_inputs(u);
        x_dot = model.get_state_derivatives(x);
        F = model.get_state_matrix(x);
        x_prev = x;
        x = x + x_dot*dt;
        MatrixXd Pd = F*P + P*F.transpose() + Q;
        P = P + Pd*dt;
        x_err = P.diagonal();
    }

    void measurement_update(VectorXd y, MatrixXd D)
    {
        // Calculate estimated output
        // std::cout << "passed D:\n" << D << std::endl;
        MatrixXd H = model.get_h_matrix(x_prev);
        // std::cout << "H matrix:\n" << H << std::endl;
        h = D*model.get_h_estimate(x_prev);
        // std::cout << "h estimate:\n" << h << std::endl;
        // Calculate innovation covariance
        MatrixXd C = D*H;
        MatrixXd S = C*P*C.transpose() + R;
        // std::cout << "S matrix:\n" << S << std::endl;
        // Calculate optimal gain
        K = P*C.transpose()*S.inverse();
        // std::cout << "K matrix:\n" << K << std::endl;
        // Update a-posteriori state estimate
        // Calculate update error
        e = y - h;
        x = x + K*e;
        // Update a-posteriori covariance estimate
        P = (MatrixXd::Identity(K.rows(), C.cols()) - K*C)*P + K*R*K.transpose();
        // std::cout << "P matrix:\n" << P << std::endl;
    }

};