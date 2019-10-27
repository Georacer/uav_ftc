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

class UavEkfModel {

    public: 
    // states: u_r, v_r, w_r, windN, windE, windD
    // inputs: body accells, body rates, euler angles
    // measurements: Earth-frame inertial velocities, qbar, alpha, beta
    Matrix<double,9,1> u; // last applied model inputs (accells, rates, angles)

    UavEkfModel() {}

    Matrix<double,6,1> get_state_derivatives(Matrix<double,6,1> x_curr)
    {
        Vector3d accel = u.segment<3>(0);
        Vector3d rates = u.segment<3>(3);
        Vector3d eul_angles = u.segment<3>(6);

        MatrixXd x_dot = Matrix<double,6,1>::Zero();
        x_dot(0) = udot(accel(0), eul_angles(1), x_curr(1), x_curr(2), rates(2), rates(1)); // u_r_dot
        x_dot(1) = vdot(accel(1), eul_angles(0), eul_angles(1), x_curr(0), x_curr(2), rates(0), rates(2)); // v_r_dot
        x_dot(2) = wdot(accel(2), eul_angles(0), eul_angles(1), x_curr(0), x_curr(1), rates(0), rates(1)); // w_r_dot

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
    }

    Matrix<double,6,6> get_h_matrix(Matrix<double,6,1> x_curr)
    {
        // Construct matrix of partial measurement / state
        MatrixXd H = Matrix<double,6,6>::Zero();
        Vector3d eul_angles = u.segment<3>(6);

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
        Vector3d eul_angles = u.segment<3>(6);

        h(1) = h1(x_curr, eul_angles);
        h(2) = h2(x_curr, eul_angles);
        h(3) = h3(x_curr, eul_angles);
        h(4) = h4(x_curr);
        h(5) = h5(x_curr);
        h(6) = h6(x_curr);

        return h;
    }

    private:

    double udot(double yaccelX, double theta, double v, double w, double r, double q)
    {
        return yaccelX + r*v - q*w;
    }

    double vdot(double yaccelY, double phi, double theta, double u, double w, double p, double r)
    {
        return yaccelY + p*w - r*u;
    }

    double wdot(double yaccelZ, double phi, double theta, double u, double v, double p, double q)
    {
        return yaccelZ + q*u - p*v;
    }

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
        double u = x(0);
        double v = x(1);
        double w = x(2);
        double rho = 1.225;
        
        double hk4 = 0.5*rho*u*u;
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
        double Vat = sqrt(u*u+v*v+w+w);
        
        double hk6 = asin(v/Vat);
        return hk6;
    }
};