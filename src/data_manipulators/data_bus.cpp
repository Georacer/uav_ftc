#include "data_bus.hpp"

void convertRosVector3(const Eigen::Vector3d vector3Eigen, geometry_msgs::Vector3 &vector3Ros)
{
    vector3Ros.x = vector3Eigen.x();
    vector3Ros.y = vector3Eigen.y();
    vector3Ros.z = vector3Eigen.z();
}

void convertEigenVector3(const geometry_msgs::Vector3 vector3Ros, Eigen::Vector3d &vector3Eigen)
{
    vector3Eigen.x() = vector3Ros.x;
    vector3Eigen.y() = vector3Ros.y;
    vector3Eigen.z() = vector3Ros.z;
}

void convertRosQuaternion(const Eigen::Quaterniond quatEigen, geometry_msgs::Quaternion &quatRos)
{
    quatRos.x = quatEigen.x();
    quatRos.y = quatEigen.y();
    quatRos.z = quatEigen.z();
    quatRos.w = quatEigen.w();
}

void convertEigenQuaternion(const geometry_msgs::Quaternion quatRos, Eigen::Quaterniond &quatEigen )
{
    quatEigen.x() = quatRos.x;
    quatEigen.y() = quatRos.y;
    quatEigen.z() = quatRos.z;
    quatEigen.w() = quatRos.w;
}

#include "data_from_last_letter.cpp"
#include "data_from_hardware.cpp"


///////////////////////////////
// SubHandler Class definitions
///////////////////////////////
SubHandler::SubHandler()
{
    flag_got_gps = false;
    flag_got_airdata = false;
};


////////////////////////////
// DataBus Class definitions
////////////////////////////
DataBus::DataBus(ros::NodeHandle par_n, uint data_source)
{
    n = par_n; // Store nodehandle
    if (data_source == DATA_SOURCE_LL)
    {
        ROS_INFO("Creating new SITL databus");
        sub_handler_ = new SubHandlerLL(n);
    }
    if (data_source == DATA_SOURCE_HW)
    {
        ROS_INFO("Creating new HW databus");
        sub_handler_ = new SubHandlerHW(n);
    }
    data_pub_ = n.advertise<uav_ftc::BusData>("dataBus", 100);
    ekf_pub_ = n.advertise<uav_ftc::BusData>("ekf", 100);

    ekf_init_();
}

DataBus::~DataBus()
{
    delete sub_handler_;
    delete ekf_;
}

void DataBus::ekf_init_()
{
    ekf_D_.setZero();

    Eigen::VectorXd x0(6);
    x0.setZero(); // Initialize EKF state
    x0(0) = bus_data_.airspeed;

    // Read and store covariances
    double cov_init_vel, cov_init_wind;
    double cov_proc_vel, cov_proc_wind;
    double cov_meas_vel_hor, cov_meas_vel_vert, cov_meas_qbar, cov_meas_alpha, cov_meas_beta;
	n.getParam("ekf/cov_init_vel", cov_init_vel);
    n.getParam("ekf/cov_init_wind", cov_init_wind);
    n.getParam("ekf/cov_proc_vel", cov_proc_vel);
    n.getParam("ekf/cov_proc_wind", cov_proc_wind);
    n.getParam("ekf/cov_meas_vel_hor", cov_meas_vel_hor);
    n.getParam("ekf/cov_meas_vel_vert", cov_meas_vel_vert);
    n.getParam("ekf/cov_meas_qbar", cov_meas_qbar);
    n.getParam("ekf/cov_meas_alpha", cov_meas_alpha);
    n.getParam("ekf/cov_meas_beta", cov_meas_beta);

    Eigen::MatrixXd P0(6,6), Q(6,6), R(6,6);
    P0.setZero();
    double q1 = std::pow(cov_init_vel, 2);
    double q2 = std::pow(cov_init_wind, 2);
    P0.diagonal() << q1, q1, q1, q2, q2, q2; // Initialize P0 with the square if state std_devs

    double pn_vel_rel, pn_vel_wind; // Setup process noises
    double q3 = std::pow(cov_proc_vel, 2);
    double q4 = std::pow(cov_proc_wind, 2);
    Q.setZero();
    Q.diagonal() << q3, q3, q3, q4, q4, q4;

    double mn_vel_gps_hor, mn_vel_gps_vert, mn_qbar, mn_alpha, mn_beta;
    double q5 = std::pow(cov_meas_vel_hor, 2);
    double q6 = std::pow(cov_meas_vel_vert, 2);
    double q7 = std::pow(cov_meas_qbar, 2);
    double q8 = std::pow(cov_meas_alpha, 2);
    double q9 = std::pow(cov_meas_beta, 2);
    R.setZero();
    R.diagonal() << q5, q5, q6, q7, q8, q9;

    double dt = 1/pub_rate_; 
    ekf_ = new Ekf(x0, P0, Q, R, dt);
}

void DataBus::ekf_build_measurement_matrix()
{
    double got_gps = 0, got_airdata = 0;
    if (sub_handler_->flag_got_gps)
    {
        got_gps = 1;
        sub_handler_->flag_got_gps = false;
    }
    if (sub_handler_->flag_got_airdata)
    {
        got_airdata = 1;
        sub_handler_->flag_got_airdata = false;
    }

    ekf_D_.diagonal() << got_gps, got_gps, got_gps, got_airdata, got_airdata, got_airdata;
}

void DataBus::ekf_build_measurements()
{
    // measurements: Earth-frame inertial velocities, qbar, alpha, beta
    // Collect available measurements
    ekf_y_ << bus_data_.inertial_velocity.x, bus_data_.inertial_velocity.y, bus_data_.inertial_velocity.z,
              bus_data_.qbar, bus_data_.angle_of_attack, bus_data_.angle_of_sideslip;

    ekf_y_ = ekf_D_ * ekf_y_; // use this to zero-out old data, to conform with MATLAB code

    // std::cout << ekf_y_.transpose() << std::endl;
}

void DataBus::ekf_build_inputs()
{
    // inputs: body accells, body rates, euler angles
    Eigen::Quaterniond temp_quat;
    convertEigenQuaternion(bus_data_.orientation, temp_quat);
    // Vector3d eul_angles = quat2euler(temp_quat);

    // std::cout << eul_angles.transpose() << std::endl;
    Eigen::Vector3d acc_vect, omega_vect, velocity_vect;
    convertEigenVector3(bus_data_.acceleration_linear, acc_vect);
    convertEigenVector3(bus_data_.velocity_angular, omega_vect);
    convertEigenVector3(bus_data_.inertial_velocity, velocity_vect);
    Eigen::Vector3d acc_linear = acc_vect;// + omega_vect.cross(velocity_vect);

    ekf_u_ << acc_linear, omega_vect, temp_quat.coeffs();
}

void DataBus::ekf_step()
{
    ekf_build_measurement_matrix();
    ekf_build_measurements();
    ekf_build_inputs();
    ekf_->iterate(ekf_u_, ekf_y_, ekf_D_);

    double WN, WE, WD;
    Eigen::Vector3d airdata = getAirData(ekf_->x.segment<3>(0));
    WN = ekf_->x(3);
    WE = ekf_->x(4);
    WD = ekf_->x(5);

    bus_data_.wind.x = WN;
    bus_data_.wind.y = WE;
    bus_data_.wind.z = WD;

    ekf_data_.airspeed = airdata(0);
    ekf_data_.angle_of_attack = airdata(1);
    ekf_data_.angle_of_sideslip = airdata(2);
    ekf_data_.wind.x = WN;
    ekf_data_.wind.y = WE;
    ekf_data_.wind.z = WD;
}

void DataBus::get_handler_data()
{
    bus_data_ = sub_handler_->bus_data;
}

void DataBus::publish_data()
{
    ros::Time timestamp = ros::Time::now();
    bus_data_.header.stamp = timestamp;
    data_pub_.publish(bus_data_);
    ekf_data_.header.stamp = timestamp;
    ekf_pub_.publish(ekf_data_);
}

void DataBus::set_pub_rate(double rate)
{
    pub_rate_ = rate;
    ekf_->set_dt(1/rate);
}

void DataBus::run()
{
	ros::Rate spinner(pub_rate_);

    // uint counter = 0;
	while (ros::ok())
	{
		ros::spinOnce();
        get_handler_data();
        ekf_step();
        publish_data();
		spinner.sleep();

        // counter++;
        // if (counter>inf)
        // {
        //     break;
        // }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_bus");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
    int pub_rate;
	np.param("pub_rate", pub_rate, 100); //publication rate in Hz
    int data_source;
	if (!np.getParam("data_source", data_source))
    {
        ROS_ERROR("Could not read 'data_source' parameter");
        ros::shutdown();
    } //publication rate in Hz

    DataBus data_bus(n, data_source);
    data_bus.set_pub_rate(pub_rate);
	ROS_INFO("DataBus node up");

    data_bus.run();
}