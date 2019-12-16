# Aggregated flight data for batch publishing
# Some data may be redundant but all should agree with each other
Header header
geometry_msgs/Vector3 coordinates 
geometry_msgs/Vector3 inertial_velocity # NED frame
geometry_msgs/Point position # NED frame
geometry_msgs/Quaternion orientation 
geometry_msgs/Vector3 velocity_angular # Body frame
geometry_msgs/Vector3 acceleration_angular # Body frame
geometry_msgs/Vector3 acceleration_linear # Body frame
float64 airspeed
float64 angle_of_attack
float64 angle_of_sideslip
geometry_msgs/Vector3 wind # NED frame
geometry_msgs/Wrench propulsion
float64 temperature_air
float64 temperature_imu
float64 pressure_absolute
float64 qbar # Differential (Dynamic) pressure
float64 rho # Air density
float64 g # Gravity acceleration
float64[8] rc_in
float64[14] rc_out # In PWM
float64 rps_motor