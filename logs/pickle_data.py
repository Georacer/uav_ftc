#!/usr/bin/python

# Read ROS .bag files, extract all required data series and save as Python
# pickle file. This is done to avoid re-parsing the whole log file while
# generating new code for log plotting.
# NOTE: Will capture only the first waypoints, obstacles and rrt_path message

import os
import pickle

import numpy as np
import click

import rosbag
from rospy import Time

from uav_ftc.uav_model import quat2euler2
import uav_ftc.log_parsing as lp
from uav_ftc.log_parsing import LogData


@click.command()
@click.option(
    "-l",
    "--log-file",
    help="log file path to parse"
)
@click.option(
    "-m",
    "--model-name",
    default="skywalker_2013_mod",
    help="last_letter_lib UAV model name"
)
@click.option(
    "-e",
    "--export-path",
    default=None,
    help="Path to export pickle files at"
)
def test_code(log_file, model_name, export_path):
    
    in_filepath = os.path.expanduser(log_file)

    if export_path is None:
        out_path = in_filepath
    else:
        out_path = os.path.expanduser(export_path)

    in_bag = rosbag.Bag(in_filepath)
    log_name = os.path.splitext(os.path.split(log_file)[1])[0]

    # Construct topic names
    databus_name = '/{}/dataBus'.format(model_name)
    refRates_name = '/{}/refRates'.format(model_name)
    refTrajectory_name = '/{}/refTrajectory'.format(model_name)
    flightEnvelope_name = '/{}/flight_envelope'.format(model_name)
    waypoints_name = '/{}/waypoints'.format(model_name)
    obstacles_name = '/{}/obstacles'.format(model_name)
    ref_path_name = '/{}/rrt_path'.format(model_name)
    topics_of_interest = [
       databus_name,
       refTrajectory_name,
       refRates_name,
       flightEnvelope_name,
       waypoints_name,
       obstacles_name,
       ref_path_name
    ]

    # Allocate numpy arrays
    log_data = LogData()

    num_databus_msgs = in_bag.get_message_count(topic_filters = [databus_name])
    print('Expecting {} databus msgs'.format(num_databus_msgs))
    for attr_name in lp.log_databus_attrs:
        setattr(log_data, attr_name, np.zeros(num_databus_msgs))

    num_refRates_msgs = in_bag.get_message_count(topic_filters = [refRates_name])
    print('Expecting {} refRates msgs'.format(num_refRates_msgs))
    for attr_name in lp.log_ref_rates_attrs:
        setattr(log_data, attr_name, np.zeros(num_refRates_msgs))

    num_refTrajectory_msgs = in_bag.get_message_count(topic_filters = [refTrajectory_name])
    print('Expecting {} refTrajectory msgs'.format(num_refTrajectory_msgs))
    for attr_name in lp.log_ref_traj_attrs:
        setattr(log_data, attr_name, np.zeros(num_refTrajectory_msgs))

    num_Fe_msgs = in_bag.get_message_count(topic_filters=flightEnvelope_name)
    print('Expecting {} flight_envelope msgs'.format(num_Fe_msgs))
    for attr_name in lp.log_fe_attrs:
        setattr(log_data, attr_name, np.zeros(num_Fe_msgs))

    start_time = in_bag.get_start_time()

    ###########
    # Read msgs
    databus_msg_counter = 0
    refRates_msg_counter = 0
    refTrajectory_msg_counter = 0
    fe_msg_counter = 0
    num_waypoints = 0
    num_obstacles = 0
    num_path_points = 0

    parsed_waypoints = False
    parsed_obstacles = False
    parsed_path = False

    print('Reading bag file...')
    for topic, msg, t in in_bag.read_messages(topics=topics_of_interest):

        if topic.endswith('dataBus'):
            msg_time = msg.header.stamp
            log_data.time_databus[databus_msg_counter] = msg_time.secs + 1.0*msg_time.nsecs/10**9 - start_time
            log_data.gps_lat[databus_msg_counter]= msg.coordinates.x
            log_data.gps_lon[databus_msg_counter]= msg.coordinates.y
            log_data.gps_alt[databus_msg_counter]= msg.coordinates.z
            log_data.p_n[databus_msg_counter]= msg.position.x
            log_data.p_e[databus_msg_counter]= msg.position.y
            log_data.p_d[databus_msg_counter]= msg.position.z
            log_data.v_n[databus_msg_counter]= msg.inertial_velocity.x
            log_data.v_e[databus_msg_counter]= msg.inertial_velocity.y
            log_data.v_d[databus_msg_counter]= msg.inertial_velocity.z
            log_data.airspeed[databus_msg_counter] = msg.airspeed
            log_data.alpha[databus_msg_counter] = msg.angle_of_attack
            log_data.beta[databus_msg_counter] = msg.angle_of_sideslip
            log_data.qx[databus_msg_counter] = msg.orientation.x
            log_data.qy[databus_msg_counter] = msg.orientation.y
            log_data.qz[databus_msg_counter] = msg.orientation.z
            log_data.qw[databus_msg_counter] = msg.orientation.w
            phi, theta, psi = quat2euler2(
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                    )
            log_data.phi[databus_msg_counter] = phi
            log_data.theta[databus_msg_counter] = theta
            log_data.psi[databus_msg_counter] = psi
            log_data.p[databus_msg_counter] = msg.velocity_angular.x
            log_data.q[databus_msg_counter] = msg.velocity_angular.y
            log_data.r[databus_msg_counter] = msg.velocity_angular.z
            log_data.ax[databus_msg_counter] = msg.acceleration_linear.x
            log_data.ay[databus_msg_counter] = msg.acceleration_linear.y
            log_data.az[databus_msg_counter] = msg.acceleration_linear.z
            log_data.wind_n[databus_msg_counter] = msg.wind.x
            log_data.wind_e[databus_msg_counter] = msg.wind.y
            log_data.wind_d[databus_msg_counter] = msg.wind.z
            log_data.t_air[databus_msg_counter] = msg.temperature_air
            log_data.t_imu[databus_msg_counter] = msg.temperature_imu
            log_data.p_abs[databus_msg_counter] = msg.pressure_absolute
            log_data.qbar[databus_msg_counter] = msg.qbar
            log_data.rho[databus_msg_counter] = msg.rho
            log_data.g[databus_msg_counter] = msg.g
            log_data.rps_motor[databus_msg_counter] = msg.rps_motor
            log_data.rcin_1[databus_msg_counter] = msg.rc_in[0]
            log_data.rcin_2[databus_msg_counter] = msg.rc_in[1]
            log_data.rcin_3[databus_msg_counter] = msg.rc_in[2]
            log_data.rcin_4[databus_msg_counter] = msg.rc_in[3]
            log_data.rcin_5[databus_msg_counter] = msg.rc_in[4]
            log_data.rcin_6[databus_msg_counter] = msg.rc_in[5]
            log_data.rcin_7[databus_msg_counter] = msg.rc_in[6]
            log_data.rcin_8[databus_msg_counter] = msg.rc_in[7]
            log_data.rcout_1[databus_msg_counter] = msg.rc_out[0]
            log_data.rcout_2[databus_msg_counter] = msg.rc_out[1]
            log_data.rcout_3[databus_msg_counter] = msg.rc_out[2]
            log_data.rcout_4[databus_msg_counter] = msg.rc_out[3]
            log_data.rcout_5[databus_msg_counter] = msg.rc_out[4]
            log_data.rcout_6[databus_msg_counter] = msg.rc_out[5]
            log_data.rcout_7[databus_msg_counter] = msg.rc_out[6]
            log_data.rcout_8[databus_msg_counter] = msg.rc_out[7]

            databus_msg_counter += 1

        if topic.endswith('refRates'):
            msg_time = msg.header.stamp
            log_data.time_refRates[refRates_msg_counter] = msg_time.secs + 1.0*msg_time.nsecs/10**9 - start_time
            log_data.ref_p[refRates_msg_counter] = msg.vector.x
            log_data.ref_q[refRates_msg_counter] = msg.vector.y
            log_data.ref_r[refRates_msg_counter] = msg.vector.z

            refRates_msg_counter += 1

        if topic.endswith('refTrajectory'):
            msg_time = msg.header.stamp
            log_data.time_refTrajectory[refTrajectory_msg_counter] = msg_time.secs + 1.0*msg_time.nsecs/10**9 - start_time
            log_data.ref_Va[refTrajectory_msg_counter] = msg.vector.x
            log_data.ref_gamma[refTrajectory_msg_counter] = msg.vector.y
            log_data.ref_psi_dot[refTrajectory_msg_counter] = msg.vector.z

            refTrajectory_msg_counter += 1

        if topic.endswith('flight_envelope'):
            msg_time = msg.header.stamp
            log_data.time_fe[fe_msg_counter] = msg_time.secs + 1.0*msg_time.nsecs/10**9 - start_time
            log_data.el_A[fe_msg_counter] = msg.el_A
            log_data.el_B[fe_msg_counter] = msg.el_B
            log_data.el_C[fe_msg_counter] = msg.el_C
            log_data.el_D[fe_msg_counter] = msg.el_D
            log_data.el_E[fe_msg_counter] = msg.el_E
            log_data.el_F[fe_msg_counter] = msg.el_F
            log_data.el_G[fe_msg_counter] = msg.el_G
            log_data.el_H[fe_msg_counter] = msg.el_H
            log_data.el_I[fe_msg_counter] = msg.el_I
            log_data.el_J[fe_msg_counter] = msg.el_J

            fe_msg_counter += 1

        if topic.endswith('waypoints') and not parsed_waypoints:
            num_waypoints = len(msg.markers)
            print('Found {} waypoints'.format(num_waypoints))
            log_data.waypoints = np.zeros((4, num_waypoints))
            for i, marker in enumerate(msg.markers):
                wp_info = np.array([[
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                    marker.scale.x/2  # Convert diameter to radius
                ]]).T
                log_data.waypoints[:, [i]] = wp_info
            parsed_waypoints = True

        if topic.endswith('obstacles') and not parsed_obstacles:
            num_obstacles = len(msg.markers)
            print('Found {} obstacles'.format(num_obstacles))
            log_data.obstacles = np.zeros((4, num_obstacles))
            for i, marker in enumerate(msg.markers):
                obs_info = np.array([[
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                    marker.scale.x/2  # Convert diameter to radius
                ]]).T
                log_data.obstacles[:, [i]] = obs_info
            parsed_obstacles = True

        if topic.endswith('rrt_path') and not parsed_path:
            num_path_points = len(msg.poses)
            print('Found {} path points'.format(num_path_points))
            log_data.ref_path = np.zeros((4, num_path_points))
            for i, pose in enumerate(msg.poses):
                point_info = np.array([[
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    30  # Taken from controller_params.yaml/waypointRadius
                ]]).T
                log_data.ref_path[:, [i]] = point_info
            parsed_path = True
        
    print('Done reading bag file')
    print('Collected {} databus msgs'.format(databus_msg_counter))
    print('Collected {} refRates msgs'.format(refRates_msg_counter))
    print('Collected {} refTrajectory msgs'.format(refTrajectory_msg_counter))
    print('Collected {} flight_envelope msgs'.format(fe_msg_counter))
    print('Collected {} waypoints'.format(num_waypoints))
    print('Collected {} obstacles'.format(num_obstacles))
    print('Collected {} path points'.format(num_path_points))

    # Construct additional, derived timeseries
    log_data.gamma = np.arcsin(
            np.cos(log_data.alpha)*np.cos(log_data.beta)*np.sin(log_data.theta)
            -(
                np.sin(log_data.phi)*np.sin(log_data.beta)
                + np.cos(log_data.phi)*np.sin(log_data.alpha)*np.cos(log_data.beta)
             )*np.cos(log_data.theta)
            )
    log_data.psi_dot = (
            (
                log_data.q*np.sin(log_data.phi)
                + log_data.r*np.cos(log_data.phi)
            )/np.cos(log_data.theta)
            )

    # Write pickle file
    out_filename = '{}.pickle'.format(log_name)
    out_filepath = os.path.join(out_path, out_filename)
    with open(out_filepath, 'wb') as f:
        # Pickle the 'data' dictionary using the highest protocol available.
        pickle.dump(log_data, f, pickle.HIGHEST_PROTOCOL)

    return

if __name__ == '__main__':
    test_code()
