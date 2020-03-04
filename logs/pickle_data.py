#!/usr/bin/python

import os
import pickle

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import click

import rosbag
from rospy import Time

import uav_ftc.trim_traj_fe as fe
from uav_ftc.ellipsoid_fit_python import ellipsoid_fit as el_fit

from log_parsing import LogData
from log_parsing import quat2euler2



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
    help="Currently unused"
)
def test_code(log_file, model_name, export_path):
    
    in_path = os.path.expanduser(log_file)
    in_bag = rosbag.Bag(in_path)
    log_name = os.path.splitext(os.path.split(log_file)[1])[0]

    databus_name = '/{}/dataBus'.format(model_name)
    refRates_name = '/{}/refRates'.format(model_name)
    refTrajectory_name = '/{}/refTrajectory'.format(model_name)
    topics_of_interest = [
       databus_name,
       refTrajectory_name,
       refRates_name
    ]

    log_data = LogData()

    num_databus_msgs = in_bag.get_message_count(topic_filters = [databus_name])
    print('Expecting {} databus msgs'.format(num_databus_msgs))
    log_data.time_databus = np.zeros(num_databus_msgs)
    log_data.p_n = np.zeros(num_databus_msgs)
    log_data.p_e = np.zeros(num_databus_msgs)
    log_data.p_d = np.zeros(num_databus_msgs)
    log_data.airspeed = np.zeros(num_databus_msgs)
    log_data.alpha = np.zeros(num_databus_msgs)
    log_data.beta = np.zeros(num_databus_msgs)
    log_data.phi = np.zeros(num_databus_msgs)
    log_data.theta = np.zeros(num_databus_msgs)
    log_data.psi = np.zeros(num_databus_msgs)
    log_data.p = np.zeros(num_databus_msgs)
    log_data.q = np.zeros(num_databus_msgs)
    log_data.r = np.zeros(num_databus_msgs)

    num_refRates_msgs = in_bag.get_message_count(topic_filters = [refRates_name])
    print('Expecting {} refRates msgs'.format(num_refRates_msgs))
    log_data.time_refRates = np.zeros(num_refRates_msgs)
    log_data.ref_p = np.zeros(num_refRates_msgs)
    log_data.ref_q = np.zeros(num_refRates_msgs)
    log_data.ref_r = np.zeros(num_refRates_msgs)

    num_refTrajectory_msgs = in_bag.get_message_count(topic_filters = [refTrajectory_name])
    print('Expecting {} refTrajectory msgs'.format(num_refTrajectory_msgs))
    log_data.time_refTrajectory = np.zeros(num_refTrajectory_msgs)
    log_data.ref_Va = np.zeros(num_refTrajectory_msgs)
    log_data.ref_gamma = np.zeros(num_refTrajectory_msgs)
    log_data.ref_psi_dot = np.zeros(num_refTrajectory_msgs)

    start_time = in_bag.get_start_time()

    ###########
    # Read msgs
    databus_msg_counter = 0
    refRates_msg_counter = 0
    refTrajectory_msg_counter = 0
    print('Reading bag file...')
    for topic, msg, t in in_bag.read_messages(topics=topics_of_interest):

        if topic.endswith('dataBus'):
            msg_time = msg.header.stamp
            log_data.time_databus[databus_msg_counter] = msg_time.secs + 1.0*msg_time.nsecs/10**9 - start_time
            log_data.p_n[databus_msg_counter]= msg.position.x
            log_data.p_e[databus_msg_counter]= msg.position.y
            log_data.p_d[databus_msg_counter]= msg.position.z
            log_data.airspeed[databus_msg_counter] = msg.airspeed
            log_data.alpha[databus_msg_counter] = msg.angle_of_attack
            log_data.beta[databus_msg_counter] = msg.angle_of_sideslip
            #rotation = Rot.from_quat([
            #    msg.orientation.x,
            #    msg.orientation.y,
            #    msg.orientation.z,
            #    msg.orientation.w,
            #    ])
            #euler_arr = rotation.as_euler('zyx', degrees=False)
            # log_data.phi[databus_msg_counter] = euler_arr[2]
            # log_data.theta[databus_msg_counter] = euler_arr[1]
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
        
    print('Done reading bag file')
    print('Collected {} databus msgs'.format(databus_msg_counter))
    print('Collected {} refRates msgs'.format(refRates_msg_counter))
    print('Collected {} refTrajectory msgs'.format(refTrajectory_msg_counter))
    print(dir(log_data))

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

    with open('{}.pickle'.format(log_name), 'wb') as f:
        # Pickle the 'data' dictionary using the highest protocol available.
        pickle.dump(log_data, f, pickle.HIGHEST_PROTOCOL)

    return


if __name__ == "__main__":
    test_code()
