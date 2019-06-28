#!/usr/bin/python3

import sys
import os
from math import sin, cos, tan
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from uav_ftc.plot_utils import get_airdata, get_turn_radius, quat2euler2, rmse

def plot_airspeed():
    axh.set_xlabel('time (s)')
    axh.set_ylabel('Airspeed (m/s)')
    state_h = axh.plot(t_states, airspeed, color='tab:red')
    ref_h = axh.step(t_ref, va_ref, color='tab:blue', where='post')
    axh.set_xlim([t_min, t_max])

    axh2 = axh.twinx()
    axh2.set_ylabel('Throttle command')
    cmd_h = axh2.plot(t_cmd_2, dt_cmd, color='k')
    axh2.set_xlim([t_min, t_max])

    axh.grid(True)
    proxy_state = mpl.patches.Patch(color='tab:red', label='Actual')
    proxy_ref = mpl.patches.Patch(color='tab:blue', label='Desired')
    proxy_cmd = mpl.patches.Patch(color='k', label='Throttle Command')
    axh.legend(handles=[proxy_state, proxy_ref, proxy_cmd])

    metric = rmse(airspeed, np.interp(t_states, t_ref, va_ref))
    axh.text(t_max/2, axh.get_ylim()[0], f'RMSE={metric:.3f}', va='bottom', ha='center')


def plot_gamma():
    axh.set_xlabel('time (s)')
    axh.set_ylabel('Flight Path Angle (rad)')
    state_h = axh.plot(t_states, gamma, color='tab:red')
    ref_h = axh.step(t_ref, gamma_ref, color='tab:blue', where='post')
    axh.set_xlim([t_min, t_max])

    axh.grid(True)
    proxy_state = mpl.patches.Patch(color='tab:red', label='Actual')
    proxy_ref = mpl.patches.Patch(color='tab:blue', label='Desired')
    axh.legend(handles=[proxy_state, proxy_ref])

    metric = rmse(airspeed, np.interp(t_states, t_ref, gamma_ref))
    axh.text(t_max/2, axh.get_ylim()[0], f'RMSE={metric:.3f}', va='bottom', ha='center')


def plot_radius():
    axh.set_xlabel('time (s)')
    axh.set_ylabel('Turn Rate (rad/s)')
    state_h = axh.plot(t_states, psi_dot, color='tab:red')
    ref_h = axh.step(t_ref, turn_rate_ref, color='tab:blue', where='post')
    axh.set_xlim([t_min, t_max])
    
    axh.grid(True)
    proxy_state = mpl.patches.Patch(color='tab:red', label='Actual')
    proxy_ref = mpl.patches.Patch(color='tab:blue', label='Desired')
    axh.legend(handles=[proxy_state, proxy_ref])

    metric = rmse(airspeed, np.interp(t_states, t_ref, turn_rate_ref))
    axh.text(t_max/2, axh.get_ylim()[0], f'RMSE={metric:.3f}', va='bottom', ha='center')
    
    # axh.plot(t_states, phi)
    # axh.plot(t_states, theta)
    # axh.plot(t_states, psi)
    # axh.grid(True)
    # axh.legend(labels=['phi', 'theta', 'psi'])


def plot_cmd_rates():
    axh.set_xlabel('time (s)')
    axh.set_ylabel('Commanded\n Angular Rates (rad/s)')
    cmd_p = axh.plot(t_cmd, p_cmd, color='tab:red')
    cmd_q = axh.plot(t_cmd, q_cmd, color='tab:blue')
    cmd_r = axh.plot(t_cmd, r_cmd, color='tab:green')
    axh.set_xlim([t_min, t_max])

    axh.grid(True)
    axh.legend()
    # proxy_state = mpl.patches.Patch(color='tab:red', label='Actual')
    # proxy_ref = mpl.patches.Patch(color='tab:blue', label='Desired')
    # axh.legend(handles=[proxy_state, proxy_ref])


if __name__ == "__main__":

    if (len(sys.argv) == 2):  # Folder name should be passed
        folder_name = sys.argv[1]
    else:
        folder_name = '.'

    # Read data files
    print("Reading .csv files...")
    states_df = pd.read_csv(f'{folder_name}/states.csv')
    ref_df = pd.read_csv(f'{folder_name}/ref.csv')
    cmd_df = pd.read_csv(f'{folder_name}/rate_input.csv')
    throttle_df = pd.read_csv(f'{folder_name}/throttle_input.csv')

    print("Converting data")
    t_states = states_df.loc[:, 'field.header.stamp']/1e9
    t_ref = ref_df.loc[:, 'field.header.stamp']/1e9
    t_cmd = cmd_df.loc[:, 'field.header.stamp']/1e9
    t_cmd_2 = throttle_df.loc[:, 'field.header.stamp']/1e9
    t_min = 0
    t_max = max([t_states.values[-1], t_ref.values[-1], t_cmd.values[-1], t_cmd_2.values[-1]])

    u_name = 'field.velocity.linear.x'
    v_name = 'field.velocity.linear.y'
    w_name = 'field.velocity.linear.z'
    u = states_df.loc[:, u_name]
    v = states_df.loc[:, v_name]
    w = states_df.loc[:, w_name]
    p_name = 'field.velocity.angular.x'
    q_name = 'field.velocity.angular.y'
    r_name = 'field.velocity.angular.z'
    p = states_df.loc[:, p_name]
    q = states_df.loc[:, q_name]
    r = states_df.loc[:, r_name]

    va_ref_name = 'field.vector.x'
    gamma_ref_name = 'field.vector.y'
    radius_ref_name = 'field.vector.z'
    va_ref = ref_df.loc[:, va_ref_name]
    gamma_ref = ref_df.loc[:, gamma_ref_name]
    radius_ref = ref_df.loc[:, radius_ref_name]
    turn_rate_ref = va_ref/radius_ref*np.cos(gamma_ref)

    p_name = 'field.vector.x'
    q_name = 'field.vector.y'
    r_name = 'field.vector.z'
    p_cmd = cmd_df.loc[:, p_name]
    q_cmd = cmd_df.loc[:, q_name]
    r_cmd = cmd_df.loc[:, r_name]
    dt_name = 'field.vector.x'
    dt_cmd = throttle_df.loc[:, dt_name]

    quat_x_name = 'field.pose.orientation.x'
    quat_y_name = 'field.pose.orientation.y'
    quat_z_name = 'field.pose.orientation.z'
    quat_w_name = 'field.pose.orientation.w'
    quat_x = states_df.loc[:, quat_x_name]
    quat_y = states_df.loc[:, quat_y_name]
    quat_z = states_df.loc[:, quat_z_name]
    quat_w = states_df.loc[:, quat_w_name]
    phi = np.zeros(quat_x.shape)
    theta = np.zeros(quat_x.shape)
    psi = np.zeros(quat_x.shape)

    for i in range(quat_x.shape[0]):
        phi_, theta_, psi_ = quat2euler2(quat_x[i], quat_y[i], quat_z[i], quat_w[i])
        phi[i] = phi_
        theta[i] = theta_
        psi[i] = psi_

    # Vectorization example    
    # fl_env = np.fromfunction(  # Function must support vector inputs
    #     np.vectorize(build_fe_element, excluded=('axes_dict', 'trimmer')), fl_env_dimension, axes_dict=axes_dict, trimmer=trimmer)

    # Convert data where needed
    airdata = np.array(tuple(map(get_airdata, u, v, w)))
    airspeed = airdata[:,0]
    alpha = airdata[:, 1]
    beta = airdata[:,2]
    psi_dot = (q*np.sin(phi) + r*np.cos(phi))/np.cos(theta)
    gamma = np.cos(alpha)*np.cos(beta)*np.sin(theta) - (np.sin(phi)*np.sin(beta)+np.cos(phi)*np.sin(alpha)*np.cos(beta))*np.cos(theta)  # Stevens-Lewis 3.4-2
    # turn_radius = get_turn_radius(airspeed, psi_dot, gamma)

    # Create a figure
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle('Trajectory Components')

    print("Plotting...")
    # Plot subplots
    axh = fig.add_subplot(4, 1, 1)
    plot_airspeed()
    axh = fig.add_subplot(4, 1, 2)
    plot_gamma()
    axh = fig.add_subplot(4, 1, 3)
    plot_radius()
    axh = fig.add_subplot(4, 1, 4)
    plot_cmd_rates()

    plt.show()
