#!/usr/bin/python3

import sys
import os
from math import sin, cos, tan
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from uav_ftc.uav_model import Vector3, u_to_airdata, get_turn_radius, quat2euler2
from uav_ftc.plot_utils import  rmse, plot_points, plot_line

mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams.update({'font.size': 12})
# mpl.rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
# mpl.rc('text', usetex=True)

def get_y(equation, x):
    return (-equation[0] * x - equation[2]) / equation[1]

def set_axes_limits():
    gamma_min = np.deg2rad(-20)
    gamma_max = np.deg2rad(30)
    axh.set_xlim([gamma_min, gamma_max])
    Va_min = 5
    Va_max = 25
    axh.set_ylim([Va_min, Va_max])


def plot_envelope():
    points_yes_df = pd.read_csv('points_yes.csv')
    points_no_df = pd.read_csv('points_no.csv')
    # separators_df = pd.read_csv('separators.csv')
    vertices_df = pd.read_csv('vertices.csv')

    # Extracting flight envelope
    points_yes_x = points_yes_df.loc[:, 'Theta']
    points_yes_y = points_yes_df.loc[:, 'Va']
    points_no_x = points_no_df.loc[:, 'Theta']
    points_no_y = points_no_df.loc[:, 'Va']
    # equations_a = separators_df.loc[:, 'Theta']
    # equations_b = separators_df.loc[:, 'Va']
    # equations_c = separators_df.loc[:, 'Offset']
    vertices_x = vertices_df.loc[:, 'Theta']
    vertices_y = vertices_df.loc[:, 'Va']

    points_yes = np.array([points_yes_x, points_yes_y])
    points_no = np.array([points_no_x, points_no_y])
    # equations = np.array([equations_a, equations_b, equations_c]).transpose()
    vertices = np.array([vertices_x, vertices_y])
    vertices = np.append(vertices, vertices[:, [0]], 1) # Repeat the last point

    plot_points(axh, points_yes, 'o', 'g')
    plot_points(axh, points_no, 'x', 'r')

    # (x_min, x_max) = axh.get_xlim()
    # (y_min, y_max) = axh.get_ylim()
    # for equation in equations:
    #     try:
    #         point_1 = np.array([x_min, get_y(equation, x_min)])
    #     except (FloatingPointError, ZeroDivisionError):
    #         point_1 = [x_min, y_min]
    #     try:
    #         point_2 = [x_max, get_y(equation, x_max)]
    #     except (FloatingPointError, ZeroDivisionError):
    #         point_2 = [x_max, y_max]
    for i in range(vertices.shape[1]-1):
        plot_line(axh, vertices[:,i], vertices[:, i+1], 'k')


def plot_env_trajectory(gamma, airspeed, gamma_ref, va_ref, config):
    # Setup axes
    axh.set_xlabel('Flight path angle (rad)')
    axh.set_ylabel('Airspeed (m/s)')
    set_axes_limits()

    # Setup plotting canvas
    axh.grid(True)

    # Plot
    color = config['color']
    style = config['style']

    traj_handle = axh.plot(gamma, airspeed, color=color, linewidth=5, linestyle=style)
    ref_h = axh.scatter(gamma_ref, va_ref, color='tab:blue', s=200, marker='d')
    

def plot_scenario(folder_name):
    print("***Scenario {}:".format(folder_name))
    # Read data files
    print("Reading .csv files...")
    # Creating Pandas dataframes to read .csv files
    states_df = pd.read_csv(f'{folder_name}/states.csv')
    ref_df = pd.read_csv(f'{folder_name}/ref.csv')
    cmd_df = pd.read_csv(f'{folder_name}/rate_input.csv')
    throttle_df = pd.read_csv(f'{folder_name}/throttle_input.csv')

    print("Converting data")
    # Extracting time vectors
    t_states = states_df.loc[:, 'field.header.stamp']/1e9
    t_ref = ref_df.loc[:, 'field.header.stamp']/1e9
    t_cmd = cmd_df.loc[:, 'field.header.stamp']/1e9
    t_cmd_2 = throttle_df.loc[:, 'field.header.stamp']/1e9
    t_min = 0
    t_max = max([t_states.values[-1], t_ref.values[-1], t_cmd.values[-1], t_cmd_2.values[-1]])

    # Extracting states
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

    quat_x_name = 'field.pose.orientation.x'
    quat_y_name = 'field.pose.orientation.y'
    quat_z_name = 'field.pose.orientation.z'
    quat_w_name = 'field.pose.orientation.w'
    quat_x = states_df.loc[:, quat_x_name]
    quat_y = states_df.loc[:, quat_y_name]
    quat_z = states_df.loc[:, quat_z_name]
    quat_w = states_df.loc[:, quat_w_name]

    # Create Euler angles
    phi = np.zeros(quat_x.shape) # Build phi
    theta = np.zeros(quat_x.shape) # Build theta 
    psi = np.zeros(quat_x.shape) # Build psi
    for i in range(quat_x.shape[0]):
        phi_, theta_, psi_ = quat2euler2(quat_x[i], quat_y[i], quat_z[i], quat_w[i])
        phi[i] = phi_
        theta[i] = theta_
        psi[i] = psi_

    # Extracting reference timeseries
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

    # Convert data where needed
    u_vec3 = map(Vector3, u, v, w)
    airdata_iter = map(u_to_airdata, u_vec3)
    airdata = np.array(tuple([airdata.to_array() for airdata in airdata_iter]))
    airspeed = airdata[:,0] # Build airspeed
    alpha = airdata[:, 1] # Build alpha
    beta = airdata[:,2] # Build beta
    psi_dot = (q*np.sin(phi) + r*np.cos(phi))/np.cos(theta) # Build psi_dot
    gamma = np.cos(alpha)*np.cos(beta)*np.sin(theta) - (np.sin(phi)*np.sin(beta)+np.cos(phi)*np.sin(alpha)*np.cos(beta))*np.cos(theta)  # Build gamma, Stevens-Lewis 3.4-2

    plot_env_trajectory(gamma, airspeed, gamma_ref, va_ref, graph_settings[folder_name])
    # turn_radius = get_turn_radius(airspeed, psi_dot, gamma)


if __name__ == "__main__":

    # Create a figure
    fig = plt.figure(figsize=(8, 6))
    # fig.suptitle('Flight Envelope') // No title, because it will have a legend in the paper

    print("Plotting...")

    nominal_settings = {'color': 'tab:red', 'style' : '--'}
    protected_settings = {'color': 'k', 'style' : '-'}
    graph_settings = {'nominal': nominal_settings, 'protected' : protected_settings}

    # Plot subplots
    axh = fig.add_subplot(1, 1, 1)
    plot_envelope()
    plot_scenario('nominal')
    plot_scenario('protected')

    # set_axes_limits()

    # Setup legend
    traj_nominal_proxy = mpl.patches.Patch(color=nominal_settings['color'], label='Baseline')
    traj_protected_proxy = mpl.patches.Patch(color=protected_settings['color'], label='Protected')
    ref_proxy = mpl.patches.Patch(color='tab:blue', label='Commanded')
    axh.legend(handles=[traj_nominal_proxy, traj_protected_proxy, ref_proxy])

    # Save to file
    dir_path = os.path.dirname(os.path.realpath(__file__))
    fig.savefig(dir_path+'/flight_envelope_trajectories.pdf', bbox_inches='tight')
    plt.show()
