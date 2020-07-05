from math import sqrt, atan2, asin, cos, pi
import numpy as np
from scipy import interpolate
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from matplotlib.ticker import FormatStrFormatter
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import re
import pickle
import io

from uav_ftc.ellipsoid_fit_python import ellipsoid_fit as el_fit

# mpl.rcParams['pdf.fonttype'] = 42
# mpl.rc('text', usetex=True)
mpl.rcParams.update({'font.size': 12})


def build_colorlist(num_lines):
    if num_lines <= 1:
        return ['b']
    else:
        c_map = plt.cm.get_cmap('jet')
        colorlist = [c_map(1.*i/(num_lines-1)+0.001) for i in range(num_lines)]
        return colorlist


def sanitize_textext(string):
    # Remove underscores from plaintext string
    underscore_regex = re.compile(r'_')
    return underscore_regex.sub(' ', string)


# Used by test_continuous_fe, deprecate at will
def plot3_points(point_array, axes_names=['x', 'y', 'z']):
    x = point_array[:, 0]
    y = point_array[:, 1]
    z = point_array[:, 2]

    fig = plt.figure()
    ax = fig.add_subplot('111', projection='3d')
    ax.set_proj_type('ortho')
    ax.scatter(x, y, z, c='b', marker='o')

    ax.set_xlabel(axes_names[0])
    ax.set_ylabel(axes_names[1])
    ax.set_zlabel(axes_names[2])

    plt.show()


# General functionality for plotting of 2D data
## INPUTS:
# plot_data: (iter of iter of tuple) Data for plotting
# plot_labels: (iter) Y-Labels for the plotted subfigures
# legend_entries: (iter) Labels for the logs plotted across figures
# x_lims: The x_lims to apply to the plots
# y_lims: (iter) the y_lims to apply to each plot
## OUTPUTS:
# fig: the generated figure handle
def plot_2d(plot_data, plot_labels, series_names,
    plot_colors=None, x_label=None, x_lims=None, y_lims=None,
    plot_type='plot', single_legend=False, single_ref=False,
    dpi=200, fig_size=[6.4, 4.8]):

    fig = plt.figure(dpi=dpi, figsize=fig_size)
    subfig_num = len(plot_data)
    if plot_colors is None:
        plot_colors = [None]*len(plot_data)
        for subfig_idx in range(len(plot_data)):
            plot_colors[subfig_idx] = build_colorlist(len(plot_data[subfig_idx]))

    for plot_idx, subfig_data in enumerate(plot_data):
        axh = fig.add_subplot('{}1{}'.format(subfig_num, plot_idx+1))
        legend_handles = []
        plotted_ref = False
        for series_idx, series_tuple in enumerate(subfig_data):
            color = plot_colors[plot_idx][series_idx]
            series_name = series_names[plot_idx][series_idx]
            series_name = sanitize_textext(series_name)
            x_data = series_tuple[0]
            y_data = series_tuple[1]
            if ('ref' in series_name) or ('est' in series_name):
                if not plotted_ref or not single_ref:
                    axh.step(x_data, y_data, where='post', color=color, linestyle='dashed', linewidth=0.5)
                    legend_handles.append(mpl.patches.Patch(color=color, label=series_name, hatch='/'))
                    plotted_ref = True
            else:
                if plot_type == 'plot':
                    axh.plot(x_data, y_data, color=color, linewidth=0.5)
                elif plot_type == 'scatter':
                    axh.scatter(x_data, y_data, c=color, s=1.0, marker='x')
                else:
                    raise ValueError('Unsupported plot type {}'.format(plot_type))
                legend_handles.append(mpl.patches.Patch(color=color, label=series_name))

        if x_lims is not None:
            axh.set_xlim(x_lims)
        # Don't tick x-axis if it's not the last subfigure
        # if plot_idx < subfig_num - 1:
        #     axh.set_xticklabels([])
        if y_lims is not None:
            if y_lims[plot_idx] is not None:
                axh.set_ylim(y_lims[plot_idx])
        axh.set_ylabel(plot_labels[plot_idx])
        axh.grid(True)
        if plot_idx==0 or not single_legend:
            axh.legend(handles = legend_handles, fontsize='xx-small')
    
    if x_label is not None:
        axh.set_xlabel(x_label)
    plt.tight_layout()

    return (fig, axh)


##########################
# High-level plot commands
# Typically operate on a list of log_parsing.LogData objects

def plot_path(log_dataset, plot_path=True):

    fig = plt.figure(dpi=200)
    colorlist = build_colorlist(len(log_dataset.keys()))
    plot_order = log_dataset.keys()

    axh = fig.add_subplot('111')
    axh.set_prop_cycle(color=build_colorlist(len(log_dataset.keys())))

    reference_log = log_dataset[plot_order[0]]
    # Plot waypoints
    if reference_log.waypoints is not None:
        waypoint_list = []
        for waypoint in reference_log.waypoints.T:
            waypoint_list.append(mpatches.Circle(
                (waypoint[1], waypoint[0]),
                radius=waypoint[3],
                alpha = 0.3,
                # color = 'g',
                # fill = False
                ))
        collection = PatchCollection(waypoint_list)
        collection.set_edgecolor('g')
        collection.set_facecolor('none')
        axh.add_collection(collection)

    # Plot obstacles
    if reference_log.obstacles is not None:
        obstacle_list = []
        for obstacle in reference_log.obstacles.T:
            obstacle_list.append(mpatches.Circle(
                (obstacle[1], obstacle[0]),
                radius=obstacle[3],
                alpha = 0.3,
                # color = 'r',
                # fill = True
                ))
        collection = PatchCollection(obstacle_list)
        collection.set_edgecolor('r')
        collection.set_facecolor('r')
        axh.add_collection(collection)

    # Plot paths
    if plot_path:
        legend_handles = []
        for i, log_data_name in enumerate(log_dataset.keys()):
            log_data = log_dataset[log_data_name]
            axh.plot(log_data.p_e, log_data.p_n)
            log_name = sanitize_textext(log_data_name)
            legend_handles.append(mpl.patches.Patch(color=colorlist[i], label=log_name))
        axh.legend(handles = legend_handles, fontsize='xx-small')

    axh.axis('equal')
    axh.set_xlabel('East (m)')
    axh.set_ylabel('North (m)')
    axh.grid(True)

    return (fig, axh)


def plot_plan(log_dataset):

    fig = plt.figure(dpi=200)
    colorlist = build_colorlist(len(log_dataset.keys()))
    plot_order = log_dataset.keys()

    axh = fig.add_subplot('111')
    axh.set_prop_cycle(color=build_colorlist(len(log_dataset.keys())))

    legend_handles = []
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_data = log_dataset[log_data_name]

        # Plot waypoints
        if log_data.waypoints is not None:
            waypoint_list = []
            for waypoint in log_data.waypoints.T:
                waypoint_list.append(mpatches.Circle(
                    (waypoint[1], waypoint[0]),
                    radius=waypoint[3],
                    alpha = 0.3,
                    ))
            collection = PatchCollection(waypoint_list)
            collection.set_edgecolor(colorlist[i])
            collection.set_facecolor('none')
            axh.add_collection(collection)

            log_name = sanitize_textext(log_data_name)
            legend_handles.append(mpl.patches.Patch(color=colorlist[i], label=log_name))

    reference_log = log_dataset[plot_order[0]]
    # Plot obstacles
    if reference_log.obstacles is not None:
        obstacle_list = []
        for obstacle in reference_log.obstacles.T:
            obstacle_list.append(mpatches.Circle(
                (obstacle[1], obstacle[0]),
                radius=obstacle[3],
                alpha = 0.3,
                # color = 'r',
                # fill = True
                ))
        collection = PatchCollection(obstacle_list)
        collection.set_edgecolor('r')
        collection.set_facecolor('r')
        axh.add_collection(collection)

    axh.axis('equal')
    axh.set_xlabel('East (m)')
    axh.set_ylabel('North (m)')
    axh.grid(True)
    axh.legend(handles = legend_handles, fontsize='xx-small')

    return (fig, axh)


def plot_angular_rates(log_dataset, log_names=None, x_lims=None, y_lims=None, plot_ref=False,
    single_ref=False, single_legend=False):

    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Roll (rad/s)', 'Pitch (rad/s)', 'Yaw (rad/s)']
    p_data = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            p_data.append((log_dataset[log_name].time_refRates, log_dataset[log_name].ref_p))
        p_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].p))
    q_data = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            q_data.append((log_dataset[log_name].time_refRates, log_dataset[log_name].ref_q))
        q_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].q))
    r_data = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            r_data.append((log_dataset[log_name].time_refRates, log_dataset[log_name].ref_r))
        r_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].r))
    plot_data = [p_data, q_data, r_data]

    series_names = [None]*3
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_names.append(log_name + ' ref')
        subfig_names.append(log_name)
    for subf_idx in range(3):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None]*3
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(3):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors,
        x_lims=x_lims, y_lims=y_lims, single_ref=single_ref, single_legend=single_legend)
    return fig


def plot_angular_rates_errors(log_dataset):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_dataset.keys()))

    axh = fig.add_subplot('311')
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_name = sanitize_textext(log_data_name)
        log_data = log_dataset[log_data_name]
        time = log_data.time_databus
        time_ref = log_data.time_refRates
        interp_func = interpolate.interp1d(time_ref, log_data.ref_p, kind='previous', fill_value='extrapolate')
        ref_p_interp = interp_func(time)
        error = log_data.p - ref_p_interp
        axh.plot(time, error, color=colorlist[i])
        # Edit these to enable plotting of corresponding command
        # axh2 = axh.twinx()
        # axh2.set_ylabel('control surface command')
        # cmd_h = axh2.plot(t_cmd, cmd, color='k')
        # proxy_state = mpl.patches.Patch(color='tab:red', label=rate_name)
        # proxy_ref = mpl.patches.Patch(color='tab:blue', label=f'ref_{rate_name}')
        # proxy_cmd = mpl.patches.Patch(color='k', label='ctrl_input')
        # axh.legend(handles = [proxy_state, proxy_ref, proxy_cmd])
        # metric = rmse(airspeed, np.interp(t_states, t_ref, gamma_ref))
        # axh.text(t_max/2, axh.get_ylim()[0], f'RMSE={metric:.3f}', va='bottom', ha='center')
    axh.set_xticklabels([])
    axh.set_ylabel('Roll (rad/s)')
    axh.grid(True)

    axh = fig.add_subplot('312')
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_data = log_dataset[log_data_name]
        time = log_data.time_databus
        time_ref = log_data.time_refRates
        interp_func = interpolate.interp1d(time_ref, log_data.ref_q, kind='previous', fill_value='extrapolate')
        ref_q_interp = interp_func(time)
        error = log_data.q - ref_q_interp
        axh.plot(time, error, color=colorlist[i])
    axh.set_xticklabels([])
    axh.set_ylabel('Pitch (rad/s)')
    axh.grid(True)

    axh = fig.add_subplot('313')
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_data = log_dataset[log_data_name]
        time = log_data.time_databus
        time_ref = log_data.time_refRates
        interp_func = interpolate.interp1d(time_ref, log_data.ref_r, kind='previous', fill_value='extrapolate')
        ref_r_interp = interp_func(time)
        error = log_data.r - ref_r_interp
        axh.plot(time, error, color=colorlist[i])
    axh.set_ylabel('Yaw (rad/s)')
    axh.set_xlabel('Time (s)')
    axh.grid(True)

    plt.tight_layout()

    return (fig, axh)

def plot_accelerations(log_dataset, log_names=None, x_lims=None, y_lims=None, plot_ref=False):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['acc x (m/s/s)', 'acc y (m/s/s)', 'acc z (m/s/s)']
    p_data = []
    for log_idx, log_name in enumerate(log_names):
        p_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].ax))
    q_data = []
    for log_idx, log_name in enumerate(log_names):
        q_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].ay))
    r_data = []
    for log_idx, log_name in enumerate(log_names):
        r_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].az))
    plot_data = [p_data, q_data, r_data]

    series_names = [None]*3
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_names.append(log_name + ' ref')
        subfig_names.append(log_name)
    for subf_idx in range(3):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None]*3
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(3):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors, x_lims=x_lims, y_lims=y_lims)
    return fig


def plot_trajectories(log_dataset, log_names=None, x_lims=None, y_lims=None,
    plot_ref=False, single_ref=False, single_legend=False):

    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Airspeed (m/s)', 'Gamma (rad)', '$\dot{\psi}$ (rad/s)']
    x_data = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            x_data.append((log_dataset[log_name].time_refTrajectory, log_dataset[log_name].ref_Va))
        x_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].airspeed))
    y_data = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            y_data.append((log_dataset[log_name].time_refTrajectory, log_dataset[log_name].ref_gamma))
        y_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].gamma))
    z_data = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            z_data.append((log_dataset[log_name].time_refTrajectory, log_dataset[log_name].ref_psi_dot))
        z_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].psi_dot))
    plot_data = [x_data, y_data, z_data]

    series_names = [None]*3
    subfigure_names = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfigure_names.append(log_name + ' ref')
        subfigure_names.append(log_name)
    for subf_idx in range(3):
        series_names[subf_idx] = subfigure_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None]*3
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(3):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors,
        x_lims=x_lims, y_lims=y_lims, single_ref=single_ref, single_legend=single_legend)
    return fig


def plot_trajectories_errors(log_dataset):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_dataset.keys()))

    axh = fig.add_subplot('311')
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_data = log_dataset[log_data_name]
        time = log_data.time_databus
        airspeed = log_data.airspeed
        time_ref = log_data.time_refTrajectory
        airspeed_ref = log_data.ref_Va
        interp_func = interpolate.interp1d(time_ref, airspeed_ref, kind='previous', fill_value='extrapolate')
        airspeed_ref_interp = interp_func(time)
        error = airspeed - airspeed_ref_interp
        axh.plot(time, error, color=colorlist[i])
    # axh.set_xlim([t_start,t_end]) # Filter data themselves
    axh.set_xticklabels([])
    axh.set_ylabel('Airspeed (m/s)')
    axh.grid(True)

    axh = fig.add_subplot('312')
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_data = log_dataset[log_data_name]
        time = log_data.time_databus
        gamma = log_data.gamma
        time_ref = log_data.time_refTrajectory
        gamma_ref = log_data.ref_gamma
        interp_func = interpolate.interp1d(time_ref, gamma_ref, kind='previous', fill_value='extrapolate')
        gamma_ref_interp = interp_func(time)
        error = gamma - gamma_ref_interp
        axh.plot(time, error, color=colorlist[i])
    axh.set_xticklabels([])
    axh.set_ylabel('$\gamma$ (rad)')
    axh.grid(True)

    axh = fig.add_subplot('313')
    for i, log_data_name in enumerate(log_dataset.keys()):
        log_data = log_dataset[log_data_name]
        time = log_data.time_databus
        psi_dot = log_data.psi_dot
        time_ref = log_data.time_refTrajectory
        psi_dot_ref = log_data.ref_psi_dot
        interp_func = interpolate.interp1d(time_ref, psi_dot_ref, kind='previous', fill_value='extrapolate')
        psi_dot_ref_interp = interp_func(time)
        error = psi_dot - psi_dot_ref_interp
        axh.plot(time, error, color=colorlist[i])
    axh.set_ylabel('$\dot{\psi}$ (rad/s)')
    axh.set_xlabel('Time (s)')
    axh.grid(True)

    plt.tight_layout()


def plot_airdata(log_dataset, log_names=None, plot_ref=False, x_lims=None, y_lims=None):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Airspeed (m/s)', 'AoA (deg)', 'AoS (deg)']
    x_data = []
    for log_idx, log_name in enumerate(log_names):
        x_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].airspeed))
    y_data = []
    for log_idx, log_name in enumerate(log_names):
        y_data.append((log_dataset[log_name].time_databus, np.rad2deg(log_dataset[log_name].alpha)))
    z_data = []
    for log_idx, log_name in enumerate(log_names):
        z_data.append((log_dataset[log_name].time_databus, np.rad2deg(log_dataset[log_name].beta)))
    plot_data = [x_data, y_data, z_data]

    series_names = [None]*3
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_names.append(log_name + ' ref')
        subfig_names.append(log_name)
    for subf_idx in range(3):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None]*3
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(3):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors, x_lims=x_lims, y_lims=y_lims)

    return fig


def plot_airspeed_estimate(log_dataset, log_names=None, x_lims=None, y_lims=None):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Airspeed (m/s)',]
    x_data = []
    for log_idx, log_name in enumerate(log_names):
        x_data.append((log_dataset[log_name].time_databus, log_dataset[log_name].airspeed))
        x_data.append((log_dataset[log_name].time_estimates, log_dataset[log_name].airspeed_est))
    plot_data = [x_data,]

    series_names = [None,]
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        subfig_names.append(log_name)
        subfig_names.append(log_name + ' estimate')
    for subf_idx in range(len(plot_data)):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None,]
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(len(plot_data)):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors, x_lims=x_lims, y_lims=y_lims)

    return fig


def plot_aoa_estimate(log_dataset, log_names=None, x_lims=None, y_lims=None):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Angle of Attack (deg)',]
    x_data = []
    for log_idx, log_name in enumerate(log_names):
        x_data.append((log_dataset[log_name].time_databus, np.rad2deg(log_dataset[log_name].alpha)))
        x_data.append((log_dataset[log_name].time_estimates, np.rad2deg(log_dataset[log_name].alpha_est)))
    plot_data = [x_data,]

    series_names = [None,]
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        subfig_names.append(log_name)
        subfig_names.append(log_name + ' estimate')
    for subf_idx in range(len(plot_data)):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None,]
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(len(plot_data)):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors, x_lims=x_lims, y_lims=y_lims)

    return fig


def plot_euler(log_dataset, log_names=None, plot_ref=False, x_lims=None, y_lims=None):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)']
    x_data = []
    for log_idx, log_name in enumerate(log_names):
        x_data.append((log_dataset[log_name].time_databus, np.rad2deg(log_dataset[log_name].phi)))
    y_data = []
    for log_idx, log_name in enumerate(log_names):
        y_data.append((log_dataset[log_name].time_databus, np.rad2deg(log_dataset[log_name].theta)))
    z_data = []
    for log_idx, log_name in enumerate(log_names):
        z_data.append((log_dataset[log_name].time_databus, np.rad2deg(log_dataset[log_name].psi)))
    plot_data = [x_data, y_data, z_data]

    series_names = [None]*3
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_names.append(log_name + ' ref')
        subfig_names.append(log_name)
    for subf_idx in range(3):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None]*3
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        if plot_ref:
            subfig_colors.append(log_colors[log_idx])
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(3):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors, x_lims=x_lims, y_lims=y_lims)

    return fig


def plot_inputs(log_dataset, channels=None, log_names=None, plot_ref=False, x_lims=None, y_lims=None,
    single_ref=False, single_legend=False):
    if log_names is None:
        log_names = log_dataset.keys()
    if channels is None:
        channels = [value+1 for value in range(8)]

    plot_labels = ['RC{}'.format(channel) for channel in channels]
    plot_data = [None]*len(plot_labels)
    for subfig_idx, channel in enumerate(channels):
    # for subfig_idx in range(len(plot_labels)):
        plot_data[subfig_idx] = []
        for log_idx, log_name in enumerate(log_names):
            plot_data[subfig_idx].append(
                (
                    log_dataset[log_name].time_databus,
                    getattr(log_dataset[log_name], 'rcout_{}'.format(channel))
                )
            )

    series_names = [None]*len(plot_labels)
    subfig_names = []
    for log_idx, log_name in enumerate(log_names):
        subfig_names.append(log_name)
    for subf_idx in range(len(series_names)):
        series_names[subf_idx] = subfig_names

    log_colors = build_colorlist(len(log_names))
    series_colors = [None]*len(plot_labels)
    subfig_colors = []
    for log_idx, log_name in enumerate(log_names):
        subfig_colors.append(log_colors[log_idx])
    for subf_idx in range(len(series_names)):
        series_colors[subf_idx] = subfig_colors

    fig = plot_2d(plot_data, plot_labels, series_names, series_colors, x_lims=x_lims, y_lims=y_lims)

    return fig


def plot_rps(log_dataset, log_names=None, plot_ref=False, x_lims=None, y_lims=None):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['RPS', 'RC3 \& Airspeed*100']
    log_colors = build_colorlist(len(log_names))
    plot_data = []
    series_names = []
    plot_colors = []

    plot_data.append([])
    plot_colors.append([])
    series_names.append([])
    for log_idx, log_name in enumerate(log_names):
        plot_data[0].append(
            (
                log_dataset[log_name].time_databus,
                log_dataset[log_name].rps_motor
            )
        )
        plot_colors[0].append(log_colors[log_idx])
        series_names[0].append(log_name)
    plot_data.append([])
    plot_colors.append([])
    series_names.append([])
    for log_idx, log_name in enumerate(log_names):
        plot_data[1].append(
            (
                log_dataset[log_name].time_databus,
                log_dataset[log_name].rcout_3
            )
        )
        plot_colors[1].append(log_colors[log_idx])
        series_names[1].append(log_name + ' RC3')
        plot_data[1].append(
            (
                log_dataset[log_name].time_databus,
                log_dataset[log_name].airspeed*100
            )
        )
        plot_colors[1].append(log_colors[log_idx])
        series_names[1].append(log_name + ' Va*100')

    for log_idx, log_name in enumerate(log_names):
        series_names.append(log_name)

    fig = plot_2d(plot_data, plot_labels, series_names, plot_colors, x_lims=x_lims, y_lims=y_lims)

    return fig


def plot_rps_vs_throttle(log_dataset, log_names=None, x_lims=None, y_lims=None):
    if log_names is None:
        log_names = log_dataset.keys()
    plot_labels = ['Motor RPS',]
    log_colors = build_colorlist(len(log_names))
    plot_data = []
    series_names = []
    plot_colors = []

    plot_data.append([])
    plot_colors.append([])
    series_names.append([])
    for log_idx, log_name in enumerate(log_names):
        plot_data[0].append(
            (
                log_dataset[log_name].rcout_3,
                log_dataset[log_name].rps_motor
            )
        )
        plot_colors[0].append(log_colors[log_idx])
        series_names[0].append(log_name)

    for log_idx, log_name in enumerate(log_names):
        series_names.append(log_name)

    fig = plot_2d(plot_data, plot_labels, series_names, plot_colors, dpi=400,
        x_label='Throttle (us of PWM)', x_lims=x_lims, y_lims=y_lims, plot_type='scatter')

    return fig


def plot_fe_subfigure(axh, log_dataset, log_names, fe_params, plot_ref=True):
    # axh.axhspan(ymin=2, ymax=10, xmin=0.5, xmax=0.9, alpha=0.1)
    # new_line = axh.plot(gamma, airspeed, label='airspeed', linewidth=2.0)
    # plt.setp(new_line, color='r', linewidth=0.5, linestyle='--', marker='1') # Custom property setter
    colorlist = build_colorlist(len(log_names))
    legend_handles = []

    for i, log_data_name in enumerate(log_names):
        log_data = log_dataset[log_data_name]
        # Plot actual trajectory
        airspeed = log_data.airspeed
        gamma = log_data.gamma
        psi_dot = log_data.psi_dot
        lh = axh.scatter(airspeed, gamma, psi_dot, c=colorlist[i], s=1)
        log_name = sanitize_textext(log_names[i])
        legend_handles.append(mpl.patches.Patch(color=colorlist[i], label=log_name))

        # Plot reference trajectory
        if plot_ref:
            airspeed = log_data.ref_Va
            gamma = log_data.ref_gamma
            psi_dot = log_data.ref_psi_dot
            # lh = axh.plot(airspeed, gamma, psi_dot, c=colorlist[i], linestyle='dashed', linewidth=0.5)
            lh = axh.scatter(airspeed, gamma, psi_dot, marker='x', c=colorlist[i], s=5)
            legend_handles.append(mpl.patches.Patch(color=colorlist[i], label=log_name+' ref', hatch='/'))
        # axh.axvline(x=100, ymin=0.1, ymax=1, ls='--', color='r')

    # axh.set_title('')
    # plt.legend()
    # axh.annotate('point of interest', xy=(1, 1), xytext=(0.5, 2.5),
    #          arrowprops=dict(facecolor='black', shrink=0.05),
    #          )
    axh.grid(True)
    axh.set_xlabel('$V_a$ (m/s)')
    axh.set_ylabel('$\gamma$')
    axh.set_zlabel('$\dot{\psi}$')

    axh.legend(handles = legend_handles, fontsize='x-small')
    center, evecs, radii = fe_params
    el_fit.ellipsoid_plot(center, radii, evecs, axh, cage_color='g', cage_alpha=0.2)


def plot_flight_envelope(log_dataset, fe_params_sets, log_names_sets=None, plot_ref=True):
    # Create a figure
    fig = plt.figure(dpi=200, figsize=(8,16))

    num_plots = len(fe_params_sets)
    subfig_cntr = 1
    for i in range(len(fe_params_sets)):
        axh = fig.add_subplot('{}1{}'.format(num_plots, subfig_cntr), projection = '3d', proj_type="ortho")
        if log_names_sets is None:
            log_names = log_dataset.keys()
        else:
            log_names = log_names_sets[i]
        plot_fe_subfigure(axh, log_dataset, log_names, fe_params_sets[i], plot_ref)
        subfig_cntr += 1

    return fig, axh


#########################
# Low-level plot commands

def plot_3_errors(time_databus, time_ref, x, y, z, ref_x, ref_y, ref_z, axis_labels, y_lims=None):
    fig = plt.figure()
    # fig.suptitle('airspeed')
    axh = fig.add_subplot(311)
    # axh.axhspan(ymin=2, ymax=10, xmin=0.5, xmax=0.9, alpha=0.1)
    # new_line = axh.plot(gamma, airspeed, label='airspeed', linewidth=2.0)
    # plt.setp(new_line, color='r', linewidth=0.5, linestyle='--', marker='1') # Custom property setter
    x_interp = np.interp(time_ref, time_databus, x)
    new_line = axh.plot(time_ref, ref_x - x_interp, c='b')
    # axh.axvline(x=100, ymin=0.1, ymax=1, ls='--', color='r')
    if y_lims is not None:
        axh.set_ylim(y_lims[0])
    axh.grid(True)
    axh.set_xlabel('Time (s)')
    axh.set_ylabel(axis_labels[0])

    axh = fig.add_subplot(312)
    y_interp = np.interp(time_ref, time_databus, y)
    new_line = axh.plot(time_ref, ref_y - y_interp, c='b')
    if y_lims is not None:
        axh.set_ylim(y_lims[1])
    axh.grid(True)
    axh.set_xlabel('Time (s)')
    axh.set_ylabel(axis_labels[1])

    axh = fig.add_subplot(313)
    z_interp = np.interp(time_ref, time_databus, z)
    new_line = axh.plot(time_ref, ref_z - z_interp, c='b')
    if y_lims is not None:
        axh.set_ylim(y_lims[2])
    axh.grid(True)
    axh.set_xlabel('Time (s)')
    axh.set_ylabel(axis_labels[2])

    plt.tight_layout()
    plt.draw()
    plt.pause(0.01)

    return fig


def save_figure_2d(img_name, fig):
    # Save figures
    fig.savefig('{}.png'.format(img_name), bbox_inches='tight', dpi=400)
    # plt.pause(0.01) # Not sure if needed


def save_figure_3d(img_name, fig, dpi=400, filetype='png'):
    # Duplicate figure to be able to selectively hide axes
    # Create temporary storage buffers
    buf_a = io.BytesIO()
    buf_b = io.BytesIO()
    buf_c = io.BytesIO()
    # Dump the figure in them using pickle
    pickle.dump(fig, buf_a)
    pickle.dump(fig, buf_b)
    pickle.dump(fig, buf_c)
    # Rewind the buffers
    buf_a.seek(0)
    buf_b.seek(0)
    buf_c.seek(0)
    # Pull the fresh figures from them
    fig_a = pickle.load(buf_a) 
    fig_b = pickle.load(buf_b) 
    fig_c = pickle.load(buf_c) 

    # Save figures
    fig.savefig('{}.{}'.format(img_name, filetype), bbox_inches='tight', dpi=400)
    plt.pause(0.01)

    for axh in fig_a.get_axes():
        axh.set_xlabel("")
        axh.set_xticklabels([])
        axh.set_xticks([])
        axh.view_init(0,0)
        label = '.\n'+axh.get_ylabel()
        axh.set_ylabel(label)
        axh.tick_params(labelsize=10)
    fig_a.savefig('{}_0_0.{}'.format(img_name, filetype), bbox_inches='tight', dpi=400)
    plt.pause(0.01)

    for axh in fig_b.get_axes():
        axh.set_zlabel("")
        axh.set_zticklabels([])
        axh.set_zticks([])
        axh.view_init(-90,0)
        label = '.\n'+axh.get_xlabel()
        axh.set_xlabel(label)
        axh.tick_params(labelsize=10)
    fig_b.savefig('{}_90_0.{}'.format(img_name, filetype), bbox_inches='tight', dpi=400)
    plt.pause(0.01)

    for axh in fig_c.get_axes():
        axh.set_ylabel("")
        axh.set_yticklabels([])
        axh.set_yticks([])
        axh.view_init(0,-90)
        label = '.\n'+axh.get_xlabel()
        axh.set_xlabel(label)
        axh.tick_params(labelsize=10)
    fig_c.savefig('{}_0_90.{}'.format(img_name, filetype), bbox_inches='tight', dpi=400)
    plt.pause(0.01)


def rmse(x, y):
    return np.sqrt(np.mean(np.power(x-y, 2)))


###########################
# Used by polytope_utils.py

def plot_points(ah, point_arr, style, color):
    x = point_arr[0, :]
    y = point_arr[1, :]
    ah.scatter(x, y, marker=style, c=color)


def plot_points_3(ah, point_arr, style, color, alpha=1):
    x = point_arr[0, :]
    y = point_arr[1, :]
    z = point_arr[2, :]
    ah.scatter(x, y, z, marker=style, c=color, alpha=alpha)


def plot_line(ah, point_1, point_2, color):
    ah.plot([point_1[0], point_2[0]], [point_1[1], point_2[1]], c=color)


def plot_polygons_3(ah, face_points, colorcode=None, alpha=None):
    # Select the plane color
    if colorcode == 'r':
        plane_color = [0.3, 0, 0]
    else:
        plane_color = [0, 1, 0.2]

    poly_collection = mplot3d.art3d.Poly3DCollection(face_points)
    poly_collection.set_alpha = alpha
    poly_collection.set_facecolor(plane_color)
    poly_collection.set_edgecolor('k')
    ah.add_collection3d(poly_collection)
