from math import sqrt, atan2, asin, cos, pi
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

from uav_ftc.ellipsoid_fit_python import ellipsoid_fit as el_fit


def build_colorlist(num_lines):
    c_map = plt.cm.get_cmap('jet')
    colorlist = [c_map(1.*i/(num_lines-1)+0.001) for i in range(num_lines)]
    return colorlist


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


##########################
# High-level plot commands
# Typically operate on a list of log_parsing.LogData objects

def plot_path(log_data_list):

    fig = plt.figure()

    axh = fig.add_subplot('111')
    axh.set_prop_cycle(color=build_colorlist(len(log_data_list)))

    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.p_e, log_data.p_n)
        #axh.grid(True)
        #axh.set_xlim([-100, 2000])
        #axh.set_ylim([-100, 2000])
        axh.axis('equal')

    return (fig, axh)


def plot_angular_rates(log_data_list):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_data_list))

    axh = fig.add_subplot('311')
    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.time_refRates, log_data.ref_p, color=colorlist[i], linestyle='dashed')
        axh.plot(log_data.time_databus, log_data.p, color=colorlist[i])
        # Edit these to enable plotting of corresponding command
        # axh2 = axh.twinx()
        # axh2.set_ylabel('control surface command')
        # cmd_h = axh2.plot(t_cmd, cmd, color='k')
        # proxy_state = mpl.patches.Patch(color='tab:red', label=rate_name)
        # proxy_ref = mpl.patches.Patch(color='tab:blue', label=f'ref_{rate_name}')
        # proxy_cmd = mpl.patches.Patch(color='k', label='ctrl_input')
        # axh.legend(handles = [proxy_state, proxy_ref, proxy_cmd])
    axh.set_xticklabels([])
    axh.set_ylabel('Roll (rad/s)')
    axh.grid(True)

    axh = fig.add_subplot('312')
    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.time_refRates, log_data.ref_q, color=colorlist[i], linestyle='dashed')
        axh.plot(log_data.time_databus, log_data.q, color=colorlist[i])
    axh.set_xticklabels([])
    axh.set_ylabel('Pitch (rad/s)')
    axh.grid(True)

    axh = fig.add_subplot('313')
    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.time_refRates, log_data.ref_r, color=colorlist[i], linestyle='dashed')
        axh.plot(log_data.time_databus, log_data.r, color=colorlist[i])
    axh.set_ylabel('Yaw (rad/s)')
    axh.set_xlabel('Time (s)')
    axh.grid(True)

    plt.tight_layout()

    return (fig, axh)


def plot_angular_rates_errors(log_data_list):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_data_list))

    axh = fig.add_subplot('311')
    for i, log_data in enumerate(log_data_list):
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
    axh.set_xticklabels([])
    axh.set_ylabel('Roll (rad/s)')
    axh.grid(True)

    axh = fig.add_subplot('312')
    for i, log_data in enumerate(log_data_list):
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
    for i, log_data in enumerate(log_data_list):
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


def plot_trajectories(log_data_list):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_data_list))

    axh = fig.add_subplot('311')
    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.time_refTrajectory, log_data.ref_Va, color=colorlist[i], linestyle='dashed')
        axh.plot(log_data.time_databus, log_data.airspeed, color=colorlist[i])
    # axh.set_xlim([t_start,t_end]) # Filter data themselves
    axh.set_xticklabels([])
    axh.set_ylabel('Airspeed (m/s)')
    axh.grid(True)

    axh = fig.add_subplot('312')
    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.time_refTrajectory, log_data.ref_gamma, color=colorlist[i], linestyle='dashed')
        axh.plot(log_data.time_databus, log_data.gamma, color=colorlist[i])
    axh.set_xticklabels([])
    axh.set_ylim([-0.4, 0.4])
    axh.set_ylabel('$\gamma$ (rad)')
    axh.grid(True)

    axh = fig.add_subplot('313')
    for i, log_data in enumerate(log_data_list):
        axh.plot(log_data.time_refTrajectory, log_data.ref_psi_dot, color=colorlist[i], linestyle='dashed')
        axh.plot(log_data.time_databus, log_data.psi_dot, color=colorlist[i])
    axh.set_ylim([-0.5, 0.5])
    axh.set_ylabel('$\dot{\psi}$ (rad/s)')
    axh.set_xlabel('Time (s)')
    axh.grid(True)

    plt.tight_layout()

    return (fig, axh)


def plot_trajectories_errors(log_data_list):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_data_list))

    axh = fig.add_subplot('311')
    for i, log_data in enumerate(log_data_list):
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
    for i, log_data in enumerate(log_data_list):
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
    for i, log_data in enumerate(log_data_list):
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

    return (fig, axh)


def plot_euler(log_data_list):
    fig = plt.figure(dpi=400)
    colorlist = build_colorlist(len(log_data_list))

    axh = fig.add_subplot('311')
    axh.set_prop_cycle(color=colorlist)
    for log_data in log_data_list:
        axh.plot(log_data.time_databus, np.rad2deg(log_data.phi))
        axh.set_ylabel('Roll Angle (deg)')
        axh.set_xticklabels([])
        axh.grid(True)

    axh = fig.add_subplot('312')
    axh.set_prop_cycle(color=colorlist)
    for log_data in log_data_list:
        axh.plot(log_data.time_databus, np.rad2deg(log_data.theta))
        axh.set_ylabel('Pitch Angle (deg)')
        axh.set_xticklabels([])
        axh.grid(True)

    axh = fig.add_subplot('313')
    axh.set_prop_cycle(color=colorlist)
    for log_data in log_data_list:
        axh.plot(log_data.time_databus, np.rad2deg(log_data.psi))
        axh.set_xlabel('Time (s)')
        axh.set_ylabel('Yaw Angle (deg)')
        axh.grid(True)

    return (fig, axh)


def plot_flight_envelope(log_data_list, fe_params):
    # Create a figure
    fig = plt.figure(dpi=200)
    # fig.suptitle('airspeed')
    axh = fig.add_subplot('111', projection = '3d', proj_type="ortho")
    
    # Must have as many styles as the length of log_data_list
    colorlist = build_colorlist(len(log_data_list))

    num_logs = len(log_data_list)
    log_cntr = 1

    for i, log_data in enumerate(log_data_list):
        # axh = fig.add_subplot('{}1{}'.format(num_logs, log_cntr), projection = '3d', proj_type="ortho")

        # axh.axhspan(ymin=2, ymax=10, xmin=0.5, xmax=0.9, alpha=0.1)
        # new_line = axh.plot(gamma, airspeed, label='airspeed', linewidth=2.0)
        # plt.setp(new_line, color='r', linewidth=0.5, linestyle='--', marker='1') # Custom property setter

        # Plot actual trajectory
        airspeed = log_data.airspeed
        gamma = log_data.gamma
        psi_dot = log_data.psi_dot
        lh = axh.scatter(airspeed, gamma, psi_dot, c=colorlist[i], s=1)

        # Plot reference trajectory
        airspeed = log_data.ref_Va
        gamma = log_data.ref_gamma
        psi_dot = log_data.ref_psi_dot
        lh = axh.plot(airspeed, gamma, psi_dot, c=colorlist[i], linestyle='dashed')
        # axh.axvline(x=100, ymin=0.1, ymax=1, ls='--', color='r')

        axh.grid(True)
        axh.set_xlabel('$V_a$ (m/s)')
        axh.set_ylabel('$\gamma$')
        axh.set_zlabel('$\dot{\psi}$')
        # axh.set_title('')
        # plt.legend()
        # axh.annotate('point of interest', xy=(1, 1), xytext=(0.5, 2.5),
        #          arrowprops=dict(facecolor='black', shrink=0.05),
        #          )

        log_cntr += 1
    
    center, evecs, radii = fe_params
    el_fit.ellipsoid_plot(center, radii, evecs, axh, cage_color='g', cage_alpha=0.2)

    plt.draw()
    plt.pause(0.01)

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
    fig.savefig('{}.png'.format(img_name), bbox_inches='tight')
    # plt.pause(0.01) # Not sure if needed


def save_figure_3d(img_name, fig):
    # Save figures
    fig.savefig('{}.png'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)

    axh = fig.get_axes()[0]

    axh.view_init(0,0)
    fig.savefig('{}_0_0.png'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)

    axh.view_init(-90,0)
    fig.savefig('{}_90_0.png'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)

    axh.view_init(0,-90)
    fig.savefig('{}_0_90.png'.format(img_name), bbox_inches='tight')
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
