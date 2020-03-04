#!/usr/bin/python

import os
import pickle

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import click
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

import rosbag
from rospy import Time
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

import uav_ftc.trim_traj_fe as fe
from uav_ftc.ellipsoid_fit_python import ellipsoid_fit as el_fit
from log_parsing import LogData

mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
mpl.rc('text', usetex=True)
# Simplify (smooth) curves with lots of data
mpl.rcParams['path.simplify'] = True
mpl.rcParams['path.simplify_threshold'] = 1.0


t_start = 0
t_end = 250
# For fault FE
#t_start = 0
#t_end = 50

def calc_gamma(alpha, beta, phi, theta):
    return np.arcsin(
            np.cos(alpha)*np.cos(beta)*np.sin(theta)
            -(np.sin(phi)*np.sin(beta)
            + np.cos(phi)*np.sin(alpha)*np.cos(beta))*np.cos(theta)
            )

def calc_psi_dot(phi, theta, q, r):
    return (q*np.sin(phi) + r*np.cos(phi))/np.cos(theta)

def get_image_name(fault_idx):
    if fault_idx == 0:
        img_name = 'nominal'
    if fault_idx == 1:
        img_name = 'fault_motor'
    if fault_idx == 2:
        img_name = 'fault_aileron'
    if fault_idx == 3:
        img_name = 'fault_flap'
    if fault_idx == 4:
        img_name = 'param_variation'
    return img_name

def load_log_data(log_name):
    with open('{}.pickle'.format(log_name), 'rb') as f:
        # The protocol version used is detected automatically, so we do not
        # have to specify it.
        log_data = pickle.load(f)
    return log_data

def filter_log_data(log_data, t_start, t_end):
    databus_start_idx = np.where(log_data.time_databus > t_start)[0][0]
    databus_end_idx = np.where(log_data.time_databus < t_end)[0][-1]
    log_data.time_databus = log_data.time_databus[databus_start_idx:databus_end_idx]
    log_data.p_n = log_data.p_n[databus_start_idx:databus_end_idx]
    log_data.p_e = log_data.p_e[databus_start_idx:databus_end_idx]
    log_data.p_d = log_data.p_d[databus_start_idx:databus_end_idx]
    log_data.airspeed = log_data.airspeed[databus_start_idx:databus_end_idx]
    log_data.alpha = log_data.alpha[databus_start_idx:databus_end_idx]
    log_data.beta = log_data.beta[databus_start_idx:databus_end_idx]
    log_data.phi = log_data.phi[databus_start_idx:databus_end_idx]
    log_data.theta = log_data.theta[databus_start_idx:databus_end_idx]
    log_data.psi = log_data.psi[databus_start_idx:databus_end_idx]
    log_data.p = log_data.p[databus_start_idx:databus_end_idx]
    log_data.q = log_data.q[databus_start_idx:databus_end_idx]
    log_data.r = log_data.r[databus_start_idx:databus_end_idx]
    log_data.gamma = log_data.gamma[databus_start_idx:databus_end_idx]
    log_data.psi_dot = log_data.psi_dot[databus_start_idx:databus_end_idx]

    refRates_start_idx = np.where(log_data.time_refRates > t_start)[0][0]
    refRates_end_idx = np.where(log_data.time_refRates < t_end)[0][-1]
    log_data.time_refRates = log_data.time_refRates[refRates_start_idx:refRates_end_idx]
    log_data.ref_p = log_data.ref_p[refRates_start_idx:refRates_end_idx]
    log_data.ref_q = log_data.ref_q[refRates_start_idx:refRates_end_idx]
    log_data.ref_r = log_data.ref_r[refRates_start_idx:refRates_end_idx]

    refTrajectory_start_idx = np.where(log_data.time_refTrajectory > t_start)[0][0]
    refTrajectory_end_idx = np.where(log_data.time_refTrajectory < t_end)[0][-1]
    log_data.time_refTrajectory = log_data.time_refTrajectory[refTrajectory_start_idx:refTrajectory_end_idx]
    log_data.ref_Va = log_data.ref_Va[refTrajectory_start_idx:refTrajectory_end_idx]
    log_data.ref_gamma = log_data.ref_gamma[refTrajectory_start_idx:refTrajectory_end_idx]
    log_data.ref_psi_dot = log_data.ref_psi_dot[refTrajectory_start_idx:refTrajectory_end_idx]

    return log_data


def get_fe_ellipsoid(model_name, fault_idx):
    flight_envelope = fe.FlightEnvelope(model_name)

    # Set the limits of the search domain
    flight_envelope.set_domain_Va((5, 25))
    flight_envelope.set_domain_gamma((-30, 30))
    flight_envelope.set_R_min(100)
    flight_envelope.initialize(model_name)

    flag_plot_points = True
    img_name = get_image_name(fault_idx)
    if fault_idx == 0:
        print('Flight Envelope for the nominal model')
    if fault_idx == 1:
        print('Inducing engine fault')
        # Set the model parameters. They will not take effect (be written)
        # param_type defined as:
        # typedef enum {
        #     PARAM_TYPE_WORLD = 0,
        #     PARAM_TYPE_ENV,
        #     PARAM_TYPE_INIT,
        #     PARAM_TYPE_INERTIAL,
        #     PARAM_TYPE_AERO,
        #     PARAM_TYPE_PROP,
        #     PARAM_TYPE_GROUND
        # } ParamType_t;
        result = flight_envelope.set_model_parameter( 5, "motor1/omega_max", 10)  # Zero-out propeller efficiency
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = False  # FE too small and poitns hide it

    elif fault_idx == 2:
        print('Inducing aileron fault')
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_l_deltaa", 0.125)  # 50% of the original  
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes

    elif fault_idx == 3:
        print('Inducing flap fault')
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_l_0", 0.09)  
        if result:
            print("Succeeded")
        else:
            print("Failed")
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_n_0", 0.02)
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = False  # FE too small and poitns hide it

    elif fault_idx == 4:
        print('Inducing parameter variations')
        randomize_parameter_list(flight_envelope, parameters_for_randomization)  # Batch randomization
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = True  # FE too small and points hide it


    # Calculate flight envelope
    safe_poly = flight_envelope.find_flight_envelope()
    return safe_poly.ellipsoid_fit()



def plot_flight_envelope(log_data_tuple, model_name, fault_idx):
    # Create a figure
    fig = plt.figure(dpi=200)
    # fig.suptitle('airspeed')
    axh = fig.add_subplot('111', projection = '3d', proj_type="ortho")
    
    plot_styles = ['b', 'r']

    num_logs = len(log_data_tuple)
    log_cntr = 1

    for i, log_data in enumerate(log_data_tuple):
        # axh = fig.add_subplot('{}1{}'.format(num_logs, log_cntr), projection = '3d', proj_type="ortho")

        # axh.axhspan(ymin=2, ymax=10, xmin=0.5, xmax=0.9, alpha=0.1)
        # new_line = axh.plot(gamma, airspeed, label='airspeed', linewidth=2.0)
        # plt.setp(new_line, color='r', linewidth=0.5, linestyle='--', marker='1') # Custom property setter

        # Plot actual trajectory
        airspeed = log_data.airspeed
        gamma = log_data.gamma
        psi_dot = log_data.psi_dot
        new_line = axh.scatter(airspeed, gamma, psi_dot, plot_styles[i], s=1)

        # Plot reference trajectory
        # airspeed = log_data.ref_Va
        # gamma = log_data.ref_gamma
        # psi_dot = log_data.ref_psi_dot
        # new_line = axh.scatter(airspeed, gamma, psi_dot, c='r', s=1)
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
    
    center, evecs, radii, _ = get_fe_ellipsoid(model_name, fault_idx)
    el_fit.ellipsoid_plot(center, radii, evecs, axh, cage_color='g', cage_alpha=0.2)

    plt.draw()
    plt.pause(0.01)

    return fig, axh


def plot_path(log_data_tuple):

    fig = plt.figure()

    axh = fig.add_subplot('111')

    for log_data in log_data_tuple:
        axh.plot(log_data.p_e, log_data.p_n)
        #axh.grid(True)
        #axh.set_xlim([-100, 2000])
        #axh.set_ylim([-100, 2000])
        axh.axis('equal')

    return (fig, axh)

def plot_trajectories(log_data_tuple):
    log_data = log_data_tuple[0]
    log_data_nofe = log_data_tuple[1]

    fig = plt.figure(dpi=400)

    axh = fig.add_subplot('311')

    axh.plot(log_data.time_refTrajectory, log_data.ref_Va, 'b--')
    axh.plot(log_data.time_databus, log_data.airspeed, 'b')
    axh.plot(log_data_nofe.time_refTrajectory, log_data_nofe.ref_Va, 'r--')
    axh.plot(log_data_nofe.time_databus, log_data_nofe.airspeed, 'r')
    axh.set_xlim([t_start,t_end])
    axh.set_xticklabels([])
    axh.set_ylabel('Airspeed (m/s)')
    axh.grid(True)

    axh = fig.add_subplot('312')
    axh.plot(log_data.time_refTrajectory, log_data.ref_gamma, 'b--')
    axh.plot(log_data.time_databus, log_data.gamma, 'b')
    axh.plot(log_data_nofe.time_refTrajectory, log_data_nofe.ref_gamma, 'r--')
    axh.plot(log_data_nofe.time_databus, log_data_nofe.gamma, 'r')
    axh.set_xlim([t_start,t_end])
    axh.set_xticklabels([])
    axh.set_ylim([-0.4, 0.4])
    axh.set_ylabel('$\gamma$ (rad)')
    axh.grid(True)

    axh = fig.add_subplot('313')
    axh.plot(log_data.time_refTrajectory, log_data.ref_psi_dot, 'b--')
    axh.plot(log_data.time_databus, log_data.psi_dot, 'b')
    axh.plot(log_data_nofe.time_refTrajectory, log_data_nofe.ref_psi_dot, 'r--')
    axh.plot(log_data_nofe.time_databus, log_data_nofe.psi_dot, 'r')
    axh.set_xlim([t_start,t_end])
    axh.set_ylim([-0.5, 0.5])
    axh.set_ylabel('$\dot{\psi}$ (rad/s)')
    axh.set_xlabel('Time (s)')
    axh.grid(True)

    plt.tight_layout()

    return (fig, axh)

def plot_euler(log_data_tuple):
    fig = plt.figure()

    axh = fig.add_subplot('311')
    for log_data in log_data_tuple:
        axh.plot(log_data.time_databus, log_data.phi)
        axh.set_ylabel('Phi')

    axh = fig.add_subplot('312')
    for log_data in log_data_tuple:
        axh.plot(log_data.time_databus, log_data.theta)
        axh.set_ylabel('Theta')

    axh = fig.add_subplot('313')
    for log_data in log_data_tuple:
        axh.plot(log_data.time_databus, log_data.psi)
        axh.set_ylabel('Psi')

    return (fig, axh)


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


def save_figure_2d(img_name, fault_idx, fig):
    # Save figures
    fault_name = get_image_name(fault_idx)

    fig.savefig('{}_{}.png'.format(img_name, fault_name), bbox_inches='tight')
    plt.pause(0.01)


def save_figure_3d(img_name, fault_idx, fig, axh):
    # Save figures
    fault_name = get_image_name(fault_idx)

    fig.savefig('{}_{}.png'.format(img_name, fault_name), bbox_inches='tight')
    plt.pause(0.01)

    axh.view_init(0,0)
    fig.savefig('{}_{}_0_0.png'.format(img_name, fault_name), bbox_inches='tight')
    plt.pause(0.01)

    axh.view_init(-90,0)
    fig.savefig('{}_{}_90_0.png'.format(img_name, fault_name), bbox_inches='tight')
    plt.pause(0.01)

    axh.view_init(0,-90)
    fig.savefig('{}_{}_0_90.png'.format(img_name, fault_name), bbox_inches='tight')
    plt.pause(0.01)



@click.command()
@click.option(
    "-m",
    "--model-name",
    default="skywalker_2013_mod",
    help="last_letter_lib UAV model name"
)
@click.option(
    "-f",
    "--fault-idx",
    default=0,
    help="Select fault to simulate",
)
@click.option(
    "-e",
    "--export-path",
    default=None,
    help="Path to export relevant data in .csv files",
)
def test_code(model_name, fault_idx, export_path):
    
    log_data_nominal = load_log_data('mission_sparse')
    log_data_nominal_nofe = load_log_data('mission_sparse_nofe')
    log_data_fault = load_log_data('mission_sparse_fault')
    log_data_fault.time_databus += -50
    log_data_fault.time_refTrajectory += -50
    log_data_fault.time_refRates += -50
    log_data_fault_nofe = load_log_data('mission_sparse_fault_nofe')

    log_data_nominal = filter_log_data(log_data_nominal, t_start, t_end)
    log_data_nominal_nofe = filter_log_data(log_data_nominal_nofe, t_start, t_end)
    log_data_fault = filter_log_data(log_data_fault, t_start, t_end)
    log_data_fault_nofe = filter_log_data(log_data_fault_nofe, t_start, t_end)

    #time_databus = log_data_nominal.time_databus
    #airspeed = log_data_nominal.airspeed
    #alpha = log_data_nominal.alpha
    #beta = log_data_nominal.beta
    #phi = log_data_nominal.phi
    #theta = log_data_nominal.theta
    #p = log_data_nominal.p
    #q = log_data_nominal.q
    #r = log_data_nominal.r
    #gamma = log_data_nominal.gamma
    #psi_dot = log_data_nominal.psi_dot
    #time_refRates = log_data_nominal.time_refRates
    #ref_p = log_data_nominal.ref_p
    #ref_q = log_data_nominal.ref_q
    #ref_r = log_data_nominal.ref_r
    #time_refTrajectory = log_data_nominal.time_refTrajectory
    #ref_Va = log_data_nominal.ref_Va
    #ref_gamma = log_data_nominal.ref_gamma
    #ref_psi_dot = log_data_nominal.ref_psi_dot

    ###############
    # Plotting part

    # Plot Flight envelope
    #fig, axh = plot_flight_envelope((log_data_nominal, log_data_nominal_nofe), model_name, fault_idx)
    #if export_path is not None:
    #    save_figure_3d('fe_el', fault_idx, fig, axh)
    #fig, axh = plot_flight_envelope((log_data_fault, log_data_fault_nofe), model_name, fault_idx)
    #if export_path is not None:
    #    save_figure_3d('fe_el', 3, fig, axh)

    fig, axh = plot_path((log_data_nominal, log_data_nominal_nofe,))
    if export_path is not None:
        save_figure_2d('uav_path', fault_idx, fig)
    # fig, axh = plot_path((log_data_fault, log_data_fault_nofe,))
    # if export_path is not None:
    #     save_figure_2d('uav_path_fault_nofe', fault_idx, fig)

    #fig, axh = plot_trajectories((log_data_nominal, log_data_nominal_nofe,))
    #if export_path is not None:
    #    save_figure_2d('trajectories', fault_idx, fig)
    #fig, axh = plot_trajectories((log_data_fault, log_data_fault_nofe,))
    #if export_path is not None:
    #    save_figure_2d('trajectories_fault', fault_idx, fig)

    #fig, axh = plot_euler((log_data_fault, log_data_fault_nofe,))
    #if export_path is not None:
    #    save_figure_2d('euler_fault', fault_idx, fig)

    # Plot rate control error
    # axis_labels = ['$p$ error', '$q$ error', '$r$ error']
    # fig = plot_3_errors(time_databus, time_refRates, p, q, r, ref_p, ref_q, ref_r, axis_labels)
    # if export_path is not None:
    #     save_figure_2d('rate_error', log_file, fault_idx, fig)

    # Plot trajectory control error
    # axis_labels = ['$V_a$ error', '$\gamma$ error', '$\dot{\psi}$ error']
    # y_lims = [(-1, 1), (-1, 1), (-1, 1)]
    # fig = plot_3_errors(time_databus, time_refTrajectory, airspeed, gamma, psi_dot, ref_Va, ref_gamma, ref_psi_dot, axis_labels, y_lims)
    # if export_path is not None:
    #     save_figure_2d('traj_error', log_file, fault_idx, fig)
    #     
    # return


if __name__ == "__main__":
    test_code()
