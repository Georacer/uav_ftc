#!/usr/bin/python

import os
import sys
import pickle
import numpy as np
import click
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

from rospy import Time
import rosbag

from uav_ftc import plot_utils as pu
from uav_ftc.uav_model import calc_gamma, calc_psi_dot
import uav_ftc.trim_traj_fe as fe
from uav_ftc.ellipsoid_fit_python import ellipsoid_fit as el_fit
from uav_ftc.log_parsing import LogData, filter_log_data

# Add do python paths because Pickle does not loot at ROS ws includes
sys.path.append('/home/george/ros_workspaces/uav_ftc/src/uav_ftc/src/uav_ftc')

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


def load_pickle_data(log_name, log_dir=None):
    if log_dir is None:
        pickle_filepath = os.path.expanduser(log_name)
    else:
        pickle_filepath = os.path.join(os.path.expanduser(log_dir), log_name)

    with open(pickle_filepath, 'rb') as f:
        # The protocol version used is detected automatically, so we do not
        # have to specify it.
        log_data = pickle.load(f)
    return log_data


def build_data_list(log_directory):

    # Collect all available log files
    name_nominal = 'nominal'
    name_nominal_nofe = 'nominal_nofe'
    name_faulty = 'faulty'
    name_faulty_nofe = 'faulty_nofe'

    file_names = [
        name_nominal,
        name_nominal_nofe,
        name_faulty,
        name_faulty_nofe
    ]

    data_list = list()

    for filename in file_names:
        log_path = os.path.join(log_directory, filename + '.pickle')
        if os.path.exists(log_path):
            print('Adding {} log file.'.format(log_path))
            log_file = load_pickle_data(log_path)
            log_file = filter_log_data(log_file, t_start, t_end)
            data_list.append(log_file)

    return data_list


def get_fe_params(log_data, index=0):
    v = [
        log_data.el_A[index],
        log_data.el_B[index],
        log_data.el_C[index],
        log_data.el_D[index],
        log_data.el_E[index],
        log_data.el_F[index],
        log_data.el_G[index],
        log_data.el_H[index],
        log_data.el_I[index],
        log_data.el_J[index]
    ]
    center, evecs, radii = el_fit.calculate_ellipsoid_parameters(v)
    return (center, evecs, radii)


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


def get_flight_envelope(model_name, fault_idx):
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

    return safe_poly


# Accepts a SafeConvexPolytope instance
def get_fe_ellipsoid(flight_envelope):
    return flight_envelope.ellipsoid_fit()


# Plot Flight envelope
def generate_flight_envelope(data_list, export_path):
    # Get the FE parameters (center, evecs, radii)
    print('Generating flight envelope plot.')
    fe_params = get_fe_params(data_list[0]) # Get fe based on first ellipsoid message
    fig, axh = pu.plot_flight_envelope(data_list, fe_params)
    if export_path is not None:
        pu.save_figure_3d(export_path+'/flight_envelope', fig)


def generate_flight_path(data_list, export_path):
    fig, axh = pu.plot_path(data_list)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/uav_path', fig)



@click.command()
@click.option(
    '-d',
    '--log-directory',
    help='Directory with the .pickle log files. Specific files are expected to be found.'
)
@click.option(
    "-m",
    "--model-name",
    default="skywalker_2013_mod",
    help="last_letter_lib UAV model name"
)
@click.option(
    '-i',
    '--plot-index',
    default=None,
    type=click.INT,
    help='Index number of figure to generate. Set to 0 to generate all figures.'
)
@click.option(
    "-e",
    "--export-path",
    default='./figures',
    help="Path to export relevant data in .csv files",
)
def test_code(log_directory, model_name, plot_index, export_path):

    log_directory = os.path.expanduser(log_directory)
    export_path = os.path.expanduser(export_path)

    data_list = build_data_list(log_directory)

    print('Selecting plot for plot_index {}...'.format(plot_index))
    if plot_index == 0:
        generate_all_figures(data_list, export_path)
    elif plot_index == 1:
        generate_flight_envelope(data_list, export_path)
    elif plot_index == 2:
        generate_flight_path(data_list, export_path)
    elif plot_index == 3:
        generate_flight_trajectories(data_list, export_path)
    elif plot_index == 4:
        generate_flight_trajectories_error(data_list, export_path)
    elif plot_index == 5:
        generate_euler(data_list, export_path)
    elif plot_index == 6:
        generate_angular_rates(data_list, export_path)
    elif plot_index == 7:
        generate_angular_rates_error(data_list, export_path)

    print('Done generating ')
    


    ###############
    # Plotting part


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
