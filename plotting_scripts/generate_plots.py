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


uav_name = 'skywalker_2013_mod'  # UAV name to use for FE extraction

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


def build_dataset(log_directory):

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

    dataset = dict()

    for filename in file_names:
        log_path = os.path.join(log_directory, filename + '.pickle')
        if os.path.exists(log_path):
            print('Adding {} log file.'.format(log_path))
            log_file = load_pickle_data(log_path)
            log_file = filter_log_data(log_file, t_start, t_end)
            dataset[filename] = log_file

    return dataset


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


parameters_for_randomization = [
    (3, 'm', 2.0),
    (3, 'j_x', 0.8244),
    (3, 'j_y', 1.135),
    (3, 'j_z', 1.759),
    (3, 'j_xz', 0.1204),

    (4, 'airfoil1/s', 0.45),
    (4, 'airfoil1/b', 1.88),
    (4, 'airfoil1/c', 0.24),
    (4, 'airfoil1/c_L_0', 0.4),
    (4, 'airfoil1/c_L_deltae', 0.0),
    (4, 'airfoil1/c_L_alpha', 6.5),
    (4, 'airfoil1/c_L_qn', 0),
    (4, 'airfoil1/mcoeff', 50),
    (4, 'airfoil1/oswald', 0.9),
    (4, 'airfoil1/alpha_stall', 0.4712),
    (4, 'airfoil1/c_D_qn', 0),
    (4, 'airfoil1/c_D_deltae', 0.0),
    (4, 'airfoil1/c_D_0', 0.09),
    (4, 'airfoil1/c_D_alpha', 0.14),
    (4, 'airfoil1/c_Y_0', 0),
    (4, 'airfoil1/c_Y_beta', -0.98),
    (4, 'airfoil1/c_Y_pn', 0),
    (4, 'airfoil1/c_Y_rn', 0),
    (4, 'airfoil1/c_Y_deltaa', 0),
    (4, 'airfoil1/c_Y_deltar', -0.2), #opposite sign than c_n_deltar
    (4, 'airfoil1/c_l_0', 0),
    (4, 'airfoil1/c_l_pn', -1.0),
    (4, 'airfoil1/c_l_beta', -0.12),
    (4, 'airfoil1/c_l_rn', 0.14),
    (4, 'airfoil1/c_l_deltaa', 0.25),
    (4, 'airfoil1/c_l_deltar', -0.037),
    (4, 'airfoil1/c_m_0', 0.01),
    (4, 'airfoil1/c_m_alpha', -1.3),
    (4, 'airfoil1/c_m_qn', -20),
    (4, 'airfoil1/c_m_deltae', 1.0),
    (4, 'airfoil1/c_n_0', 0),
    (4, 'airfoil1/c_n_beta', 0.25),
    (4, 'airfoil1/c_n_pn', 0.022),
    (4, 'airfoil1/c_n_rn', -1),
    (4, 'airfoil1/c_n_deltaa', 0.00),
    (4, 'airfoil1/c_n_deltar', 0.1), #opposite sign than c_y_deltar
    (4, 'airfoil1/deltaa_max', 0.3491),
    (4, 'airfoil1/deltae_max', 0.3491),
    (4, 'airfoil1/deltar_max', 0.3491),
    (4, 'airfoil1/deltaa_max_nominal', 0.3491),
    (4, 'airfoil1/deltae_max_nominal', 0.3491),
    (4, 'airfoil1/deltar_max_nominal', 0.3491),

    (5, 'motor1/propDiam', 0.28),
]


def randomize_parameter(fe, parameter_tuple):
    param_type, param_name, param_value = parameter_tuple
    sign = np.random.choice(np.array([-1, 1]))

    change_prct = 0.05
    if param_value == 0:
        new_value = sign*0.05
    else:
        new_value = param_value + sign*param_value*change_prct

    result = fe.set_model_parameter( param_type, param_name, new_value)  
    if result:
        print("Succeeded changing {}".format(param_name))
    else:
        print("Failed changing {}".format(param_name))


def randomize_parameter_list(fe, parameter_list):
    for parameter_tuple in parameter_list:
        randomize_parameter(fe, parameter_tuple)


def get_flight_envelope(model_name, fault_idx=0):
    # TODO: Deprecate fault_idx argument
    flight_envelope = fe.FlightEnvelope(model_name)

    # Set the limits of the search domain
    flight_envelope.set_domain_Va((5, 25))
    flight_envelope.set_domain_gamma((-30, 30))
    flight_envelope.set_R_min(100)
    flight_envelope.initialize(model_name)

    flag_plot_points = True
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


# Plot Flight envelope as freshly calculated offline
def generate_theoretical_flight_envelope(dataset, export_path):
    # Get the FE parameters (center, evecs, radii)
    print('Generating theoretical flight envelope plot.')
    safe_poly = get_flight_envelope(uav_name)
    center, evecs, radii, _ = get_fe_ellipsoid(safe_poly)
    fe_params = (center, evecs, radii)
    fig, axh = pu.plot_flight_envelope(dataset, (fe_params,))
    if export_path is not None:
        pu.save_figure_3d(export_path+'/flight_envelope_theoretical', fig)


# Plot Flight envelope as logged
def generate_logged_flight_envelope(dataset, export_path):
    # Get the FE parameters (center, evecs, radii)
    print('Generating logged flight envelope plot.')
    fe_params = get_fe_params(dataset['nominal']) # Get fe based on first ellipsoid message
    fig, _ = pu.plot_flight_envelope(dataset, (fe_params,))
    if export_path is not None:
        pu.save_figure_3d(export_path+'/flight_envelope_logged', fig)


def generate_flight_path(dataset, export_path):
    fig, _ = pu.plot_path(dataset)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/uav_path', fig)


def generate_flight_trajectories(dataset, export_path):
    fig, _ = pu.plot_trajectories(dataset)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/trajectory_components', fig)


def generate_flight_trajectories_error(dataset, export_path):
    fig, _ = pu.plot_trajectories_errors(dataset)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/trajectory_error_components', fig)


def generate_euler(dataset, export_path):
    fig, _ = pu.plot_euler(dataset)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/euler', fig)


def generate_angular_rates(dataset, export_path):
    fig, _ = pu.plot_angular_rates(dataset)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/angular_rates', fig)


def generate_angular_rates_error(dataset, export_path):
    fig, _ = pu.plot_angular_rates_errors(dataset)
    if export_path is not None:
        pu.save_figure_2d(export_path+'/angular_rates_errors', fig)


def generate_all_figures(dataset, export_path):
    for func in plot_functions_list:
        func(dataset, export_path)


plot_functions_list = [
    generate_all_figures,
    generate_theoretical_flight_envelope,
    generate_logged_flight_envelope,
    generate_flight_path,
    generate_flight_trajectories,
    generate_flight_trajectories_error,
    generate_euler,
    generate_angular_rates,
    generate_angular_rates_error,
]

def plot_functions_help_msg():
    msg = ''
    for i, func in enumerate(plot_functions_list):
        new_str = '{}: {}\n'.format(i, func.__name__)
        msg += new_str
    return msg


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
    help='Index number of figure to generate:\n'+plot_functions_help_msg()
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

    dataset = build_dataset(log_directory)

    print('Selecting plot for plot_index {}...'.format(plot_index))
    plot_functions_list[plot_index](dataset, export_path)
    print('Done generating ')
    

if __name__ == "__main__":
    test_code()
