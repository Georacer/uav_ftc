#!/usr/bin/env python

import numpy as np
import click
import matplotlib as mpl
import matplotlib.pyplot as plt

import uav_ftc.trim_traj_fe as fe

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

    flight_envelope = fe.FlightEnvelope(model_name)

    # Set the limits of the search domain
    flight_envelope.set_domain_Va((5, 25))
    flight_envelope.set_domain_gamma((-30, 30))
    flight_envelope.set_R_min(100)
    flight_envelope.initialize(model_name)

    flag_plot_points = False
    if fault_idx == 0:
        print('Flight Envelope for the nominal model')
        img_name = 'nominal'
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
        img_name = 'fault_motor'

    elif fault_idx == 2:
        print('Inducing aileron fault')
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_l_deltaa", 0.125)  # 50% of the original  
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes
        img_name = 'fault_aileron'

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
        img_name = 'fault_flap'

    elif fault_idx == 4:
        print('Inducing parameter variations')
        randomize_parameter_list(flight_envelope, parameters_for_randomization)  # Batch randomization
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = False  # FE too small and points hide it
        img_name = 'param_variation'


    # Calculate flight envelope
    safe_poly = flight_envelope.find_flight_envelope()
    ##########################
    # Approximate by ellipsoid
    center, evecs, radii, v = safe_poly.ellipsoid_fit()
    print("Fitted Ellipsoid coefficients:\n")
    print('Center')
    print(center)
    print('EVecs')
    print(evecs)
    print('Radii')
    print(radii)
    print('Coeffs')
    print(v)

    safe_poly._plot_points = flag_plot_points
    safe_poly.plot()
    #safe_poly.plot_ellipsoid()

    # Save figures
    safe_poly._figure_handle.savefig('fe_{}.pdf'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)
    # plt.waitforbuttonpress(timeout=-1)

    safe_poly._axis_handle.view_init(0,0)
    safe_poly._figure_handle.savefig('fe_{}_0_0.pdf'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)

    safe_poly._axis_handle.view_init(-90,0)
    safe_poly._figure_handle.savefig('fe_{}_90_0.pdf'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)

    safe_poly._axis_handle.view_init(0,-90)
    safe_poly._figure_handle.savefig('fe_{}_0_90.pdf'.format(img_name), bbox_inches='tight')
    plt.pause(0.01)

    ##########################
    # Approximate by ellipsoid
    center, evecs, radii, v = safe_poly.ellipsoid_fit()
    print("Fitted Ellipsoid coefficients:\n")
    print('Center')
    print(center)
    print('EVecs')
    print(evecs)
    print('Radii')
    print(radii)
    print('Coeffs')
    print(v)


    ######################################################
    # Approximate the safe convex polytope with k vertices
    # print("Performing clustering")
    # # cluster_num = 2 ** n_dim
    # # cluster_num = 2 * (n_dim-3)
    # if safe_poly._n_dim == 2:
    #     cluster_num = 4
    # elif safe_poly._n_dim == 3:
    #     cluster_num = 12

    # safe_poly.cluster(cluster_num)
    # # if plot:
    # #     safe_poly.plot()
    # #     plt.show()
    # safe_poly.display_constraints()

    # print("C-type definition:")
    # print(safe_poly.print_c_arrays())

    # # Export polytope search results
    # if export_path is not None:
    #     safe_poly.export_csv(export_path)


if __name__ == "__main__":
    test_code()
