#!/usr/bin/python

# NOTE: This file was experimental and not fully working
# It was found that it didn't perform well, because a shifting indicator function would have either:
# 1. Its no-points patched by the _patch_polytope_holes or
# 2. if old no-points are left, they persist and prevent the polytope from going over them

import itertools as it
import time

import numpy as np
import click
import imp
import matplotlib as mpl
import matplotlib.pyplot as plt

import uav_model as mdl  # Import UAV model library
import plot_utils as plu  # Import plotting utilities
import polytope_utils as poly  # Used for SafeConvexPoly class
import trim_script as ts
tcpp = imp.load_source('trimmer', '/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/src/trimmer.py')


mpl.rcParams['pdf.fonttype'] = 42

@click.command()
@click.option(
    "-p", "--plot", is_flag=True, help="Enable plotting. Only for 2D and 3D problems."
)
@click.option(
    "-i",
    "--interactive",
    is_flag=True,
    help="Wait for user input after each plot refresh",
)
@click.option(
    "-o",
    "--optimizer",
    type=click.Choice(["scipy", "nlopt", "nlopt_cpp"]),
    default="nlopt",
    help="Select optimizer backend."
)
@click.option(
    "-e",
    "--export",
    is_flag=True,
    help="Export relevant data in .csv files"
)
def test_code(plot, interactive, optimizer, export):
    trimmer = ts.Trimmer(optim_lib=optimizer)

    # Provide default values
    phi_des = np.deg2rad(0)
    theta_des = np.deg2rad(0)
    Va_des = 15
    alpha_des = np.deg2rad(0)
    beta_des = np.deg2rad(0)
    r_des = np.deg2rad(5)
    list_defaults = [phi_des, theta_des, Va_des, alpha_des, beta_des, r_des]

    # Set search degrees of freedom
    # Theta, Va, alpha
    fix_phi = True
    fix_theta = False
    fix_Va = False
    fix_alpha = True
    fix_beta = True
    fix_r = True

    list_fixed = [fix_phi, fix_theta, fix_Va, fix_alpha, fix_beta, fix_r]

    # Provide domain bounds
    phi_domain = (np.deg2rad(-45), np.deg2rad(45))
    theta_domain = (np.deg2rad(-25), np.deg2rad(60))
    Va_domain = (5, 30)
    alpha_domain = (np.deg2rad(-10), np.deg2rad(25))
    beta_domain = (np.deg2rad(-5), np.deg2rad(5))
    r_domain = [-0.5, 0.5]

    domain = np.array(
        [phi_domain, theta_domain, Va_domain, alpha_domain, beta_domain, r_domain]
    )
    current_domain = domain[np.invert(list_fixed)]
    n_dim = len(current_domain)

    # Set trimmer optimization bounds
    trimmer.bound_phi = phi_domain
    trimmer.bound_theta = theta_domain
    trimmer.bound_Va = Va_domain
    trimmer.bound_alpha = alpha_domain
    trimmer.bound_beta = beta_domain
    trimmer.bound_r = r_domain

    # Create indicator function
    indicator = trimmer.get_indicator(list_defaults, list_fixed)

    # Select eps values
    eps_phi = np.deg2rad(3)
    eps_theta = np.deg2rad(3)
    eps_Va = 2
    eps_alpha = np.deg2rad(3)
    eps_beta = np.deg2rad(2)
    eps_r = 0.2
    eps = np.array([eps_phi, eps_theta, eps_Va, eps_alpha, eps_beta, eps_r])
    current_eps = eps[np.invert(list_fixed)]

    # Initialize starting time
    t_start = time.time()

    # Initialize the polytope
    safe_poly = poly.SafeConvexPolytope(indicator, current_domain, current_eps)

    # Pass the variable strings
    string_phi = 'Phi'
    string_theta = 'Theta'
    string_Va = 'Va'
    string_alpha = 'Alpha'
    string_beta = 'Beta'
    string_r = 'r'
    string_list = [string_phi, string_theta, string_Va, string_alpha, string_beta, string_r]
    safe_poly.axis_label_list = list(it.compress(string_list, np.invert(list_fixed)))
    # safe_poly.plotting_mask = [False, True, True, True]

    # User interface options
    if plot:
        safe_poly.enable_plotting = True
    if interactive:
        safe_poly.wait_for_user = True

    # Iteratively sample the polytope
    print("Progressive sampling of the polytope")
    algorithm_end = False
    while not algorithm_end:
        print("New safe convex polytope iteration")
        algorithm_end = safe_poly.step()

    print("Final number of sampled points: {}".format(len(safe_poly._set_points)))
    print("Total number of samples taken: {}".format(safe_poly._samples_taken))
    print("Total optimization time required: {}".format(trimmer.optim_time_accumulator))

    # Approximate the safe convex polytope with k vertices
    print("Performing clustering")
    # cluster_num = 2 ** n_dim
    # cluster_num = 2 * (n_dim-3)
    if n_dim == 2:
        cluster_num = 4
    elif n_dim == 3:
        cluster_num = 8
    elif n_dim == 4:
        cluster_num = 8
    elif n_dim == 5:
        cluster_num = 8
    elif n_dim == 6:
        cluster_num = 9

    safe_poly.cluster(cluster_num)

    t_end = time.time()
    print("Total script time: {}".format(t_end - t_start))

    # Print human-readable polytope equations
    unscaled_polytope = safe_poly._reduced_polytope.scale(safe_poly.eps.reshape(safe_poly._n_dim, 1))
    eqns = unscaled_polytope.get_equations()
    divisor = max(np.abs(eqns).min(), 1)
    eqns = eqns/divisor
    # Switch coefficient signs where appropriate to make the hyperplane equation<0 sastisfy the polytope
    centroid = safe_poly.get_centroid(unscaled_polytope.get_points())
    for i, eqn in enumerate(eqns):
        sign = -np.sign(poly.evaluate_hyperplane(centroid, eqn))[0,0]
        eqns[i, :] = sign*eqn

    print("Polytope equations ({}):".format(eqns.shape[0]))
    header = list(safe_poly.axis_label_list)
    header.append('Offset')
    print(''.join('{:>15s}'.format(v) for v in header))
    for eqn in eqns:
        line_string = ''
        for elem in eqn:
            line_string += '{:>15.1f}'.format(elem)
        line_string += '{:>5s}'.format('<0')
        print(line_string)

    print('C-type definition:')
    print(ts.print_c_arrays(eqns))

    if plot:
        print("Plotting")
        safe_poly.plot()
        plt.show()


    if export:
        
        unscaled_yes_points = safe_poly.points_yes*safe_poly.eps.reshape(safe_poly._n_dim, 1)
        unscaled_no_points = safe_poly.points_no*safe_poly.eps.reshape(safe_poly._n_dim, 1)
        unscaled_polytope = safe_poly._polytope.scale(safe_poly.eps.reshape(safe_poly._n_dim, 1))
        eqns = unscaled_polytope.get_equations()
        divisor = max(np.abs(eqns).min(), 1)
        eqns = eqns/divisor
        # Switch coefficient signs where appropriate to make the hyperplane equation<0 sastisfy the polytope
        centroid = safe_poly.get_centroid(unscaled_polytope.get_points())
        for i, eqn in enumerate(eqns):
            sign = -np.sign(poly.evaluate_hyperplane(centroid, eqn))[0,0]
            eqns[i, :] = sign*eqn
        vertices = unscaled_polytope.get_points()
        print(vertices)

        output_path = '/home/george/ros_workspaces/uav_ftc/src/uav_ftc/logs/flight_envelope_va_gamma'
        header_txt = ','.join(safe_poly.axis_label_list)
        np.savetxt('{}/points_yes.csv'.format(output_path), unscaled_yes_points.transpose(), fmt='%.3g', delimiter=',', header=header_txt, comments='')
        np.savetxt('{}/points_no.csv'.format(output_path), unscaled_no_points.transpose(), fmt='%.3g', delimiter=',', header=header_txt, comments='')
        np.savetxt('{}/separators.csv'.format(output_path), eqns, fmt='%15.1f', delimiter=',', header=(header_txt+',Offset'), comments='')
        np.savetxt('{}/vertices.csv'.format(output_path), vertices.transpose(), fmt='%15.1f', delimiter=',', header=header_txt, comments='')

    # # Static Flight envelope construction
    # domain = {
    #     "Va": np.linspace(5, 30, 8),
    #     "alpha": np.linspace(np.deg2rad(-10), np.deg2rad(45), 8),
    #     "beta": [0],
    #     "phi": [0],
    #     "theta": np.linspace(np.deg2rad(-45), np.deg2rad(45), 8),
    #     "r": [0],
    # }

    # flight_envelope = FlightEnvelope(domain, trimmer)
    # flight_envelope.find_static_trim()
    # # plt_figure = flight_envelope.static_trim.xyz.heatmap(x='Va', y='alpha', z='delta_t', colors=True, colormap='viridis')
    # # flight_envelope.static_trim.xyz.heatmap(x='Va', y='alpha', z='delta_t', colors=True, colormap='viridis')

    # # Create a new flight_envelope with only feasible trim points
    # ds_f = build_feasible_fe(flight_envelope.static_trim)

    # # Create the convex hull of the flight envelope
    # convex_hull = build_convex_hull(ds_f)
    # # Plot a 3D slice
    # plu.plot3_points(convex_hull.points[:, [0, 1, 4]], ["Va", "alpha", "theta"])

    # # Build the Va, R, gamma variables for each trim point
    # flight_envelope.nav_trim = find_nav_trim(convex_hull.points)
    # # Plot the navigation envelope
    # plu.plot3_points(flight_envelope.nav_trim, ["Va", "gamma", "R"])

if __name__ == "__main__":
    test_code()
