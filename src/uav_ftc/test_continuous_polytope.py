#!/usr/bin/python

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import click

import polytope_utils as pu

mpl.rcParams['pdf.fonttype'] = 42

# Answer if p is inside the hypersphere c, r
def hypersphere(p_c, p_r):
    def indicator(p):
        c = p_c * np.ones(p.shape)
        r = p_r
        distance = np.linalg.norm(p - c)
        return distance <= r
    return indicator

def shape_center(iteration):
    T = 50
    w = 2*np.pi/T
    mean = 3
    amplitude = 1
    return amplitude*np.sin(w*iteration) + mean

def shape_radius(iteration):
    T = 50
    w = 2*np.pi/T
    mean = 3
    amplitude = 2
    return amplitude*np.sin(w*iteration) + mean


@click.command()
@click.option(
    "-d", "--dimensions", default=2, type=int, help="Dimensions of the problem"
)
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
    "-w",
    "--write-figures",
    is_flag=True,
    help="Save each plotted figure"
)
def test_code(dimensions, plot, interactive, write_figures):

    n_dim = dimensions
    print("Testing polytope code for {} dimensions".format(n_dim))
    flag_plot = plot

    # define the indicator function to approximate
    indicator_center = shape_center(0)
    indicator_radius = shape_radius(0)
    ind_func = hypersphere(indicator_center, indicator_radius)

    # print(ind_func)
    # print(ind_func())
    # return

    # define a domain as a n_dim x 2 array
    print("Creating problem domain and sampling initial points")
    domain = np.zeros((n_dim, 2))

    domain[:, 0] = -3
    domain[:, 1] = 10

    # Set the desired precision
    eps = 1 * np.ones(n_dim)

    # Initialize the polytope
    safe_poly = pu.SafeConvexPolytope(ind_func, domain, eps)
    # safe_poly.set_eps(eps)
    safe_poly.set_sampling_method("radial")
    # safe_poly._patch_polytope_holes = False

    if flag_plot:
        safe_poly.enable_plotting = True
    if interactive:
        safe_poly.wait_for_user = True
    if write_figures:
        safe_poly.save_figures = True

    # Iteratively sample the polytope until convergence
    print("Progressive sampling of the polytope")
    flag_convergence = False
    while not flag_convergence:
        print("New safe convex polytope iteration")
        flag_convergence = safe_poly.step()

    # Start moving the indicator function
    iteration = 0
    max_iteration = 5000
    safe_poly._delete_old_points = True
    while iteration<max_iteration:
        new_center = shape_center(iteration)
        new_radius = shape_radius(iteration)
        safe_poly.indicator = hypersphere(new_center, new_radius)
        safe_poly.step()

        # Approximate the safe convex polytope with k vertices
        # print("Performing clustering")
        # safe_poly.cluster(2 ** n_dim)

        if flag_plot:
            print("Plotting")
            safe_poly.plot()
            # plt.show()
        
        iteration += 1

if __name__ == "__main__":
    test_code()
