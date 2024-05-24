
import sys
import numpy as np
import g_image as gi
import matplotlib
import matplotlib.pyplot as plt
import re

"""
Global constants for the module
"""

# Example: A_star_output_path_key = "(-19.80 + -61.57i)"
output_path_key_format = "a=%.2f, b=%.2f, L=(%.2f + %.2fi)" # from astar.c

# Corresponding regex pattern to match and extract L value from the key
pattern = r"L=\((-?\d+\.\d+) \+ (-?\d+\.\d+)i\)"

"""
PLOTTING FUNCTIONS: adding plotas with relevant features to an already-existing graph infrastructure
"""

def plot_base_rings(params, ax, ring_color='red'):
    """
    This function uses the g_image computational module to compute the boundaries of obstacles 
    whose parameters are stored in the params dictionary, and adds a plot of the resulting coordinates
    to the given axis object
    """
    for e in params["elliptical_obstacles"]:  # NOTE: hard-code z_min = 0 and z_height = 0 here to impose the 2-d case
        X1, Y1, Z1 = gi.generate_elliptical_ring(e[0], e[1], e[2], e[3], e[4])
        ax.plot(X1, Y1, Z1, color=ring_color, linewidth=2)  # Adding the ring-like ellipse

def plot_path(path, ax, z_height=0, color='orange', label='A* Path'):
    """
    Plots a given A* output path on an existing 3D plot.
    
    Parameters:
        path (list of list): The path points as [[x1, y1], [x2, y2], ..., [xn, yn]].
        z_height (float): The height at which to plot the path.
        ax (matplotlib.axes.Axes3D): The existing 3D axes to plot on.
        color (str): Color of the path line.
    """
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]
    z_coords = [z_height] * len(path)
    ax.plot(x_coords, y_coords, z_coords, color=color, marker='o', linestyle='-', markersize=1, linewidth=1, label=label)

def plot_A_star_homotopies_output_paths(params, ax, z_height=0):
    """
    This method uses the regex patterns to recognize the key corresponding to
    each homotopy class, and plots all paths belonging to that homotopy class 
    under the same color. 
    """
    L_groups = {} # Group paths by the L value
    for key, path in params.items():
        match = re.search(pattern, key)
        if match:
            L_value = match.group(1)
            if L_value not in L_groups:
                L_groups[L_value] = []
            L_groups[L_value].append((key, path))
        else:
            print(f"Skipping key with invalid format: {key}")
    # Assign a unique color for each L group
    cmap = matplotlib.colormaps.get_cmap('viridis')
    colors = cmap(np.linspace(0, 1, len(L_groups)))
    color_index = 0
    for L_value, paths in L_groups.items():
        color = colors[color_index]
        for key, path in paths:
            plot_path(path, ax, z_height=z_height, color=color, label=key)
        color_index += 1

"""
Main function of the module that generates the graphic instance using Matplotlib
"""
def generate_final_graph(
        params,
        graph_title,
        coarsing_factor = 10,
        z_lim = (0, 5), 
        input_figsize = (10,8)
    ):    

    # (1) Create main figure instance
    fig = plt.figure(figsize=input_figsize)

    # (2) Create main axis object
    ax = fig.add_subplot(111, projection='3d') # granularily create a set of INDEPENDENT axes

    # (3) Eventually add further independent datasets to global axis object 
    plot_base_rings(params, ax, "red") # Plot obstacles
    plot_A_star_homotopies_output_paths(params, ax) # Plot all the homotopy paths returned by A*
    
    # NOTE: Recompute separate coarser version of g_image to be graphed
    X, Y = gi.compute_meshgrids(params, coarsing_factor)
    g_image_coarse = gi.compute_g_image(params, X, Y)
    ax.plot_surface(X, Y, np.array(g_image_coarse), rstride=1, cstride=1, alpha=0.7) # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids

    # Set axis limits (inputs)
    ax.set_xlim(params['x_min'], params['x_max'])
    ax.set_ylim(params['y_min'], params['y_max'])
    ax.set_zlim(z_lim[0], z_lim[1])

    # Label the axes
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    # Title and showing the plot
    ax.set_title(graph_title)
    plt.tight_layout()
    plt.show()

"""
NOTE: This module assumes that the input .json file has already been populated with the g_image matrix  
"""
def main():
    # Argument check
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_filepath> <graph_title>")
        sys.exit(1)  # Exit the program indicating an error

    input_filepath = sys.argv[1]
    graph_title = sys.argv[2]
    print(f"The input file path is: {input_filepath}")
    params = gi.load_parameters(input_filepath)
    generate_final_graph(params, graph_title)

if __name__ == '__main__':
    main()

