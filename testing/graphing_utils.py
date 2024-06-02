
import sys
import numpy as np
from params_utils import Params
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import re
import os

"""
Global constants for the module
"""

# Example: A_star_output_path_key = "(-19.80 + -61.57i)"
output_path_key_format = "a=%.2f, b=%.2f, L=(%.2f + %.2fi)" # from astar.c

# Corresponding regex pattern to match and extract L value from the key
pattern = r"L=\((-?\d+\.\d+) \+ (-?\d+\.\d+)i\)"

# Directory where to save test graph image generated
save_dir = "./graph_images/"

"""
PLOTTING FUNCTIONS: adding plotas with relevant features to an already-existing graph infrastructure
"""

def plot_base_rings(params, ax, ring_color='red'):
    """
    This function uses the g_image computational module to compute the boundaries of obstacles 
    whose parameters are stored in the params dictionary, and adds a plot of the resulting coordinates
    to the given axis object
    """
    for e in params["elliptical_obstacles"]:
        X1, Y1, Z1 = Params.generate_elliptical_ring(e[0], e[1], e[2], e[3], e[4])
        ax.plot(X1, Y1, color=ring_color, linewidth=2)  # Adding the ring-like ellipse
        ax.fill(X1, Y1, color=ring_color, alpha=1.0)    # Fill the ellipse with the specified color


def plot_path(path, ax, color='orange', label=None, linewidth=1):
    """
    Plots a given A* output path on an existing plot.
    
    Parameters:
        path (list of list): The path points as [[x1, y1], [x2, y2], ..., [xn, yn]].
        ax (matplotlib.axes.Axes): The existing axes to plot on.
        color (str): Color of the path line.
        label (str, optional): Label for the path for use in the legend.
        linewidth (int): Width of the line.
    """
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]
    ax.plot(x_coords, y_coords, color=color, marker='o', linestyle='-', markersize=1, linewidth=linewidth, label=label)


def plot_A_star_homotopies_output_paths(params, ax):
    """
    This method uses the regex patterns to recognize the key corresponding to
    each homotopy class, and plots all paths belonging to that homotopy class 
    under the same color, with a legend showing the correspondence between
    colors and L-values.
    """
    L_groups = {}
    for key, path in params.items():
        match = re.search(pattern, key)
        if match:
            real_part = match.group(1)   # Real part of the complex number
            imag_part = match.group(2)   # Imaginary part of the complex number
            L_value = f"{real_part} + {imag_part}i"  # Format to include both parts
            if L_value not in L_groups:
                L_groups[L_value] = []
            L_groups[L_value].append(path)  # Store only the path, key no longer needed here
        else:
            print(f"Skipping key with invalid format: {key}")
    # Assign a unique color for each L group
    cmap = matplotlib.colormaps.get_cmap('viridis')
    colors = cmap(np.linspace(0, 1, len(L_groups)))
    color_index = 0
    for L_value, paths in L_groups.items():
        color = colors[color_index]
        first_path = True  # Flag to add label only to the first path of each group
        for path in paths:
            if first_path:
                plot_path(path, ax, color=color, label=f'L={L_value}')  # Label only the first path
                first_path = False  # Reset the flag after the first path is labeled
            else:
                plot_path(path, ax, color=color)  # Label here defaults to None --> not added to plot's legend
        color_index += 1

    # Enlarge line segments in the legend 
    handles, labels = ax.get_legend_handles_labels()
    # Increase the linewidth of legend handles
    new_handles = [plt.Line2D([], [], c=handle.get_c(), marker=handle.get_marker(), linestyle=handle.get_linestyle(),
                          markersize=10, linewidth=4) for handle in handles]  # Adjust markersize and linewidth as needed
    # Now use these new handles in the legend
    ax.legend(handles=new_handles, labels=labels, loc='upper center', bbox_to_anchor=(0.5, -0.15), 
          shadow=True, ncol=3, fontsize='large', handlelength=2)



"""
Main function of the module that generates the graphic instance using Matplotlib
json_params: parameter dictionary extracted from json that contains the paths
params_inst: current parameter instance to compute coarse version of g_image
"""
def generate_final_graph(
        json_params,
        X, Y, g_image_coarse,   # coarse data sets for the contour
        graph_title,
        save_dir=None,        # Add parameter for directory to save the plot
        filename='plot.png',  # Default filename for the saved plot
        input_figsize = (10,8)
    ):    
    # Directory handling
    if save_dir is None:
        save_dir = os.getcwd()  # Use current working directory if none provided
    os.makedirs(save_dir, exist_ok=True) # Ensure the directory exists

    # Set global font properties right from the beginning
    plt.rc('text', usetex=True)
    plt.rc('text.latex', preamble=r'\usepackage{bm}')  # Optional, for bold math (\bm)
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = 'Computer Modern Roman'
    plt.rcParams['font.weight'] = 'bold'  # This sets the weight for non-math text
    plt.rcParams['font.size'] = 12

    # (1) Create main figure instance
    fig = plt.figure(figsize=input_figsize)

    # (2) Create main axis object
    ax = fig.add_subplot(111) # granularily create a set of INDEPENDENT axes

    # (3) Eventually add further independent datasets to global axis object with plotting functions    

    # Create a custom single-color colormap (e.g., blue reversed)
    colors = ["#0000ff", "#ffffff"]  # White to Blue gradient
    cmap = LinearSegmentedColormap.from_list("custom_blue", colors, N=100)

    """ Reverse gray colormap
    # Create a custom single-color colormap (e.g., grayscale reversed)
    colors = ["#000000", "#ffffff"]  # White to Black gradient
    cmap = LinearSegmentedColormap.from_list("custom_gray", colors, N=100)
    """

    # Plot contour with coarse meshgrids and g_image
    contour = ax.contourf(X, Y, np.array(g_image_coarse), cmap=cmap, levels=512, alpha=0.7)
    cbar = plt.colorbar(contour)
    cbar.set_label(r'\LARGE g-value')

    # Plot the rest using the json dictionary
    plot_base_rings(json_params, ax, "black") # Plot obstacles in black
    plot_A_star_homotopies_output_paths(json_params, ax) # Plot all the homotopy paths returned by A*

    ax.set_xlim(json_params['x_min'], json_params['x_max'])
    ax.set_ylim(json_params['y_min'], json_params['y_max'])

    ax.set_xlabel(r'\LARGE X-axis')
    ax.set_ylabel(r'\LARGE Y-axis')
    ax.set_title(r'\Huge ' + graph_title)
    plt.tight_layout()

    # Save the figure to teh desired directory
    full_path = os.path.join(save_dir, filename)
    plt.savefig(full_path)
    plt.close(fig)  # Close the figure to free up memory

    return full_path  # Optional, return the path of the saved file for further use


