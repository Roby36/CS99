
import numpy as np
import json 

# t & theta parameters
t_min = 0
t_max = 2 * np.pi
theta_max = 4
theta_min = 1

# .json filename to extract data from 
input_filepath = "environment2 copy.json"

# Function extracting envirnoment parameters from .json file
def load_parameters(filename):
    with open(filename, 'r') as file:
        params = json.load(file)
    return params

# Function to write back to .json simulated uncertainty values, after being added to the params variable 
def save_to_json(g_image, filename):
    with open(filename, 'r+') as file:
        params = json.load(file) # retrieve current file params
        params['g_image'] = g_image.tolist()  # Convert numpy array to list for JSON serialization, and add new dictionary entry to params
        file.seek(0)  # Rewind file to the beginning
        json.dump(params, file, indent=4)
        file.truncate()  # Truncate file to the new size

# This function generates an ellipse characterized by the given parameters, returning the 
# resulting Z 2d array correcponding to the global X, Y grids 
def generate_elliptical_surface(X, Y, x_0, y_0, a, b, z_min, alpha):
    # Define root of elliptical cone, using the desired ground level for the ellipse 
    z_0 = z_min - (alpha * theta_min)
    # Compute the 2-D matrix for the z-values corresponding to each (x,y) pair 
    return np.maximum(0, z_0 + alpha * np.sqrt(((X - x_0) / a)**2 + ((Y - y_0) / b)**2))

# Based on the input obstacles, grid, and uncertainty cap, compute the full image of the uncertainty function g
# NOTE: This function depends on the axis global variable object ax
def compute_g_image(params, X, Y):
    # Iterate through the input ellipses
    Z_list = [] # list holding the 2-dimensional numpy arrays corresponding to z-coordinates for each obstacle
    for e in params["elliptical_obstacles"]: 
        ZS = generate_elliptical_surface(X, Y, e[0], e[1], e[2], e[3], e[4], e[5])
        Z_list.append(ZS) # add the newly generated ellipse coordinates to the running list
    # Return the resulting values for the uncertainty function in Z
    Z = np.full_like(Z_list[0], params['g_max']) # fill-up Z with g_max
    # Iterate through the list, and keep a running minimum
    for Z0 in Z_list:
        Z = np.minimum(Z, Z0)
    return Z

# This function generates a cross sectional ring corresponding to the ellipse parameters,
# plus a parameter giving the desired height of the ring,
# and returns a 3-tuple (X, Y, Z) of the 1d arrays necessary to graph the ring
def generate_elliptical_ring(x_0, y_0, a, b, z_min, alpha, z_height, t_steps=100, theta_steps=100):
    # First retrieve the z_0 corresponding to the input elliptical surface 
    z_0 = z_min - (alpha * theta_min)
    t     = np.linspace(t_min,     t_max,     t_steps)      # t parameter from 0 to 2*pi, in discrete step in ndarray
    theta = np.linspace(theta_min, theta_max, theta_steps)  # theta parameter from 0 to theta_max, in discrete steps in ndarray
    ### NOTE: Because the theta array can take no values below theta_min, defining z at any value below z_min yields out-of-range errors
    theta_fixed = (z_height - z_0)/alpha # determine the corresponding value for the scale factor theta 
    step_fixed  = (int) ((theta_steps / (theta_max - theta_min)) * (theta_fixed - theta_min) + 1) # determine corresponding step in the theta progression
    ### NOTE: '+1' added for indexing issues
    X2 = x_0 + (theta[step_fixed] * a) * np.cos(t)  # Selecting the FIRST ROW of T ONLY, thus a complete cycle, for a single ellipse
    Y2 = y_0 + (theta[step_fixed] * b) * np.sin(t)  # Selecting appropriate Theta row (array of constant desired Theta value)
    ### NOTE: X2, Y2 are ONE dimensional arrays
    Z2 = np.full_like(X2, z_height) # Create a 2-d numpy array of the same dimension as X2,Y2, but filled with z_fixed 
    return (X2, Y2, Z2)

# Iterate throughout the given obstacle parameters to plot the base rings for each obstacle; 
# NOTE: Requires input axes object on which to plot the rings
def plot_base_rings(params, input_axes, z_height=0, ring_color='red'):
    for e in params["elliptical_obstacles"]: 
        X1, Y1, Z1 = generate_elliptical_ring(e[0], e[1], e[2], e[3], e[4], e[5], z_height)
        input_axes.plot(X1, Y1, Z1, color=ring_color, linewidth=2)  # Adding the ring-like ellipse

# Main routine handling ALL graphing procedures:
def generate_surface_graph(params, X, Y, g_image,
                           z_lim = (0, 5), 
                           input_figsize = (10,8)):
    # Function-local import statement 
    import matplotlib.pyplot as plt

    # (1) Create figure object
    fig = plt.figure(figsize=input_figsize)

    # (2) Create axis object
    ax = fig.add_subplot(111, projection='3d') # granularily create a set of INDEPENDENT axes

    # (3) Eventually add further independent datasets to global axis object 
    plot_base_rings(params, ax) # Plot the base rings & surface
    ax.plot_surface(X, Y, g_image, rstride=1, cstride=1, alpha=1.0) # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids

    # Set axis limits (inputs)
    ax.set_xlim(params['x_min'], params['x_max'])
    ax.set_ylim(params['y_min'], params['y_max'])
    ax.set_zlim(z_lim[0], z_lim[1])

    # Label the axes
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    # Title and showing the plot
    ax.set_title('3D Parametric Surface of Ellipses')
    plt.tight_layout()
    plt.show()


def main():

    # Load parameters from .json file
    params = load_parameters(input_filepath)

    # Coomputing linear 1-D arrays for x,y depending on the given step and range values
    x_steps = (int)((params['x_max'] - params['x_min']) / params['r'])
    y_steps = (int)((params['y_max'] - params['y_min']) / params['r'])
    x = np.linspace(params['x_min'], params['x_max'], x_steps)
    y = np.linspace(params['y_min'], params['y_max'], y_steps)

    # Create the meshgrid for the parameters x and y
    # NOTE: We use X, Y as global variables representing the robot grid throughout the entire module
    X, Y = np.meshgrid(x, y)    # generates matrices for parameter space

    # Generate and save resulting image of g
    g_image = compute_g_image(params, X, Y)
    save_to_json(g_image, input_filepath)

    # If we want to graph the surface:
    generate_surface_graph(params, X, Y, g_image)


if __name__ == '__main__':
    main()

