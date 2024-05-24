
import numpy as np
import json 
import sys

# t & theta parameters
t_min = 0
t_max = 2 * np.pi
theta_max = 4
theta_min = 1

# Key constant strings 
g_image_key = "g_image"

def load_parameters(filename):
    """
    This function extracts a params dictionary from the input .json file and returns it
    """
    with open(filename, 'r') as file:
        params = json.load(file)
    return params

def save_to_json(g_image, filename):
    """
    Given a freshly computed numpy array g_image, and filepath,
    this function writes the computed values for g_image into the json file
    """
    with open(filename, 'r+') as file:
        params = json.load(file) # retrieve current file params
        params[g_image_key] = g_image.tolist()  # Convert numpy array to list for JSON serialization, and add new dictionary entry to params
        file.seek(0)  # Rewind file to the beginning
        json.dump(params, file, indent=4)
        file.truncate()  # Truncate file to the new size

def compute_meshgrids(params, coarsing_factor=1):
    """
    Given input ranges for x,y stored in params, this function
    converts them to numpy 2-d arrays for processing, with a coarsing factor arbitrarily reducing resolution
    """
    # Computing linear 1-D arrays for x,y depending on the given step and range values
    x_steps = (int)((params['x_max'] - params['x_min']) / (coarsing_factor * params['r']))
    y_steps = (int)((params['y_max'] - params['y_min']) / (coarsing_factor * params['r']))
    x = np.linspace(params['x_min'], params['x_max'], x_steps)
    y = np.linspace(params['y_min'], params['y_max'], y_steps)

    X, Y = np.meshgrid(x, y)  # generates matrices for parameter space
    return X,Y

def generate_elliptical_surface(X, Y, x_0, y_0, a, b, alpha, z_min=0):
    """
    This function generates an ellipse characterized by the given parameters, 
    applying the key equation to convert from the (t, theta) to the (x, y) representation,
    and returns the corresponding numpy 2-d array Z corresponding to the meshgrids X, Y given
    """
    # Define root of elliptical cone, using the desired ground level for the ellipse 
    z_0 = z_min - (alpha * theta_min)
    # Compute the 2-D matrix for the z-values corresponding to each (x,y) pair 
    return np.maximum(0, z_0 + alpha * np.sqrt(((X - x_0) / a)**2 + ((Y - y_0) / b)**2))

def compute_g_image(params, X, Y):
    """
    This function applies generate_elliptical_surface using the input data for the obstacles (in params dictionary)
    to compute the comprehensive image of g. It returns the corresponding 2-d numpy array
    """
    # Iterate through the input ellipses
    Z_list = [] # list holding the 2-dimensional numpy arrays corresponding to z-coordinates for each obstacle
    for e in params["elliptical_obstacles"]: # NOTE: hard-code z_min = 0 here to impose the 2-d case
        ZS = generate_elliptical_surface(X, Y, e[0], e[1], e[2], e[3], e[4])
        Z_list.append(ZS) # add the newly generated ellipse coordinates to the running list
    # Return the resulting values for the uncertainty function in Z
    Z = np.full_like(Z_list[0], params['g_max']) # fill-up Z with g_max
    # Iterate through the list, and keep a running minimum
    for Z0 in Z_list:
        Z = np.minimum(Z, Z0)
    return Z

def generate_elliptical_ring(x_0, y_0, a, b, alpha, z_height=0, z_min=0, t_steps=100, theta_steps=100):
    """
    This function generates a cross sectional ring corresponding to the ellipse parameters,
    plus a parameter giving the desired height of the ring,
    and returns a 3-tuple (X, Y, Z) of the 1d arrays necessary to graph the ring
    """
    z_0 = z_min - (alpha * theta_min)   # First retrieve the z_0 corresponding to the input elliptical surface
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

"""
This module's main function has the objective to take a .json file from the user
and use the information in the file to compute g, and write
the corresponding results for the image of g to the file itsels as a 2x2 matrix
"""
def main():
    # Check if exactly one argument (besides the program name) is given
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <input_filepath>")
        sys.exit(1)  # Exit the program indicating an error

    input_filepath = sys.argv[1]
    print(f"The input file path is: {input_filepath}")
    params = load_parameters(input_filepath)
    X,Y = compute_meshgrids(params)
    g_image = compute_g_image(params, X, Y)
    save_to_json(g_image, input_filepath)


if __name__ == '__main__':
    main()

