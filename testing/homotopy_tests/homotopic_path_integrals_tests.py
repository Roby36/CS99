

import sys
import numpy as np
import elliptical_surface_graph as esg
from scipy.integrate import quad
import cmath
import matplotlib.pyplot as plt
import json

### Hard-ccoded globals here:
z_i = complex(0, 10) # intermediary point for the line segment
s = 0.7 # Linearization factor for circular path
steps_path1    = "test_path_1_steps"
steps_path2    = "test_path_2_steps"
test_path_key1 = "test_path_1" # important to parse from C counterpart
test_path_key2 = "test_path_2" # important to parse from C counterpart
steps = 10000 # precision to which we write the data from the paths


# Define the complex function F(z), in terms of file parameters
def F(z, poles, BL, TR, a, b):
    f0 = (z - BL)**a * (z - TR)**b
    denominator = 1
    for pole in poles:
        denominator *= (z - pole)
    return f0 / denominator

def initialize_globals(params):
    global poles, BL, TR, a, b, z_s, z_g

    N = params["num_obstacles"]
    poles = [complex(params["elliptical_obstacles"][i][0], params["elliptical_obstacles"][i][1]) for i in range(N)]
    BL = complex(params["x_min"], params["y_min"])
    TR = complex(params["x_max"], params["y_max"])

    # We assume f_0 to be a polynomial of order N - 1 */
    a = N // 2
    b = (N - 1) - a

    z_s = complex(params["x_s"], params["y_s"])
    z_g = complex(params["x_g"], params["y_g"])

# Define arbitrary path between some point z_1 and z_2 
def straight_line(t, z_1, z_2):
    return z_1 + t * (z_2 - z_1)

# Now define two-segment path with a given intermediary point, to eventually break through all homotopic classes
def gamma_1(t):
    if (t < 0.5):
        return straight_line(2*t, z_s, z_i)
    elif (t >= 0.5):
        return straight_line(2*(t - 0.5), z_i, z_g)

'''
def gamma_1(t):
    return z_s + t * (z_g - z_s)
'''

def gamma_2(t):
    center = (z_s + z_g) / 2
    radius = abs(z_s - z_g) / 2
    # Calculate the angle where the semicircle should start
    angle_offset = cmath.phase(z_s - center)  # This gives the phase angle of z_s relative to the center
    # Define the semicircular path correctly
    circular_path = center + radius * cmath.exp(1j * (angle_offset + np.pi * t))  # Adjust phase based on t
    linear_path = z_s + t * (z_g - z_s)
    # Interpolated path based on stretch factor s
    return (1 - s) * circular_path + s * linear_path

# Integration wrapper function
def integrate_path(gamma, dt = 1e-6):
    def real_func(t):
        dz = (gamma(t + dt) - gamma(t)) / dt
        return F(gamma(t), poles, BL, TR, a, b) * dz
    re_integral, re_error = quad(lambda t: real_func(t).real, 0, 1)
    im_integral, im_error = quad(lambda t: real_func(t).imag, 0, 1)
    integral = complex(re_integral, im_integral)
    return integral

# Plotting function
def plot_paths_and_poles():
    t_values = np.linspace(0, 1, 10000)
    gamma1_values = [gamma_1(t) for t in t_values]
    gamma2_values = [gamma_2(t) for t in t_values]

    plt.figure(figsize=(8, 6))
    plt.plot([z.real for z in gamma1_values], [z.imag for z in gamma1_values], label='Gamma 1 (Line)', marker='.')
    plt.plot([z.real for z in gamma2_values], [z.imag for z in gamma2_values], label='Gamma 2 (Semicircle)', marker='o')

    # Plot poles
    for p in poles:
        plt.plot(p.real, p.imag, 'rx', markersize=10, label='Pole at {}'.format(p))

    plt.xlabel('Real Part')
    plt.ylabel('Imaginary Part')
    plt.title('Path Integrals Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()

def write_path_to_json(params, input_filepath, steps, gamma, func_name, step_key_name):
    # Generate the parameterized values and compute the path
    t_values = np.linspace(0, 1, steps)
    gamma_values = [gamma(t) for t in t_values]
    # Format the complex path values into (x, y) tuples
    path_data = [(z.real, z.imag) for z in gamma_values]
    # Update the existing JSON data with new path data under the given func_name
    params[func_name]     = path_data
    params[step_key_name] = steps
    # Write the updated dictionary back to the JSON file
    with open(input_filepath, 'w') as file:
        json.dump(params, file, indent=4)
    print(f"Data for {func_name} has been written to {input_filepath}")


# Takes as argument the filepath for the json file storing the parameters
def main():

    # Check if exactly one argument (besides the program name) is given
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <input_filepath>")
        sys.exit(1)  # Exit the program indicating an error
    # Assign the command-line argument to a variable
    input_filepath = sys.argv[1]
    print(f"The input file path is: {input_filepath}")
    # Load parameters from .json file 
    params = esg.load_parameters(input_filepath)

    # Now load the global variables before calling any functions requiring them
    initialize_globals(params)

    # Compute the integrals
    integral_gamma_1 = integrate_path(gamma_1)
    integral_gamma_2 = integrate_path(gamma_2)

    print("Integral over gamma_1:", integral_gamma_1)
    print("Integral over gamma_2:", integral_gamma_2)

    # Write the data from the paths to .json
    write_path_to_json(params, input_filepath, steps, gamma_1, test_path_key1, steps_path1)
    write_path_to_json(params, input_filepath, steps, gamma_2, test_path_key2, steps_path2)

    # Plot the resulting paths and poles
    plot_paths_and_poles()


if __name__ == '__main__':
    main()

