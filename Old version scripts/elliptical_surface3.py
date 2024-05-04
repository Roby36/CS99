
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

# Main Grid parameters 
r = 0.25 
x_min = -20
x_max = 20
y_min = -20
y_max = 20

# Elliptical obstacle parameters (will eventually come from .json config file)
# NOTE: These are stored in a 2-d array, indexed by the number of the object, which itself
# is an array with entries (x_0, y_0, a, b, z_min, alpha)

elliptical_obstacles = [
    [2, 3, 4, 3, 0, 0.8],
    [6, 7, 2, 3, 0, 0.5],
    [-5, 10, 2, 6, 0, 0.3],
    [5, -6, 7, 3, 0, 0.4],
]

# Uncertainty cap:
g_max = 1.0

# Axes limits parameters 
x_lim = [x_min, x_max]
y_lim = [y_min, y_max]
z_lim = [0, 5]

# t & theta parameters
t_min = 0
t_max = 2 * np.pi
theta_max = 4
theta_min = 1

# Coomputing linear 1-D arrays for x,y depending on the given step and range values
x_steps = (int)((x_max - x_min)/r)
y_steps = (int)((y_max - y_min)/r)
x = np.linspace(x_min, x_max, x_steps)
y = np.linspace(y_min, y_max, y_steps)

# Create the meshgrid for the parameters x and y
X, Y = np.meshgrid(x, y)    # generates matrices for parameter space 

# 1) Create figure object
fig = plt.figure(figsize=(10, 8))

# 2) Create axis object
ax = fig.add_subplot(111, projection='3d') # granularily create a set of INDEPENDENT axes

# This function generates an ellipse characterized by the given parameters, returning the 
# resulting Z 2d array correcponding to the global X, Y grids 
def generate_ellipse(x_0, y_0, a, b, z_min, alpha):
    # Define root of elliptical cone, using the desired ground level for the ellipse 
    z_0 = z_min - (alpha * theta_min)
    # Compute the 2-D matrix for the z-values corresponding to each (x,y) pair 
    return np.maximum(0, z_0 + alpha * np.sqrt(((X - x_0) / a)**2 + ((Y - y_0) / b)**2))

# This function generates a cross sectional ring corresponding to the ellipse parameters,
# plus a parameter giving the desired height of the ring,
# and returns a 3-tuple (X, Y, Z) of the 1d arrays necessary to graph the ring
def generate_ring(x_0, y_0, a, b, z_min, alpha, z_height, t_steps=100, theta_steps=100):
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

# Based on the input obstacles, grid, and uncertainty cap, compute the full image of the uncertainty function g
def compute_g_image_n_plot_base_rings(z_height=0, ring_color='red'):
    # Iterate through the input ellipses
    Z_list = [] # list holding the 2-dimensional numpy arrays corresponding to z-coordinates for each obstacle
    for e in elliptical_obstacles:
        ZS = generate_ellipse(e[0], e[1], e[2], e[3], e[4], e[5])
        Z_list.append(ZS) # add the newly generated ellipse coordinates to the running list
        X1, Y1, Z1 = generate_ring(e[0], e[1], e[2], e[3], e[4], e[5], z_height)
        ax.plot(X1, Y1, Z1, color=ring_color, linewidth=2)  # Adding the ring-like ellipse
    # Return the resulting values for the uncertainty function in Z
    Z = np.full_like(Z_list[0], g_max) # fill-up Z with g_max
    # Iterate through the list, and keep a running minimum
    for Z0 in Z_list:
        Z = np.minimum(Z, Z0)
    return Z

# 3) Eventually add further independent datasets to global axis object 
g_image = compute_g_image_n_plot_base_rings() # Get the resulting image of g, whilst graphing the rings
ax.plot_surface(X, Y, g_image, rstride=1, cstride=1, alpha=1.0) # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids

# Set axis limits 
ax.set_xlim(x_lim[0], x_lim[1])
ax.set_ylim(y_lim[0], y_lim[1])
ax.set_zlim(z_lim[0], z_lim[1])

# Label the axes
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Title and showing the plot
ax.set_title('3D Parametric Surface of Ellipses')
plt.tight_layout()
plt.show()
