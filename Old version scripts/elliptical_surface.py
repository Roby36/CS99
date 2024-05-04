
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

# Given parameters
x_0, y_0, = 2, 3
a, b = 4, 3  # Semi-major and semi-minor axes of the ellipses
z_min = 0
alpha = 2

# t & theta parameters
t_min = 0
t_max = 2 * np.pi
theta_max = 4
theta_min = 1
t_steps = 100
theta_steps = 100

# Define root of elliptical cone, using the desired ground level for the ellipse 
z_0 = z_min - (alpha * theta_min)

#!! Define the STEP SIZE HERE!

# Define the parameter ranges (we are defining parameters discretely with a hard-coded step size)
# DFINITION: numpy.linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None, axis=0)
t     = np.linspace(t_min,     t_max,     t_steps)      # t parameter from 0 to 2*pi, in discrete step in ndarray
theta = np.linspace(theta_min, theta_max, theta_steps)  # theta parameter from 0 to theta_max, in discrete steps in ndarray


# Create the meshgrid for the parameters t and theta
T, Theta = np.meshgrid(t, theta)    # generates matrices for parameter space 

# Parametric equations for the surface
X = x_0 + (Theta * a) * np.cos(T)   # Now we make the actual meshed matrices for each x,y,z by component-wise, vecotorized operations 
Y = y_0 + (Theta * b) * np.sin(T)
Z = z_0 + (alpha * Theta)

# Plotting the surface 
# 1) Create figure object
fig = plt.figure(figsize=(10, 8))

# 2) Create axis object
ax = fig.add_subplot(111, projection='3d') # granularily creat a set of INDEPENDENT axes

# 3) Eventually add further independent datasets to axis object 

# Generating a test ring ellipse here, dissecting the surface at a desire z_fixed height
### NOTE: Because the theta array can take no values below theta_min, defining z at any value below z_min yields out-of-range errors
z_fixed     = z_min # pick the z-coordinate of this fixed ellipse
theta_fixed = (z_fixed - z_0)/alpha # determine the corresponding value for the scale factor theta 
step_fixed  = (int) ((theta_steps / (theta_max - theta_min)) * (theta_fixed - theta_min) + 1) # determine corresponding step in the theta progression
### NOTE: '+1' added for indexing issues
X2 = x_0 + (Theta[step_fixed, :] * a) * np.cos(T[0, :])  # Selecting the FIRST ROW of T ONLY, thus a complete cycle, for a single ellipse
Y2 = y_0 + (Theta[step_fixed, :] * b) * np.sin(T[0, :])  # Selecting appropriate Theta row (array of constant desired Theta value)
### NOTE: X2, Y2 are ONE dimensional arrays
Z2 = np.full_like(X2, z_fixed) # Create a 2-d numpy array of the same dimension as X2,Y2, but filled with z_fixed 
ax.plot(X2, Y2, Z2, color='red', linewidth=2)  # Adding the ring-like ellipse

# Successively: Plot the surface
ax.plot_surface(X, Y, Z, rstride=4, cstride=4, alpha=0.7) # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids

# Hardcode axis limits 
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 5)


###DBG:
print(X)
print("\n")
print(Y) 
print("\n")
print(Z) 
print("\n")


# Labeling the axes
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Title and showing the plot
ax.set_title('3D Parametric Surface of Ellipses')
plt.tight_layout()
plt.show()

