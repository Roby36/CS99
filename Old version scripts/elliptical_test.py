
### This test aims to establish the equivalence between ellipses surfaces 
### parametrized using the (t, theta) and (x, y) parametrization  

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

# Grid parameters
r = 0.25 
x_min = -10
x_max = 10
y_min = -10
y_max = 10

# Ellipse parameters
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

# Define the parameter ranges (we are defining parameters discretely with a hard-coded step size)
# DFINITION: numpy.linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None, axis=0)
t     = np.linspace(t_min,     t_max,     t_steps)      # t parameter from 0 to 2*pi, in discrete step in ndarray
theta = np.linspace(theta_min, theta_max, theta_steps)  # theta parameter from 0 to theta_max, in discrete steps in ndarray

# Create the meshgrid for the parameters t and theta
T, Theta = np.meshgrid(t, theta)    # generates matrices for parameter space 

# Parametric equations for the surface, in terms of t, theta
X1 = x_0 + (Theta * a) * np.cos(T)   # Now we make the actual meshed matrices for each x,y,z by component-wise, vecotorized operations 
Y1 = y_0 + (Theta * b) * np.sin(T)
Z1 = z_0 + (alpha * Theta)

# Parametric equations for the surface, in terms of x, y
# Coomputing linear 1-D arrays for x,y depending on the given step and range values
x_steps = (int)((x_max - x_min)/r)
y_steps = (int)((y_max - y_min)/r)
x = np.linspace(x_min, x_max, x_steps)
y = np.linspace(y_min, y_max, y_steps)
X, Y = np.meshgrid(x, y)    # generates matrices for parameter space 
Z = np.maximum(0, z_0 + alpha * np.sqrt(((X - x_0) / a)**2 + ((Y - y_0) / b)**2))

# Plotting the surface 
# 1) Create figure object
fig = plt.figure(figsize=(10, 8))

# 2) Create axis object
ax = fig.add_subplot(111, projection='3d') # granularily creat a set of INDEPENDENT axes

# 3) Eventually add further independent datasets to axis object 
# Successively: Plot the surface for both parametrizations
ax.plot_surface(X1, Y1, Z1, rstride=4, cstride=4, alpha=0.7, color='green') # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids
ax.plot_surface(X, Y, Z, rstride=4, cstride=4, alpha=0.7) # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids

# Hardcode axis limits 
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 5)

# Labeling the axes
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Title and showing the plot
ax.set_title('3D Parametric Surface of Ellipses')
plt.tight_layout()
plt.show()