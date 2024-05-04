

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

# Define root of elliptical cone, using the desired ground level for the ellipse 
z_0 = z_min - (alpha * theta_min)

# Coomputing linear 1-D arrays for x,y depending on the given step and range values
x_steps = (int)((x_max - x_min)/r)
y_steps = (int)((y_max - y_min)/r)
x = np.linspace(x_min, x_max, x_steps)
y = np.linspace(y_min, y_max, y_steps)

# Create the meshgrid for the parameters x and y
X, Y = np.meshgrid(x, y)    # generates matrices for parameter space 

# Compute the 2-D matrix for the z-values corresponding to each (x,y) pair 
Z = np.maximum(0, z_0 + alpha * np.sqrt(((X - x_0) / a)**2 + ((Y - y_0) / b)**2))

# 1) Create figure object
fig = plt.figure(figsize=(10, 8))

# 2) Create axis object
ax = fig.add_subplot(111, projection='3d') # granularily create a set of INDEPENDENT axes

# 3) Eventually add further independent datasets to axis object 
# Here we add an elliptic ring to highlight a cross-sectional part

# Generating a test ring ellipse here, dissecting the surface at a desire z_fixed height
# Generate also another whole set of theta and t arrays for this purpose 
t_steps = 100
theta_steps = 100
t     = np.linspace(t_min,     t_max,     t_steps)      # t parameter from 0 to 2*pi, in discrete step in ndarray
theta = np.linspace(theta_min, theta_max, theta_steps)  # theta parameter from 0 to theta_max, in discrete steps in ndarray
### NOTE: Because the theta array can take no values below theta_min, defining z at any value below z_min yields out-of-range errors
z_fixed     = z_min # pick the z-coordinate of this fixed ellipse
theta_fixed = (z_fixed - z_0)/alpha # determine the corresponding value for the scale factor theta 
step_fixed  = (int) ((theta_steps / (theta_max - theta_min)) * (theta_fixed - theta_min) + 1) # determine corresponding step in the theta progression
### NOTE: '+1' added for indexing issues
X2 = x_0 + (theta[step_fixed] * a) * np.cos(t)  # Selecting the FIRST ROW of T ONLY, thus a complete cycle, for a single ellipse
Y2 = y_0 + (theta[step_fixed] * b) * np.sin(t)  # Selecting appropriate Theta row (array of constant desired Theta value)
### NOTE: X2, Y2 are ONE dimensional arrays
Z2 = np.full_like(X2, z_fixed) # Create a 2-d numpy array of the same dimension as X2,Y2, but filled with z_fixed 
ax.plot(X2, Y2, Z2, color='red', linewidth=2)  # Adding the ring-like ellipse

# Successively: Plot the surface
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.7) # rstride and cstrinde represent sampling frequency for rows & columns in the meshgrids

# Hardcode axis limits 
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 5)

# Labeling the axes
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')


###DBG:
print(X)
print("\n")
print(Y) 
print("\n")
print(Z) 
print("\n")




# Title and showing the plot
ax.set_title('3D Parametric Surface of Ellipses')
plt.tight_layout()
plt.show()
