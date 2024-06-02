
import random
import numpy as np
import json

class Params:

    """
    Wrapper class for the parameters that will be written to the intermediate .json
    {
        "r":       set to fixed val
        "s": > r ; set to fixed s/r ratio val
        "x_s":  randomize
        "y_s":  randomize
        "x_g":  randomize
        "y_g":  randomize
        "x_min":    set to fixed val
        "x_max":    set to fixed val
        "y_min":    set to fixed val
        "y_max":    set to fixed val
        
        "g_max":    1.0 (fixed)
        "num_obstacles":    randomize
        "num_obstacle_parameters": 5 (fixed)
        "elliptical_obstacles": [num_obstacles][num_obstacle_parameters] 
    }
    """

    # Constant parameters defining the grid (class variables)
    x_min = -100
    x_max = 100
    y_min = -100
    y_max = 100
    s = 0.2     # makes reasonable 1000 X 1000 grid matrix for the states
    r = (s / 4) # set to some reasonable number of steps for each integral between states
    g_max = 1.0 # cap for g-value
    num_obstacle_parameters = 5  # (x_0, y_0, a, b, alpha)

    # Private class constants for generating ellipses
    _t_min = 0
    _t_max = 2 * np.pi
    _theta_max = 4
    _theta_min = 1    
    
    # instance variables declared here 
    def __init__(self): 
        # Randomized parameters specific to each test 
        self.x_s = None
        self.y_s = None
        self.x_g = None 
        self.y_g = None 
        self.num_obstacles = None 
        self.elliptical_obstacles = None 

        self.g_image = None

    """ generate_random_obstacles
    num_obstacles:      how many obstacles we want in the environment
    obst_size_mean:     mean size of each obstacle, scaled to grid size, in range (0, 1)
    alpha_fixed:        fixed value of alpha parameter for the obstacles, in range (0, 1)
    randomized_start_goal:  whether to randomize the start and goal states, or set them to the extremes
    obst_size_c, alpha_c:   beta distribution parameters; c = 2: uniform --> highest variance, c = INFINITY: centered perfectly around mean
    """
    def generate_random_obstacles(self,  
            num_obstacles,          
            obst_size_mean,         
            alpha_fixed,             
            randomized_start_goal, 
            obst_cent_c = 10,
            obst_size_c = 10,
        ):

        if randomized_start_goal:
            self.x_s = random.randint(self.x_min, self.x_max)
            self.y_s = random.randint(self.y_min, self.y_max)
            self.x_g = random.randint(self.x_min, self.x_max)
            self.y_g = random.randint(self.y_min, self.y_max)
        else:  
        # TODO: Fix A* marking start and goal as unaccessible
            self.x_s = self.x_min + 1
            self.y_s = self.y_min + 1
            self.x_g = self.x_max - 1
            self.y_g = self.y_max - 1
        
        self.num_obstacles = num_obstacles

        """ structure of the elliptical_obstacles matrix
        for e in elliptical_obstacles: 
            e = [x_0, y_0, a, b, alpha]
        """
        self.elliptical_obstacles = []
        for _ in range(num_obstacles):
            """ random centering
            x_0 = random.randint(self.x_min, self.x_max)
            y_0 = random.randint(self.y_min, self.y_max)
            """
            # obstacle center biased toward center of grid
            x0_size = np.random.beta(0.5 * obst_cent_c,  0.5 * obst_cent_c) 
            y0_size = np.random.beta(0.5 * obst_cent_c,  0.5 * obst_cent_c) 
            x_0 = self.x_min + x0_size * (self.x_max - self.x_min)
            y_0 = self.y_min + y0_size * (self.y_max - self.y_min)
            """ 
            Define maximum allowable shape parameters 
            NOTE: These maximum shape parameters do not guarantee the existence of a path to the goal,
            but that, at least, the obstacle cannot exceed each boundary of the grid
            """
            a_max = (self.x_max - self.x_min) / 2
            b_max = (self.y_max - self.y_min) / 2
            """ Pick randomly distributed values between (0, 1), biased towards the mean """
            a_obst_size = np.random.beta(obst_size_mean * obst_size_c,  (1 - obst_size_mean) * obst_size_c) 
            b_obst_size = np.random.beta(obst_size_mean * obst_size_c,  (1 - obst_size_mean) * obst_size_c) 
            a = a_max * a_obst_size 
            b = b_max * b_obst_size
            # alpha_val = np.random.beta(alpha_mean * alpha_c,    (1 - alpha_mean) * alpha_c)  
            """ Store these parameters for the obstacle """
            self.elliptical_obstacles.append([x_0, y_0, a, b, alpha_fixed])
    """ generate_random_obstacles end """

    """ Private helpers for g_image computation """
    @classmethod
    def compute_meshgrids(cls, coarsing_factor):
        """
        Given input ranges for x,y stored in params, this function
        converts them to numpy 2-d arrays for processing, with a coarsing factor arbitrarily reducing resolution
        """
        x_steps = (int)((cls.x_max - cls.x_min) / (coarsing_factor * cls.r))
        y_steps = (int)((cls.y_max - cls.y_min) / (coarsing_factor * cls.r))
        x = np.linspace(cls.x_min, cls.x_max, x_steps)
        y = np.linspace(cls.y_min, cls.y_max, y_steps)
        X, Y = np.meshgrid(x, y)  # generates matrices for parameter space
        return X, Y
    
    @classmethod
    def _generate_elliptical_surface(cls, X, Y, x_0, y_0, a, b, alpha, z_min=0):
        """
        This function generates an ellipse characterized by the given parameters, 
        applying the key equation to convert from the (t, theta) to the (x, y) representation,
        and returns the corresponding numpy 2-d array Z corresponding to the meshgrids X, Y given
        """
        z_0 = z_min - (alpha * cls._theta_min) # root of elliptical cone
        # Compute the 2-D matrix for the z-values corresponding to each (x,y) pair 
        return np.maximum(0, z_0 + alpha * np.sqrt(((X - x_0) / a)**2 + ((Y - y_0) / b)**2))
    
    def compute_g_image(self, X, Y):
        """
        This function applies generate_elliptical_surface using its data for the obstacles
        to compute the comprehensive image of g. 
        """
        Z_list = [] # numpy Z-coord arrays for each ellipse
        for e in self.elliptical_obstacles:
            ZS = self._generate_elliptical_surface(X, Y, e[0], e[1], e[2], e[3], e[4])
            Z_list.append(ZS)
        Z = np.full_like(Z_list[0], self.g_max) # fill-up Z with g_max
        for Z0 in Z_list:   # Iterate through the list, keeping a running minimum
            Z = np.minimum(Z, Z0)
        self.g_image = Z.tolist() # keep as json serializable
    """ compute_g_image end """

    @classmethod
    def generate_elliptical_ring(cls, x_0, y_0, a, b, alpha, z_height=0, z_min=0, t_steps=100, theta_steps=100):
        """
        This function generates a cross sectional ring corresponding to the ellipse parameters,
        plus a parameter giving the desired height of the ring,
        and returns a 3-tuple (X, Y, Z) of the 1d arrays necessary to graph the ring
        """
        z_0 = z_min - (alpha * cls._theta_min)   # First retrieve the z_0 corresponding to the input elliptical surface
        t     = np.linspace(cls._t_min,     cls._t_max,     t_steps)      # t parameter from 0 to 2*pi, in discrete step in ndarray
        theta = np.linspace(cls._theta_min, cls._theta_max, theta_steps)  # theta parameter from 0 to theta_max, in discrete steps in ndarray
        ### NOTE: Because the theta array can take no values below theta_min, defining z at any value below z_min yields out-of-range errors
        theta_fixed = (z_height - z_0)/alpha # determine the corresponding value for the scale factor theta 
        step_fixed  = (int) ((theta_steps / (cls._theta_max - cls._theta_min)) * (theta_fixed - cls._theta_min) + 1) # determine corresponding step in the theta progression
        ### NOTE: '+1' added for indexing issues
        X2 = x_0 + (theta[step_fixed] * a) * np.cos(t)  # Selecting the FIRST ROW of T ONLY, thus a complete cycle, for a single ellipse
        Y2 = y_0 + (theta[step_fixed] * b) * np.sin(t)  # Selecting appropriate Theta row (array of constant desired Theta value)
        ### NOTE: X2, Y2 are ONE dimensional arrays
        Z2 = np.full_like(X2, z_height) # Create a 2-d numpy array of the same dimension as X2,Y2, but filled with z_fixed 
        return (X2, Y2, Z2)
    """ generate_elliptical_ring end """


    # File writing utils
    def save_to_json(self, filename):
        """
        Given a freshly computed numpy array g_image, and filepath,
        this function writes the computed values for g_image into the json file
        """
        with open(filename, 'w') as file:               # 'w' mode to create or truncate existing
            params = self.get_attributes_dictionary()   # store all the PUBLIC attributes in dictionary
            json.dump(params, file, indent=4)           # store dictionary in .json
    
    @classmethod
    def load_parameters(cls, filename):
        """
        Load parameters from a JSON file and return the corresponding dictionary
        """
        with open(filename, 'r') as file:  # Open the file in read mode
            params = json.load(file)  # Load and parse the JSON content into a dictionary
        return params               

    # public getter 
    def get_attributes_dictionary(self):
        """Returns a dictionary of all public (non-underscore prefixed and non-callable) attributes of the class and instance, with instance attributes taking precedence."""
        attributes = {attr: getattr(self, attr) for attr in dir(self)
                        if not attr.startswith('_') and not callable(getattr(self, attr))}
        return attributes

""" class Params end """





""" Unit test, to only create random .json configurations """
def main():
    import graphing_utils as gu 
    test_json = "./params_test.json"
    # Generate random environment parameters

    params_inst = Params()
    Params.s = 1.0     # coarser grid only for testig purposes
    Params.r = 1.0 / 4 
    params_inst.generate_random_obstacles(
        num_obstacles = 6, 
        obst_size_mean = 0.20,
        alpha_fixed = 2.0, 
        randomized_start_goal = False, 
        obst_cent_c = 4, 
        obst_size_c = 10
    )

    X, Y = Params.compute_meshgrids(1.0)    # full-resolution meshgrid and g_image
    params_inst.compute_g_image(X, Y)
    params_inst.save_to_json(test_json)     # unflag if you just want to graph existing .json file

    params_dict = Params.load_parameters(test_json)
    gu.generate_final_graph(params_dict, X, Y, params_dict["g_image"], "plot_title", "./", "test_plot")




if __name__ == '__main__':
    main()


