
import graphing_utils as gu  # graphing the resulting graph
from params_utils import Params
import subprocess
import shlex
import sys
import os

"""
This is the overarching testing module, using the interface of the executable
compiled from ./src, and the graphing module, to test the algorithm on 
randomized data sets.

For each specific test run of boundary_optimizations, we store the following
main pieces of output in a dedicated directory:
    1) The log file, containing with LaTex-formattable table, of the boundary optimization execution
    2) The resulting output image from the graphing module 
"""

# Global constants
JSON_FILENAME    = "params.json"  # local filename containing .json parameters within directories 
LOGGING_FILENAME = "boundary_optimization_log.txt"

# Wrapper class to run the boundary optimization subroutine programmatically
class Boundary_Optimization:
    """
    bo_bin_path:    where to find the boundary optimization binary file
    float_tol:  in range (0, 1); determines how obstacle-adverse the robot is; the higher, the further from obstacles, but also more likely to find start / goal unaccessible
    bound:      maximum admissible percentage for the fixed path metric (path length), beyond the benchmark value 
    var_mult:   by how much we multiply the variable configuration parameter (generally a, the weight for uncertainty) between each run of A*
    max_it:     maximum number of runs of A* (note that we start by default at a = 0.01, b = 1.0, so this parameter and var_mult should be quite high to span enough values)
    max_expandible_states_mult:     how many times the total states of the grid are we allowed to visit, at maximum, for each A* iteration
    max_hom_classes: how many homotopy classes to keep track of during the A* iterations
    """
    def __init__(self, bo_bin_path, float_tol, bound, var_mult, max_it, max_expandible_states_mult, max_hom_classes):
        self.bo_bin_path = bo_bin_path
        self.float_tol = float_tol
        self.bound = bound
        self.var_mult = var_mult
        self.max_it = max_it
        self.max_expandible_states_mult = max_expandible_states_mult
        self.max_hom_classes = max_hom_classes
        
        # Must be initialized by randomized_test
        self.input_json_path = None
        self.table_title     = None
        self.logging_path    = None
    
    def run(self):
        # Format the command with the instance variables
        command = f"{self.bo_bin_path} {self.input_json_path} \"{self.table_title}\" {self.float_tol} {self.bound} {self.var_mult} {self.max_it} {self.max_expandible_states_mult} {self.max_hom_classes} > {self.logging_path}"
        # Execute the command using subprocess.run
        result = subprocess.run(command, shell=True)
        # Check and print the exit code
        print(f"\nExit code for command '{command}': {result.returncode}")

""" randomized test
bo:                     pre-initialized desired boundary optimization arguments, except for input_json_path and logging_path, which get initialized here
test_output_directory:  desired directory where to store the params.json, logfile, and graph image; created if it doesn't exist yet
test_title:             title for the table in logfile and graph 
num_obstacles:          how many obstacles we want in the environment
obst_size_mean:         mean size of each obstacle under beta distribution, scaled to grid size, must be in range (0, 1) where 0 is zero sized and 1 spans entire grid
obst_size_c:            standard deviation of randomly generated obstacles from the mean; c = 2: uniform distribution, very different obstacle sizes; c = INF: all obstacles with same size determined by the mean
obst_cent_c:            how much obstacle deviate from the center of the grid; c = 2; obstacles are sparse, completely random positions; c = INF: all obstacles centered at the grid's center
alpha_fixed:            fixed value of alpha parameter for the obstacles, in range (0, 1): 0: no uncertainty build up as we move away from the obstacles, 1: extremely rapid uncertainty gradient around obstacles
randomized_start_goal:  whether to randomize the start and goal states, or set them to the extremes
coarsing_factor:        how much coarser the graphed contour of g_image is compared to the actual one fed into A* (required for rendering reasons)
"""
def randomized_test(
    test_output_directory, test_title, num_obstacles,
    obst_size_mean, alpha_fixed, bo,
    obst_cent_c=4, obst_size_c=10, coarsing_factor=8,
    randomized_start_goal=False, plot_title="test_plot.png", figsize=(10,8)
):
    # Check if the directory exists
    if not os.path.exists(test_output_directory):
        # Create the directory if it does not exist
        os.makedirs(test_output_directory)
    full_params_file_path  = os.path.join(test_output_directory, JSON_FILENAME)  # Creates a full path
    full_logging_file_path = os.path.join(test_output_directory, LOGGING_FILENAME)  # Creates a full path

    # Now that we handled the directory, we update the parameters for the boundary_optimization
    bo.input_json_path = full_params_file_path
    bo.logging_path    = full_logging_file_path
    bo.table_title     = test_title

    # Generate random environment parameters
    params_inst = Params()
    params_inst.generate_random_obstacles(num_obstacles, obst_size_mean, alpha_fixed, randomized_start_goal, obst_cent_c, obst_size_c)
    X, Y = Params.compute_meshgrids(1.0)    # full-resolution meshgrid and g_image
    params_inst.compute_g_image(X, Y)
    params_inst.save_to_json(full_params_file_path)  

    # boundary optimization subprocess
    bo.run()
    json_params = Params.load_parameters(full_params_file_path) # retrieve the result from boundary_optimizations

    # now feed into gu the output, along with a coarser version of g
    X, Y = Params.compute_meshgrids(coarsing_factor)
    params_inst.compute_g_image(X, Y)           # NOTE: this effectively changes the internal g_image to a coarser one
    gu.generate_final_graph(json_params, X, Y, params_inst.g_image, test_title, test_output_directory, plot_title, figsize)
""" randomized_test end """


def main():
   
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <bo_bin_path> <test_output_directory> <test_title>")
        sys.exit(1)  # Exit the program indicating an error
    bo_bin_path = sys.argv[1]
    test_output_directory = sys.argv[2]
    test_title = sys.argv[3]

    # run a test, exposing clearly all configurable parameters
    """
    randomized_test(test_output_directory,
                    test_title, 
                    num_obstacles = 1, 
                    obst_size_mean = 0.25, 
                    obst_cent_c = 4, 
                    obst_size_c = 10,
                    alpha_fixed = 0.9, 
                    randomized_start_goal = False,
                    bo=Boundary_Optimization(bo_bin_path, 
                                             float_tol = 0.05, 
                                             bound = 0.3, 
                                             var_mult = 64.0, 
                                             max_it = 4, 
                                             max_expandible_states_mult = 16, 
                                             max_hom_classes = 2
                    )
    )
    """
    alpha_it_tests(bo_bin_path, test_output_directory, 6, 6, 0, 24, 0.05, 4.05)
""" main end """


""" Hard-coded test-runs with some set parameters """

def alpha_it_tests(bo_bin_path, test_output_directory, num_obstacles, max_hom_classes, start_run, end_run, alpha_start, alpha_end):
    """
    The aim of these tests is to experiment with a fixed number of obstacles for different values of alpha,
    whilst maintaining all the other parameters at a relaxed state
    """
    dalpha = (alpha_end - alpha_start) / (end_run - start_run)   # how much alpha increments at each iteration
    curr_alpha = alpha_start    # alpha cannot be zero 
    print(f"\nStarting alpha_it_tests {test_output_directory}, with start_run: {start_run} and end_run: {end_run}, start_alpha: {alpha_start} and end_alpha: {alpha_end}")
    for i in range(start_run, end_run):
        curr_dir = f"{test_output_directory}{i}"
        curr_title = f"alpha_it_tests: alpha = {curr_alpha}"
        print(f"\nRunning alpha_it_tests iteration {i} in directory {curr_dir}")
        randomized_test(curr_dir,
                        curr_title, 
                        num_obstacles=num_obstacles, 
                        obst_size_mean = 0.15, 
                        obst_cent_c = 4, 
                        obst_size_c = 10,
                        alpha_fixed=curr_alpha, 
                        randomized_start_goal = False,
                        bo=Boundary_Optimization(bo_bin_path, 
                                                float_tol = 0.05, 
                                                bound = 0.3, 
                                                var_mult = 64.0, 
                                                max_it = 4, 
                                                max_expandible_states_mult = 16, 
                                                max_hom_classes = max_hom_classes
                    )
        )
        curr_alpha += dalpha 





    



if __name__ == '__main__':
    main()