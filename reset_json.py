
# This small script deletes select entries from a .json file 

import sys
import json
import homotopic_path_integrals_tests as ht
import path_output_graph as pog
import elliptical_surface_graph as esg
import matrix_comparison_tester as mct

entries_to_delete = [esg.g_image_key, mct.g_image_test_key, ht.test_path_key1, ht.test_path_key2, ht.steps_path1, ht.steps_path2]

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
    with open(input_filepath, 'r') as file:
        params = json.load(file)
    # Delete entries to delete
    for entry in entries_to_delete:
        if entry in params:
            del params[entry]
    # Rewrite the parameters in the json file
    with open(input_filepath, 'w') as file:
        json.dump(params, file, indent=4)

if __name__ == '__main__':
    main()

