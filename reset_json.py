
# This small script deletes select entries from a .json file 

import sys
import json

entries_to_delete = ["g_image", "g_image_test", "A_star_output_path"]

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

