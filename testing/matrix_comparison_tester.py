
import json
import sys
import g_image as gi

def compare_matrices_with_tolerance(file_path1, key1, file_path2, key2, tolerance=1e-6):

    matrix1 = gi.load_parameters(file_path1)[key1]
    matrix2 = gi.load_parameters(file_path2)[key2]

    # Ensure both matrices have the same dimensions
    if len(matrix1) != len(matrix2) or any(len(row1) != len(row2) for row1, row2 in zip(matrix1, matrix2)):
        print("The matrices have different dimensions and cannot be compared.")
        return

    # Compare each element with a given tolerance
    for i, (row1, row2) in enumerate(zip(matrix1, matrix2)):
        for j, (val1, val2) in enumerate(zip(row1, row2)):
            if not (abs(val1 - val2) <= tolerance):
                print(f"Difference found at row {i}, column {j}: {val1} != {val2}")
                return

    print(f"The matrices {key1} and {key2} are identical within the tolerance of {tolerance}.")

def main():

    # Check if exactly one argument (besides the program name) is given
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <file_path1> <key1> <file_path2> <key2>")
        sys.exit(1)  # Exit the program indicating an error

    file_path1, key1, file_path2, key2 = sys.argv[1:5]
    print(f"Running matrix comparison with inputs file_path1: {file_path1}, key1: {key1}, file_path2: {file_path2}, key2: {key2}")
    compare_matrices_with_tolerance(file_path1, key1, file_path2, key2)

if __name__ == '__main__':
    main()

