
import json

def compare_matrices_with_tolerance(file_path, key1, key2, tolerance=1e-6):
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    matrix1 = data[key1]
    matrix2 = data[key2]

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

# Example usage
compare_matrices_with_tolerance('./environment copy.json', 'g_image', 'g_image_test')

