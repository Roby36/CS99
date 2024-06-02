



## Minimum Heap Priority Queue Module

### Overview
The Open Set module implements a priority queue using a min-heap structure, designed to be generic to accommodate different data types. It's primarily intended for use in pathfinding algorithms like A*, where elements (either vertices or homotopy class objects) are prioritized by a heuristic score (`f_score`).

### Data Structures
- **`open_set_t`**: Represents the priority queue (open set). Holds an array of `minheap_node` pointers, the size of the queue, its capacity, and accessor functions for node manipulation.
- **`minheap_node`**: Represents a node in the min-heap. Contains a generic pointer (`void *`) to the data, a float key for priority comparison, and an index tracking the node's position in the heap.

### Public Functions

#### `open_set_new`
**Prototype:**
```c
open_set_t *open_set_new(float (* get_f_score) (void *), void (* set_minheap_node) (void *, minheap_node *));
```

**Description:**
Initializes a new open set (priority queue). The function pointers provided allow the heap to operate generically on any data type that implements these interfaces.

**Parameters:**
- `get_f_score`: Function to retrieve the `f_score` from the data.
- `set_minheap_node`: Function to associate a minheap node with the data.

**Returns:**
- A pointer to an initialized `open_set_t` structure. Returns `NULL` if initialization fails.

**Example Usage:**
```c
open_set_t *os = open_set_new(hom_class_get_f_score, hom_class_set_minheap_node);
```

#### `free_open_set`
**Prototype:**
```c
void free_open_set(open_set_t *set);
```

**Description:**
Frees all memory associated with an `open_set_t` structure, including its nodes. It does not free the data within the nodes, which must be managed externally.

**Parameters:**
- `set`: Pointer to an `open_set_t` structure to be freed.

**Returns:**
- None.

**Example Usage:**
```c
free_open_set(os);
```

#### `insert_sorted`
**Prototype:**
```c
void insert_sorted(open_set_t *set, void * data);
```

**Description:**
Inserts an element into the heap while maintaining the heap property. The element is added based on its `f_score` determined by the `get_f_score` function.

**Parameters:**
- `set`: Pointer to the `open_set_t`.
- `data`: Generic pointer to the data to be inserted.

**Returns:**
- None.

#### `dequeue_min`
**Prototype:**
```c
void * dequeue_min(open_set_t *set);
```

**Description:**
Removes and returns the element with the minimum `f_score` from the heap. This operation also maintains the heap property post-deletion.

**Parameters:**
- `set`: Pointer to the `open_set_t`.

**Returns:**
- Generic pointer to the data with the minimum `f_score`. Returns `NULL` if the heap is empty.

#### `decrease_key`
**Prototype:**
```c
void decrease_key(open_set_t *set, minheap_node *node, float new_key);
```

**Description:**
Decreases the `f_score` of a specific element in the heap and reorders the heap to maintain the heap property. This function is critical for pathfinding algorithms where the cost to reach a node may decrease as more paths are evaluated.

**Parameters:**
- `set`: Pointer to the `open_set_t`.
- `node`: Pointer to the `minheap_node` whose key is to be decreased.
- `new_key`: The new key value.

**Returns:**
- None.

#### `remove_from_open_set`
**Prototype:**
```c
void remove_from_open_set(open_set_t *set, minheap_node *node);
```

**Description:**
Removes a specific element from the heap. This function handles the removal internally and maintains the heap property afterward.

**Parameters:**
- `set`: Pointer to the `open_set_t`.
- `node`: Pointer to the `minheap_node` to be removed.

**Returns:**
- None.

### Simple Getters

#### `open_set_is_empty`
**Prototype:**
```c
bool open_set_is_empty(open_set_t *set);
```

**Description:**
Checks if the open set is empty.

**Parameters:**
- `set`: Pointer to the `open_set_t`.

**Returns:**
- `true` if the open set is empty, `false` otherwise.

#### `open_set_size`
**Prototype:**


```c
int open_set_size(open_set_t * set);
```

**Description:**
Returns the number of elements in the open set.

**Parameters:**
- `set`: Pointer to the `open_set_t`.

**Returns:**
- The number of elements in the open set.

### Notes on Implementation
This module leverages function pointers to achieve type generality, allowing it to be used with any data type that implements the required interface. The use of dynamic memory allocation and careful management of indices and keys allows the heap to operate efficiently even as elements are added, removed, or updated.


## Obstacle Marker Module

### Overview
The Obstacle Marker module leverages complex analysis to calculate properties and interactions regarding obstacles in a given environment. It manages the calculation of path integrals and L-values which are essential for determining homotopy classes in path planning algorithms.

### Functions

#### `initialize_obstacle_marker_func_params`
**Prototype:**
```c
F_t * initialize_obstacle_marker_func_params(Params * params, float float_tol);
```

**Description:**
Initializes and returns a pointer to an `F_t` structure containing all necessary parameters and precomputed values for obstacle marker calculations. This function integrates several private helper functions to extract parameters, compute residues, and calculate L-values.

**Parameters:**
- `params`: Pointer to a `Params` structure containing configuration parameters extracted from a JSON file.
- `float_tol`: A floating-point tolerance used for numerical comparisons.

**Returns:**
- Pointer to the initialized `F_t` structure.

**Example Usage:**
```c
Params *params = load_json("config.json");
float tol = 0.001;
F_t *F = initialize_obstacle_marker_func_params(params, tol);
```

#### `delete_obstacle_marker_func_params`
**Prototype:**
```c
void delete_obstacle_marker_func_params(F_t * F);
```

**Description:**
Frees all memory associated with an `F_t` structure initialized by `initialize_obstacle_marker_func_params`. This includes all internal matrices and dynamic arrays.

**Parameters:**
- `F`: Pointer to an `F_t` structure to be freed.

**Returns:**
- None.

**Example Usage:**
```c
delete_obstacle_marker_func_params(F);
```

#### `get_Lval`
**Prototype:**
```c
Complex get_Lval(F_t * F, int x_c, int y_c, int x_n, int y_n);
```

**Description:**
Retrieves the complex L-value corresponding to the path integral contribution between two neighboring grid points, specified by their coordinates. This function ensures the points are accessible and within valid ranges before fetching the precomputed value.

**Parameters:**
- `F`: Pointer to an initialized `F_t` structure.
- `x_c`: X-coordinate of the current grid point.
- `y_c`: Y-coordinate of the current grid point.
- `x_n`: X-coordinate of the neighboring grid point.
- `y_n`: Y-coordinate of the neighboring grid point.

**Returns:**
- Complex L-value of the obstacle marker function between the specified points.

**Example Usage:**
```c
Complex L_value = get_Lval(F, 0, 0, 1, 1);
```

#### `calculate_path_integral`
**Prototype:**
```c
Complex calculate_path_integral(float ** test_path, int num_points, F_t * F);
```

**Description:**
Calculates the complex path integral along a specified path using the obstacle marker function parameters. This function sums up the contributions between consecutive points along the path.

**Parameters:**
- `test_path`: 2D array of float coordinates representing the path.
- `num_points`: Number of points in the path.
- `F`: Pointer to an initialized `F_t` structure.

**Returns:**
- Complex-valued result of the path integral.

**Example Usage:**
```c
float **path = {{0.0, 0.0}, {1.0, 1.0}};
int points = 2;
Complex integral = calculate_path_integral(path, points, F);
```

### Remarks
The module's implementation includes a significant amount of pre-computation during initialization, which may result in longer startup times but will optimize runtime performance during actual operation.


# Parser Module 

## Overview
The Parser module is designed to handle the reading and writing of JSON configuration files using the cJSON library. This module provides functionality to parse JSON files into a `Params` structure, write matrices to JSON, and manage memory for dynamic structures.

## Functions

### `write_json`
**Prototype:**
```c
void write_json(const char* file_path, const char * key, float ** matrix, int rows, int cols);
```

**Description:**
Writes a 2-dimensional float matrix under a specified key into a JSON file at the provided file path. The matrix dimensions are defined by the rows and columns parameters.

**Parameters:**
- `file_path`: Path to the JSON file where data will be written.
- `key`: Key under which the matrix will be stored in the JSON file.
- `matrix`: Pointer to the 2-dimensional float matrix to write.
- `rows`: Number of rows in the matrix.
- `cols`: Number of columns in the matrix.

**Returns:**
- None.

**Example Usage:**
```c
float** matrix_data = {{1.0, 2.0}, {3.0, 4.0}};
write_json("config.json", "matrix_key", matrix_data, 2, 2);
```

---

### `load_json`
**Prototype:**
```c
Params * load_json(const char * file_name);
```

**Description:**
Loads parameters from a JSON file into a newly allocated `Params` structure. This function parses the JSON file and fills the structure with extracted float and integer values, and matrices.

**Parameters:**
- `file_name`: Path to the JSON file to be parsed.

**Returns:**
- Pointer to a newly allocated `Params` structure filled with configuration data. Returns `NULL` on parsing failure.

**Example Usage:**
```c
Params *simulation_params = load_json("config.json");
```

---

### `free_params`
**Prototype:**
```c
void free_params(Params * params);
```

**Description:**
Frees all memory allocated for a `Params` structure, including any dynamically allocated matrices within.

**Parameters:**
- `params`: Pointer to the `Params` structure to be freed.

**Returns:**
- None.

**Example Usage:**
```c
Params *params = load_json("config.json");
// Use params for processing
free_params(params);
```

