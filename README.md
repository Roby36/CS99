


## Cost and Heuristic Functions Module

### Overview
This module includes functions for calculating costs and heuristics necessary for pathfinding algorithms like A*. It offers precise methods for computing the distance and uncertainty accumulation along paths, as well as various heuristic estimates.

### Functions

#### `len`
**Prototype:**
```c
float len(int x_1, int y_1, int x_2, int y_2, Params *params);
```
- **Description**: Calculates the absolute distance between two adjacent points on a grid, scaled to map units.
- **Formula**: 
  \[
  \text{len} = s \cdot D_{\text{adjacent}}(x_1, y_1, x_2, y_2)
  \]
  Here, \( D_{\text{adjacent}} \) represents the Euclidean distance between the points, and \( s \) is the scaling factor from the `Params` struct.
- **Parameters**:
  - `x_1, y_1, x_2, y_2`: Coordinates of two adjacent points on the grid.
  - `params`: Contains the scaling factor `s` for the grid size.
- **Returns**: The distance between the points in map units.

#### `g_image_riemann_sum`
**Prototype:**
```c
float g_image_riemann_sum(int x_1, int y_1, int x_2, int y_2, Params *params);
```
- **Description**: Computes the Riemann sum of an uncertainty function `g` between two adjacent points.
- **Formula**:
  \[
  U = r \cdot D_{\text{adjacent}}(x_1, y_1, x_2, y_2) \cdot \sum_{i=0}^{s/r} g_{\text{image}}[y_0 + i \cdot dy][x_0 + i \cdot dx]
  \]
  where \( r \) and \( s \) are scaling factors, \( D_{\text{adjacent}} \) is the distance function, and \( g_{\text{image}} \) contains precomputed values of the function `g`.
- **Parameters**:
  - `x_1, y_1, x_2, y_2`: Coordinates of two adjacent points.
  - `params`: Contains scaling factors and the `g_image` matrix.
- **Returns**: The Riemann sum contribution of taking a path integral of `g` between the points, scaled to map units.

#### `edge_cost`
**Prototype:**
```c
float edge_cost(int x_1, int y_1, int x_2, int y_2, Params *params, Config *config);
```
- **Description**: Combines the uncertainty accumulation and path length to compute the overall cost of moving between two points.
- **Formula**:
  \[
  C = a \cdot U + b \cdot L
  \]
  where \( U \) is the uncertainty accumulation calculated by `g_image_riemann_sum`, \( L \) is the length from `len`, and \( a \) and \( b \) are coefficients from the `Config` struct.
- **Parameters**:
  - `x_1, y_1, x_2, y_2`: Coordinates of two adjacent points.
  - `params`: Environment parameters used in sub-calculations.
  - `config`: Contains the weights `a` and `b`.
- **Returns**: The total cost of moving between the two points.

#### `heuristic`
**Prototype:**
```c
float heuristic(int x_n, int y_n, int x_g, int y_g, Params *params, Config *config);
```
- **Description**: Calculates a heuristic estimate of the cost from a current point to the goal, favoring diagonal movement.
- **Formula**:
  \[
  H = b \cdot s \cdot ((\sqrt{2} - 1) \cdot \min(dx, dy) + \max(dx, dy))
  \]
  where \( dx \) and \( dy \) are the absolute differences in grid steps between the current point and the goal, \( s \) is the grid scaling factor, and \( b \) is the coefficient for path length in the cost function.
- **Parameters**:
  - `x_n, y_n`: Current point on the grid.
  - `x_g, y_g`: Goal point.
  - `params`: Contains scaling factors for the grid.
  - `config`: Contains the weight `b` for the path length in the cost function.
- **Returns**: The heuristic cost estimate from the

 current point to the goal.

#### `zero_heuristic`
**Prototype:**
```c
float zero_heuristic(int x_n, int y_n, int x_g, int y_g, Params *params, Config *config);
```
- **Description**: Provides a zero-value heuristic, effectively transforming A* into Dijkstra's algorithm. This is used when an unbiased exploration of all paths is required.
- **Parameters**:
  - `x_n, y_n, x_g, y_g`: Coordinates for the current and goal points (used to match the function signature required by A*).
  - `params` and `config`: Included to conform to the required function signature.
- **Returns**: Always returns `0.0`.

This documentation details how each function contributes to the pathfinding process, using precise mathematical formulas to clarify the computation of distances, costs, and heuristic estimates.




## Homotopy Classes Module

### Overview
The Homotopy Classes module provides data structures and functions for managing homotopy classes associated with vertices in a computational graph. These classes are critical for pathfinding in environments with obstacles, as they allow differentiating paths based on the topological constraints imposed by obstacles.

### Data Structures

#### `hom_vertex_t`
Represents a vertex in the graph with potential homotopy classes extending from it.
- **Fields**:
  - `int x_s, y_s`: Discrete coordinates of the vertex.
  - `hom_classes_list_t *hom_classes`: Linked list of homotopy classes associated with this vertex.

#### `hom_class_t`
Stores data defining a homotopy class.
- **Fields**:
  - `Complex Lval`: Descriptor for the homotopy class based on the integral of the obstacle marker function.
  - `hom_class_t *next`: Pointer to the next homotopy class in a linked list.
  - `struct hom_vertex *endpoint_vertex`: Pointer to the vertex where this homotopy class ends.
  - `struct minheap_node *minheap_node`: Pointer to a node in a min-heap, used in priority queues.
  - `Backtrack_Path *backtrack`: Path associated with this homotopy class, used for backtracking in pathfinding.
  - `float g_score, f_score`: Cost metrics used in A* and similar algorithms.
  - `bool is_evaluated, updated`: Flags to manage the state within algorithms.

#### `Backtrack_Path`
Holds a path and its associated costs.
- **Fields**:
  - `int total_edges`: Number of edges in the path.
  - `float **point_list`: List of points (coordinates) making up the path.
  - `float absolute_length, g_image_riemann_tot`: Metrics evaluating the path's length and integral cost.

### Functions

#### `hom_vertex_new`
**Prototype:**
```c
hom_vertex_t *hom_vertex_new(int x_s, int y_s, int max_hom_classes, bool (*less_than)(hom_class_t *, hom_class_t *));
```
- **Description**: Constructs a new homotopy vertex with an empty list of homotopy classes.
- **Parameters**:
  - `x_s, y_s`: Coordinates of the vertex.
  - `max_hom_classes`: Maximum number of homotopy classes the vertex can hold.
  - `less_than`: Comparison function used for ordering homotopy classes.
- **Returns**: Pointer to the newly created `hom_vertex_t`.

#### `hom_class_new`
**Prototype:**
```c
hom_class_t *hom_class_new(hom_vertex_t *endpoint_vertex, Complex Lval);
```
- **Description**: Creates a new homotopy class with a specified endpoint vertex and descriptor.
- **Parameters**:
  - `endpoint_vertex`: Vertex at the end of the homotopy class.
  - `Lval`: Complex descriptor of the homotopy class.
- **Returns**: Pointer to the newly created `hom_class_t`.

#### `hom_class_free`
**Prototype:**
```c
void hom_class_free(hom_class_t *hc);
```
- **Description**: Frees the memory allocated for a homotopy class, including its associated backtrack path.
- **Parameters**:
  - `hc`: Homotopy class to free.

#### `expand_backtrack`
**Prototype:**
```c
void expand_backtrack(Params *params, hom_class_t *hom_class);
```
- **Description**: Expands the backtrack path for a given homotopy class using stored homotopy class information.
- **Parameters**:
  - `params`: Parameters of the environment.
  - `hom_class`: Homotopy class to expand.

#### `write_path`
**Prototype:**
```c
void write_path(hom_class_t *hom_class, Config *config, char *filepath);
```
- **Description**: Writes the points of a homotopy class's path to a JSON file, formatted for visualization.
- **Parameters**:
  - `hom_class`: Homotopy class containing the path.
  - `config`: Configuration struct containing cost function coefficients.
  - `filepath`: Path to the output JSON file.

#### `free_backtrack`
**Prototype:**
```c
void free_backtrack(hom_class_t *hom_class);
```
- **Description**: Frees the memory allocated for the backtrack path of a homotopy class.
- **Parameters**:
  - `hom_class`: Homotopy class whose backtrack path is to be freed.

### Usage
This module is used extensively in pathfinding algorithms where paths must be differentiated based

 on their traversal of complex environments. Homotopy classes provide a method to keep track of paths that are topologically distinct due to obstacles, which is crucial for applications like robotic navigation and simulation.

This documentation should be integrated with specific details on how each function interacts with other modules, especially in terms of data handling and memory management, to ensure that the system operates efficiently and correctly.




## Homotopy Classes List Module

### Overview
The `hom_classes_list` module implements a simple linked list to manage homotopy classes efficiently. Each homotopy class is linked via a `next` pointer, forming a chain that represents possible paths through a computational graph. The list manages insertion based on a comparison function, ensuring that only the best (according to a given metric, usually `f_score`) homotopy classes are retained up to a predefined capacity.

### Data Structure

#### `hom_classes_list`
- **Description**: Manages a linked list of homotopy classes.
- **Fields**:
  - `hom_class_t *head`: Pointer to the first homotopy class in the list.
  - `int capacity`: The maximum number of homotopy classes the list can hold.
  - `int size`: Current number of homotopy classes in the list.
  - `bool (*less_than)(hom_class_t *, hom_class_t *)`: Pointer to a function that defines the sorting criterion of homotopy classes.

### Functions

#### `hom_classes_list_new`
**Prototype**:
```c
hom_classes_list_t *hom_classes_list_new(int capacity, bool (*less_than)(hom_class_t *, hom_class_t *));
```
- **Description**: Initializes a new list for managing homotopy classes with a specified capacity and comparison function.
- **Parameters**:
  - `capacity`: The maximum number of homotopy classes the list can hold.
  - `less_than`: Function to compare two homotopy classes; determines the sorting and retention of classes in the list.
- **Returns**: A pointer to the newly created `hom_classes_list` or `NULL` if memory allocation fails.

#### `free_hom_classes_list`
**Prototype**:
```c
void free_hom_classes_list(hom_classes_list_t *hom_classes_list);
```
- **Description**: Frees all memory associated with a `hom_classes_list`, including all homotopy classes it contains.
- **Parameters**:
  - `hom_classes_list`: Pointer to the list to be freed.
- **Note**: This function is responsible for freeing all `hom_class_t` objects in the list. The caller must ensure that no double frees occur by managing the homotopy classes not in the list.

#### `hom_class_get`
**Prototype**:
```c
hom_class_t *hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);
```
- **Description**: Searches for a homotopy class in the list that matches a given complex L-value within a specified tolerance.
- **Parameters**:
  - `hom_classes_list`: The list to search.
  - `Lval`: The L-value descriptor of the homotopy class.
  - `abs_tol`: The absolute tolerance for comparing L-values.
- **Returns**: Pointer to the homotopy class if found; `NULL` otherwise.

#### `insert_hom_class_sorted`
**Prototype**:
```c
bool insert_hom_class_sorted(hom_classes_list_t *hom_classes_list, hom_class_t *hom_class, double abs_tol, hom_class_t **truncated);
```
- **Description**: Inserts a new homotopy class into the list in a sorted order based on the `less_than` function, maintaining the list's capacity.
- **Parameters**:
  - `hom_classes_list`: The list into which the homotopy class is to be inserted.
  - `hom_class`: The homotopy class to insert.
  - `abs_tol`: Tolerance used to check uniqueness of the new homotopy class in the list.
  - `truncated`: Outputs the homotopy class that was removed from the list to maintain the capacity, if any.
- **Returns**: `true` if the class was inserted; `false` otherwise. If `false`, `truncated` will be `NULL`.

#### `complex_list_to_string`
**Prototype**:
```c
char* complex_list_to_string(hom_classes_list_t *list);
```
- **Description**: Converts the list of homotopy classes to a string representation.
- **Parameters**:
  - `list`: The homotopy class list to convert.
- **Returns**: A dynamically allocated string representing the list. The caller is responsible for freeing this string.

#### `unflag_homotopy_classes`
**Prototype**:
```c
void unflag_homotopy_classes(hom_classes_list_t *list);
```
- **Description**: Resets the `updated` flag on all homotopy classes in the list, typically used to mark classes that have not been processed in the current iteration of an algorithm.
- **Parameters**:
  - `list`: The list of homot

opy classes to update.

### Usage Example
This module is typically used in pathfinding algorithms where paths are differentiated based on their traversal of an environment with obstacles. The list manages these paths, ensuring efficient operations by maintaining only the most promising paths according to a predefined metric.

```c
hom_classes_list_t *list = hom_classes_list_new(10, g_score_compare);
hom_class_t *new_class = hom_class_new(some_vertex, some_lval);
hom_class_t *truncated_class;
if (insert_hom_class_sorted(list, new_class, 0.01, &truncated_class)) {
    if (truncated_class) {
        hom_class_free(truncated_class);
    }
}
free_hom_classes_list(list);
```

This documentation provides a clear overview of how to integrate and use the `hom_classes_list` module in larger computational frameworks, especially those dealing with complex pathfinding and graph traversal scenarios.




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
This module encapsulates complex analysis techniques to effectively manage homotopy classes in environments with obstacles. It utilizes a polynomial obstacle marker function, residues, and path integral calculations to uniquely identify paths based on their interaction with obstacles.

### Mathematical Formulation

#### Obstacle Marker Polynomial
The obstacle marker function, \( f_0(z) \), is chosen based on the polynomial:
\[ f_0(z) = (z - BL)^a \cdot (z - TR)^b \]
where:
- \( z \) is the complex representation of a point in the plane.
- \( BL \) and \( TR \) are the bottom-left and top-right corners of the operational area, respectively, treated as complex numbers.
- \( a \) and \( b \) are integers that partition \( N-1 \), where \( N \) is the total number of obstacles.

This choice of polynomial is motivated by its ability to simplify the calculation of residues and integrals, with \( \zeta_i \) (centers of elliptical obstacles) serving as critical points that influence path differentiation.

#### Residue Computation
The residue for each obstacle marker \( \zeta_l \) is calculated using the formula:
\[ A_l = \frac{f_0(\zeta_l)}{\prod_{\substack{j=0 \\ j \neq l}}^N (\zeta_l - \zeta_j)} \]
where \( \zeta_j \) are the complex representations of all obstacle markers. The product in the denominator computes the influence of all other obstacles on the specific marker \( \zeta_l \), excluding itself.

#### Analytical Formula for L-Value Calculation
The L-value between any two points \( z_1 \) and \( z_2 \) on the complex plane is computed as:
\[ L(z_1, z_2) = \sum_{l=0}^N A_l \cdot \left( \log|z_2 - \zeta_l| - \log|z_1 - \zeta_l| + i (\arg(z_2 - \zeta_l) - \arg(z_1 - \zeta_l) + 2\pi k_l) \right) \]
where:
- \( A_l \) are the residues associated with each obstacle.
- \( \arg \) and \( \log \) represent the argument and logarithm functions, respectively.
- \( k_l \) is chosen to minimize the absolute value of the argument difference, ensuring it lies within \([- \pi, \pi]\).

### Implementation Details

The module defines a structure, `F`, which contains the parameters necessary for the computations:
- `N`, `a`, `b`: Parameters defining the polynomial order and partitioning.
- `BL`, `TR`: Complex numbers representing the operational area corners.
- `obstacle_markers`: Dynamically allocated array of complex numbers representing the centers of elliptical obstacles.
- `A`: Array storing computed residues.
- `Lvals`: Four-dimensional array for storing precomputed L-values for grid points.

This setup allows for efficient computation of path integrals and obstacles interaction by precomputing and reusing critical values like residues and L-values across different parts of the algorithm.

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

