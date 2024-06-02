




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

