
#include "parser.h"

/* Static Helpers */
static char * store_file(const char * file_name);
static void print_matrix_to_file(FILE *fp, const char* key, float** matrix, int rows, int cols);
static void free_float_matrix(float ** M, int rows, int columns);
static float ** extract_float_matrix(cJSON *json, char *key);
static float extract_float(cJSON *json, char *key);
static int extract_int(cJSON *json, char *key);


/************** extract_float_matrix ****************
 * 
 * Uses cJSON library to extract a 2-dimensional float matrix, 
 * taking as input cJSON struct and key under which the matrix is stored
 * 
 * NOTE: function returns malloc'd matrix, which the caller 
 *       is then responsible to free by invoking free_float_matrix
 */
float ** extract_float_matrix(cJSON *json, char *key) {
    // Find the JSON array using the provided key
    cJSON *matrix_json = cJSON_GetObjectItemCaseSensitive(json, key);
    
    // Check if the matrix key exists and is an array
    if (!cJSON_IsArray(matrix_json)) {
        fprintf(stderr, "Error: JSON key '%s' is not an array or does not exist\n", key);
        return NULL;
    }
    
    // Determine the number of rows in the matrix
    int num_rows = cJSON_GetArraySize(matrix_json);
    
    // Allocate memory for the matrix
    float **matrix = malloc(num_rows * sizeof(float *));
    if (!matrix) {
        fprintf(stderr, "Error: Failed to allocate memory for matrix rows\n");
        return NULL;
    }
    
    // Iterate over each row of the matrix
    for (int i = 0; i < num_rows; i++) {
        // Get the JSON array that represents the current row
        cJSON *row_json = cJSON_GetArrayItem(matrix_json, i);
        if (!cJSON_IsArray(row_json)) {
            fprintf(stderr, "Error: Row %d is not an array\n", i);
            // Free any previously allocated rows
            for (int j = 0; j < i; j++) {
                free(matrix[j]);
            }
            free(matrix);
            return NULL;
        }

        int num_cols = cJSON_GetArraySize(row_json);
        
        // Allocate memory for this row
        matrix[i] = malloc(num_cols * sizeof(float));
        if (!matrix[i]) {
            fprintf(stderr, "Error: Failed to allocate memory for matrix row %d\n", i);
            // Free any previously allocated rows and the matrix itself
            for (int j = 0; j < i; j++) {
                free(matrix[j]);
            }
            free(matrix);
            return NULL;
        }
        
        // Iterate over each column in the row
        for (int j = 0; j < num_cols; j++) {
            // Get the value at the current position
            cJSON *value_json = cJSON_GetArrayItem(row_json, j);
            if (!cJSON_IsNumber(value_json)) {
                fprintf(stderr, "Error: Non-numeric value in matrix at row %d, col %d\n", i, j);
                // Free any previously allocated rows and the matrix itself
                for (int k = 0; k <= i; k++) {
                    free(matrix[k]);
                }
                free(matrix);
                return NULL;
            }
            
            // Store the value in the matrix
            matrix[i][j] = (float)cJSON_GetNumberValue(value_json);
        }
    }
    
    // Return the constructed matrix
    return matrix;
}

/************** extract_float ****************
 * 
 * Uses cJSON library to extract a float value, 
 * taking as input cJSON struct and key under which the float is stored
 */
float extract_float(cJSON *json, char *key) {
    // Check if the key exists and is the correct type
    cJSON *item = cJSON_GetObjectItemCaseSensitive(json, key);
    if (item == NULL) {
        fprintf(stderr, "Error: Key '%s' does not exist in the JSON data.\n", key);
        return 0.0f; // Return a default value or error code
    }

    if (!cJSON_IsNumber(item)) {
        fprintf(stderr, "Error: Key '%s' is not a number.\n", key);
        return 0.0f; // Return a default value or error code
    }

    // Return the number value
    return (float)item->valuedouble;
}

/************** extract_int ****************
 * 
 * Uses cJSON library to extract an int value, 
 * taking as input cJSON struct and key under which the int is stored
 */
int extract_int(cJSON *json, char *key) {
    // Attempt to retrieve the item associated with 'key'
    cJSON *item = cJSON_GetObjectItemCaseSensitive(json, key);
    if (item == NULL) {
        fprintf(stderr, "Error: Key '%s' does not exist in the JSON data.\n", key);
        return 0; // Return a default value or error code
    }

    // Ensure the item is a number
    if (!cJSON_IsNumber(item)) {
        fprintf(stderr, "Error: Key '%s' is not a number.\n", key);
        return 0; // Return a default value or error code
    }

    // Return the integer value
    return item->valueint;
}

/***************** load_json ******************
 * 
 * This is one of the two main functions of the module.
 * 
 * It stores the contents of the .json file to a string
 * through the store_file subroutine, converts the string into a 
 * cJSON struct, and finally wraps the static functions defined above, 
 * extracting integers, floats, and float matrices,
 * to fill up a struct Params containing all the relevant parameters
 * stored in the original .json configuration file.  
 * 
 * NOTE: If more parameters are added to the .json configuration file in the future,
 *       the only sections of the code requiring an update are this function's definition,
 *       and the struct Params definition in utils.h
 * 
 *       The caller is responsible for later free'ing the Params struct returned
 *       by calling free_params
 */
Params * load_json(const char * file_name) {

    char * copy_str = store_file(file_name);

    cJSON *json = cJSON_Parse(copy_str);
    if (!json) {
        fprintf(stderr, "Error parsing JSON\n");
        return NULL;
    }

    // Allocate space 
    Params * params = malloc(sizeof(Params));
    
    // Extract floats
    params->r     = extract_float(json, "r");
    params->s     = extract_float(json, "s");
    params->x_max = extract_float(json, "x_max");
    params->x_min = extract_float(json, "x_min");
    params->y_min = extract_float(json, "y_min");
    params->y_max = extract_float(json, "y_max");
    params->x_g   = extract_float(json, "x_g");
    params->y_g   = extract_float(json, "y_g");
    params->x_s   = extract_float(json, "x_s");
    params->y_s   = extract_float(json, "y_s");
    params->g_max = extract_float(json, "g_max");

    // Hard-code the division so it doesn't need to be carried out repeatedly
    params->s_div_r = params->s / params->r;

    // Calculate the total number of s_steps based on the input parameters
    params->xs_steps = calculate_steps(params->x_min, params->x_max, params->s);
    params->ys_steps = calculate_steps(params->y_min, params->y_max, params->s);
    // Calculate the total number of r_steps based on the input parameters
    params->xr_steps = calculate_steps(params->x_min, params->x_max, params->r);
    params->yr_steps = calculate_steps(params->y_min, params->y_max, params->r);

    // Extract integers
    params->num_obstacles           = extract_int(json, "num_obstacles");
    params->num_obstacle_parameters = extract_int(json, "num_obstacle_parameters");

    // Extract matrices
    params->g_image              = extract_float_matrix(json,  "g_image");
    params->elliptical_obstacles = extract_float_matrix(json,  "elliptical_obstacles");
    
    // Clean-up
    cJSON_Delete(json);
    free(copy_str); 

    return params;
}

/************** free_params ********************
 * 
 * IMPORTANT: Must be called by the caller after 
 * allocating the params struct when calling load_json,
 * in order to properly deallocate the struct 
*/
void free_params(Params * params) {

    free_float_matrix(
        params->g_image,
        calculate_steps(params->y_min, params->y_max, params->r),
        calculate_steps(params->x_min, params->x_max, params->r)
    ); 
    free_float_matrix(
        params->elliptical_obstacles,
        params->num_obstacles,
        params->num_obstacle_parameters
    );
    free(params);
}

/************* store_file *************
 * 
 * This function takes as argument the filepath for the 
 * .json file to parse, and saves it into a dynamically allocated 
 * string, returning the resulting char * 
 * 
 * NOTE: The caller is critically responsible for later free'ing the returned char *,
 *       especially given that this is potentially a large chunk of memory allocation
 */
char * store_file(const char * file_name) {
    FILE *fp;
    int capacity = 1024; // Initial capacity for the buffer
    int it = 0;
    char c;
    char *buffer = malloc(capacity); // Dynamically allocate the initial buffer
    if (buffer == NULL) {
        DEBUG_ERROR("Memory allocation failed");
        return NULL;
    }

    if ((fp = fopen(file_name, "r")) == NULL) {
        DEBUG_ERROR("Failed to open file");
        free(buffer); // Clean up allocated memory
        return NULL;
    }

    while ((c = fgetc(fp)) != EOF) {
        if (it >= capacity - 1) { // Ensure there's room for more characters and a null terminator
            capacity *= 2; // Double the buffer size
            char *new_buffer = realloc(buffer, capacity);
            if (new_buffer == NULL) {
                DEBUG_ERROR("Memory reallocation failed");
                free(buffer); // Clean up allocated memory
                fclose(fp);
                return NULL;
            }
            buffer = new_buffer;
        }
        buffer[it++] = c;
    }
    buffer[it] = '\0'; // Null-terminate the string

    fclose(fp);

    // Optionally reduce the buffer to the exact needed size
    char *final_buffer = realloc(buffer, it + 1);
    if (final_buffer == NULL) {
        DEBUG_ERROR("Final memory reallocation failed");
        free(buffer); // Use the original buffer if realloc fails
        return buffer;
    }
    return final_buffer;
}

/******* print_matrix_to_file **********
 * 
 * This helper function takes a 2-dimensional matrix with its corresponding numbers of rows and columns,
 * and writes it to the desired file pointer under the desired key 
*/
void print_matrix_to_file(FILE *fp, const char* key, float** matrix, int rows, int cols) {
    fprintf(fp, "\"%s\": [\n", key);
    for (int i = 0; i < rows; i++) {
        fprintf(fp, "    [");
        for (int j = 0; j < cols; j++) {
            fprintf(fp, "%.10f", matrix[i][j]);
            if (j < cols - 1) {
                fprintf(fp, ", ");
            }
        }
        fprintf(fp, "]");
        if (i < rows - 1) {
            fprintf(fp, ",\n");
        } else {
            fprintf(fp, "\n");
        }
    }
    fprintf(fp, "]");
}

/********** print_matrix_row *************
 * 
 * Prints single row of matrix for debugging purposes.
 */
void print_matrix_row(float ** M, int curr_col, int curr_row) {
    printf("Row %d of matrix:\n\t[ ", curr_row);
        for (int c = 0; c < curr_col; c++) {
            printf("M[%d, %d] = %f, ", curr_row, c, M[curr_row][c]);
        }
        printf("]\n");
}

/************** write_json ***************
 * 
 * This function takes a 2-dimensional matrix with its corresponding numbers of rows and columns,
 * and writes it to the desired file_path pointer under the desired key,
 * by calling the static / private subroutine print_matrix_to_file
*/
void write_json(const char* file_path, const char * key, float ** matrix, int rows, int cols) {
    FILE *fp = fopen(file_path, "r+");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file");
        return;
    }

    // Move to the end of the file to find the last closing bracket
    fseek(fp, -1, SEEK_END);
    long pos = ftell(fp);

    // Ensure we are placing the new key before the closing bracket of the JSON object
    while (pos > 0 && fgetc(fp) != '}') {
        fseek(fp, --pos, SEEK_SET);
    }

    if (pos > 0) {
        fseek(fp, -1, SEEK_CUR); // Move one character back to write before '}'
        fprintf(fp, ",\n"); // Proper JSON formatting with a comma before the new key
        print_matrix_to_file(fp, key, matrix, rows, cols);
        fprintf(fp, "\n}"); // Close the JSON object properly
    } else {
        fprintf(fp, "{\n"); // If file was empty, we start a new JSON object
        print_matrix_to_file(fp, key, matrix, rows, cols);
        fprintf(fp, "\n}");
    }

    fclose(fp);
}

/********** free_float_matrix ***********
 * 
 * Frees 2-dimensional matrix of given rows and columns.
*/
void free_float_matrix(float **M, int rows, int columns) {
    if (M != NULL) {
        for (int i = 0; i < rows; i++) {
            free(M[i]);  // Free each row
        }
        free(M);  // Finally, free the pointer to the row pointers
    }
}

/***************** Unit testing area *******************/

#ifdef PARS_DBG // Execute as standalone program for debugging only

int main(){   

    FILE * dfp; // global debugging file pointer
    const char * input_json = "./params.json";
    const char * output_test = "./test.txt";
    
    if ((dfp = fopen(output_test, "w")) == NULL) {
        fprintf(stderr, "Error: failed to open debugging file %s\n", output_test);
        return 1;
    }

    // Example usage of params struct
    Params * params = load_json(input_json);
    // write test matrix to same .json file
    write_json( 
        input_json, "g_image_test", params->g_image,
        calculate_steps(params->y_min, params->y_max, params->r),
        calculate_steps(params->x_min, params->x_max, params->r)
    );

    // Clean up
    free_params(params);

    fclose(dfp);

    return 0;
}

#endif


