
#include "parser.h"

// #define PARS_DBG 1
#define BUFF_SIZE 128 // Maximum buffer size for names of parameters

FILE * dfp; // global debugging file pointer

/* Internal constants */
const char arr_open ='[';
const char arr_close = ']';
const char delim = ',';

/* Static Helpers */
static char * store_file(const char * file_name);
static void remove_spaces(char * str);
static void print_matrix_to_file(FILE *fp, const char* key, float** matrix, int rows, int cols);
static void free_float_matrix(float ** M, int rows, int columns);
static float ** parse_float_matrix(const int max_col, const int max_row, char ** token_start);
static void insert_matrix(set_t * params, char ** last_token, const int columns, const int rows);
static set_t * parse_json_string(char * input_str);

float ** parse_float_matrix(const int max_col, const int max_row, char ** token_start) {

    #ifdef ASTCLCK
    struct timeval start, end;
    long seconds, microseconds;
    double elapsed;
    gettimeofday(&start, NULL); // Start clock 
    #endif

    float ** M;       // Array of row pointers
    float curr_float; 
    int curr_row = 0;
    int curr_col = 0;
    
    if ((M = malloc(max_row * sizeof(float *))) == NULL) {
        DEBUG_ERROR("malloc failure for M");
        return NULL;
    }

    while (**token_start && **token_start != arr_open) { 
        (*token_start)++;
    }
    if (**token_start == '\0') {
        DEBUG_ERROR("No opening bracket found");
        free(M);
        return NULL;
    }

    while (**token_start && **token_start != arr_close && curr_row < max_row) {
        while (**token_start && **token_start == arr_open) { 
            (*token_start)++; // skip open brackets
            // Could use a depth/dimension counter here
        }
        /* start of ROW */
        if ((M[curr_row] = malloc(max_col * sizeof(float))) == NULL) {
            DEBUG_ERROR("malloc failure for row of M");
            return NULL;
        }
        curr_col = 0; // reset column to zero
        // Now that (*token_start) points directly at first float digit, catch it with sscanf
        while (**token_start && (**token_start != arr_close) && curr_col < max_col &&
            (sscanf((*token_start), "%f,", &curr_float) == 1 ||
             sscanf((*token_start), "%f]", &curr_float) == 1)) {
            M[curr_row][curr_col++] = curr_float; // if parsing successful, store in matrix
            while (**token_start && **token_start != delim && **token_start != arr_close) {
                (*token_start)++; // advance token_start to next delimiter; , or ]
            }
            while (**token_start == delim) {
                (*token_start)++;  // Skip the comma & ]
            }
        }
        // This generally terminates because **token_start == ']'
        #ifdef PARS_DBG
        // print_matrix_row(M, curr_col, curr_row);
        #endif

        while (**token_start == delim || **token_start == arr_close ) {
                (*token_start)++; // Skip closed bracket and comma, aiming to reach next open bracket if possible
        }
        if (**token_start != arr_open) {
            #ifdef PARS_DBG   // If we can't find an open bracket for the next row, END OF MATRIX
            printf("No open bracket found; parse_float_matrix terminates at curr_row = %d, curr_col = %d\n", curr_row, curr_col);
            #endif
            break;
        }
        curr_row++;
    }

    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf("\nparse_float_matrix took %.5f seconds\n", elapsed);
    #endif

    return M; 
}


/* Helper to insert matrix in set given parameter entries; pushes last_token to start of next matrix */
void insert_matrix(set_t * params, char ** last_token, const int columns, const int rows) {

    char key[BUFF_SIZE]; // Buffer holding the key for matrix that will be inserted in set

    if (sscanf(*last_token, "\"%[^\"]\"", key) != 1) { // parse key of matrix, ignoring rest of string
        DEBUG_ERROR("sscanf failed to extract key or value_param from last token");
    }

    // Parse & allocate matrix --> last_token should be pointing at " character of start of next matrix after the call
    float ** M = parse_float_matrix(columns, rows, last_token);
    set_insert(params, key, M); // insert matrix pointer in set

    #ifdef PARS_DBG // TEST both retrieval from the set and matrix structure
    printf("After parsing matrix with key = %s, *last_token = %c and saved matrix retrieved as:\n", key, **last_token);
    float ** test_matrix = set_find(params, key); //attempt to retrieve matrix by the key with which it was stored 
    print_matrix_to_file(stdout, key, test_matrix, rows, columns);
    #endif
}

set_t * parse_json_string(char * input_str) {

    set_t * params; // this will be the set containing all the parameters; analogously to the json object in Python
    const char pre_delim = '#'; // remove tabs as well
    const char * delims = "{}#"; // VERY precise "cooki-cutter" delimiters
    char * pre_it = input_str; 
    char * token;
    char * last_token;
    float * value_param;
    char key[BUFF_SIZE]; // Buffer holding the key that will be inserted in set

    // Guard against null strings 
    if (input_str == NULL) {
        DEBUG_ERROR("Error: null input string");
        return NULL;
    }
    /** First remove the commas and new-lines only up to the arrays, which will keep commas as separators **/
    while (*(pre_it++) != arr_open) { /*** CRUCIAL ASUMPTION: stop substitution at first open bracket found ***/
        if (*pre_it == delim || *pre_it == '\n'){
            *pre_it = pre_delim;
        }
    }
    /* Prepare new set data structure for incoming elements */
    if ((params = set_new()) == NULL) {
        DEBUG_ERROR("could not initiate a new set\n");
        return NULL;
    }
    /* Tokenization of parameters w/ modified delimiter */
    token = strtok(input_str, delims); // Get first token
    while (token != NULL) {
    remove_spaces(token);   // strip out the spaces from the token for easier parsing with sscanf
    /*** IMPORTANT: We have to malloc the item AND later free it but NOT the key (see set.h documentation) ***/
    value_param = malloc(sizeof(int)); 
    if (sscanf(token, "\"%[^\"]\":%f", key, value_param) != 2) {
        DEBUG_ERROR("sscanf failed to extract key or value_param; possibly start of .json matrix entries");
    } else if (!set_insert(params, key, value_param)) {
        DEBUG_ERROR("Failed to insert key in the set");
    }

    #ifdef PARS_DBG  // check that floats were effectively added to the set
    float * test_value;
    if ((test_value = set_find(params, key)) == NULL) {
        DEBUG_ERROR("Error insertng into set; key could not be found");
    } else {
        printf("Inserted key in set: %s; float: %f in set\n", key, *test_value);
    }
    #endif
    last_token = token; // keep track of last token
    token = strtok(NULL, delims); // proceed to next token
    }

    /** NOTE: Tokenization of matrices with HARDCODED keys ***/
    insert_matrix(
        params, &last_token, 
        * (float *) set_find(params, "num_obstacle_parameters"),
        * (float *) set_find(params, "num_obstacles")
    ); 

    insert_matrix(
        params, &last_token, 
        calculate_steps(
            * (float *) set_find(params, "x_min"),
            * (float *) set_find(params, "x_max"),
            * (float *) set_find(params, "r")
            ), 
        calculate_steps(
            * (float *) set_find(params, "y_min"),
            * (float *) set_find(params, "y_max"),
            * (float *) set_find(params, "r")
            )
    ); 

    return params;
}

Params * load_json(const char * file_name) {

    char * copy_str = store_file(file_name);
    set_t * params_set = parse_json_string(copy_str);

    if (params_set == NULL) {
        DEBUG_ERROR("Error extracting parameters from .json");
        return NULL;
    }

    // Allocate space 
    Params * params = malloc(sizeof(Params));
    
    // Extract parameters from the set, paying close attention to casting, and save them to struct 
    params->r = * (float *) set_find(params_set, "r");
    params->s = * (float *) set_find(params_set, "s");
    params->x_max = * (float *) set_find(params_set, "x_max");
    params->x_min = * (float *) set_find(params_set, "x_min");
    params->y_min = * (float *) set_find(params_set, "y_min");
    params->y_max = * (float *) set_find(params_set, "y_max");
    params->x_g = * (float *) set_find(params_set, "x_g");
    params->y_g = * (float *) set_find(params_set, "y_g");
    params->x_s = * (float *) set_find(params_set, "x_s");
    params->y_s = * (float *) set_find(params_set, "y_s");
    params->g_max = * (float *) set_find(params_set, "g_max");

    // Hard-code the division so it doesn't need to be carried out repeatedly
    params->s_div_r = params->s / params->r;

    // Calculate the total number of s_steps based on the input parameters
    params->xs_steps = calculate_steps(params->x_min, params->x_max, params->s);
    params->ys_steps = calculate_steps(params->y_min, params->y_max, params->s);
    // Calculate the total number of r_steps based on the input parameters
    params->xr_steps = calculate_steps(params->x_min, params->x_max, params->r);
    params->yr_steps = calculate_steps(params->y_min, params->y_max, params->r);

    params->num_obstacles = * (int *) set_find(params_set, "num_obstacles");
    params->num_obstacle_parameters = * (int *) set_find(params_set, "num_obstacle_parameters");

    // Extract already matrix containg g_image, handling conversions carefully
    params->g_image = set_find(params_set, "g_image");
    params->elliptical_obstacles = set_find(params_set, "elliptical_obstacles");
    
    /** ISSUE: Set entries tedious to delete manually since some are float **, some float * */

    //?? Can we free(copy_str); here ??// 

    return params;
}


#ifdef PARS_DBG // Execute as standalone program for debugging only

int main(){   

    const char * input_json = "./params.json";
    const char * output_test = "./test.txt";
    
    if ((dfp = fopen(output_test, "w")) == NULL) {
        fprintf(stderr, "Error: failed to open debugging file %s\n", output_test);
        return 1;
    }

    // Testing:
    Params * params = load_json(input_json);
    // write test matrix to same .json file
    write_json( 
        input_json, "g_image_test", params->g_image,
        calculate_steps(params->y_min, params->y_max, params->r),
        calculate_steps(params->x_min, params->x_max, params->r)
    );

    // Matrices can be manually removed and free'd from the set for now
    // Tedious to free them through set_iterate due to the different parameters involved 
    free_float_matrix(
        params->g_image,
        calculate_steps(params->y_min, params->y_max, params->r),
        calculate_steps(params->x_min, params->x_max, params->r)
    ); // example deleting g_image matrix

    fclose(dfp);

    return 0;
}

#endif

/* Function definitions */
/* Dynamically allocating a file here */
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

void remove_spaces(char * str) {
    char * write = str;  // Pointer to the position to write the next non-space character
    while (*str != '\0') {
        if (!isspace((unsigned char)*str)) {  // Check if the character is not a space
            *write++ = *str;  // Write it down if it's not a space
        }
        str++;  // Move to the next character
    }
    *write = '\0';  // Null-terminate the modified string
}

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

void print_matrix_row(float ** M, int curr_col, int curr_row) {
    printf("Row %d of matrix:\n\t[ ", curr_row);
        for (int c = 0; c < curr_col; c++) {
            printf("M[%d, %d] = %f, ", curr_row, c, M[curr_row][c]);
        }
        printf("]\n");
}

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

void free_float_matrix(float **M, int rows, int columns) {
    if (M != NULL) {
        for (int i = 0; i < rows; i++) {
            free(M[i]);  // Free each row
        }
        free(M);  // Finally, free the pointer to the row pointers
    }
}

