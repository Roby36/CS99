
/**
 * This module handles the data extraction process from an input .json file containing the main paramters
 * for the robot's environment, by first parsing the file, and then storing the parameters in a set.
 * It is thus a small implementation in C of the Python json package, and is particularly useful as it 
 * allows to change the parameters both programmatically and manually from the .json file shared amongst 
 * the modules, and access them conveniently in a dedicated data structure, rather than having to 
 * hardcode the constants. 
 * 
 * 
 *  * IMPORTANT: The following module crucially assumes a simplistic .json structure, 
 * where we firstly place all the easier-to-handle constants,
 * and then one or more long arrays of numbers: For example, something of teh form:
 * 
 * "r": 0.25,
    "x_min": -20,
    "x_max": 20,
    "y_min": -20,
    "y_max": 20,

    ...

    "elliptical_obstacles": [
        [
            2,
            3,
            4,
            3,
            0,
            0.8
        ],
        [
            6,
            7,
            2,
            3,
            0,
    ...
    ]

    NOTE: We can add as many constants in the upper part, and as many coordinate arrays like
    "elliptical_obstacles" in the lower part, as long as we 
    1) Keep the two sets of entries separated
    2) Avoid further depth of the nesting of the structure
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>  // For isspace()
#include "set.h"

/* Error MACRO */
#define DEBUG_ERROR(msg) fprintf(stderr, "Error: %s\nIn file: %s, line: %d, function: %s\n", msg, __FILE__, __LINE__, __func__)

#define MAX_FILE_CHARS 1048576
#define DBG 1
#define BUFF_SIZE 32 // Maximum buffer size for names of parameters

const char * input_json = "./environment.json";
const char * output_test = "./test.txt";
FILE * dfp;

/****
 * This method returns a malloc'd string containing the entire file content 
 * IMPORTANT: Returned string must be free'd
*/
char * store_file(const char * file_name) {

    char buffer[MAX_FILE_CHARS];
    int it = 0;
    char c;
    FILE * fp;

    if ((fp = fopen(file_name, "r")) == NULL) {
        DEBUG_ERROR("failed to open file");
        return NULL;
    }

    while ((c = fgetc(fp)) != EOF) {
        buffer[it++] = c;
    }
    char * dest_str = malloc(it + 1);
    /*** NOTE: Not checking for NULL malloc returns ***/
    strncpy(dest_str, buffer, it + 1);
    fclose(fp);
    return dest_str;
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

/**
 * This is one of the key functions in the module, which fully parses a matrix from .json,
 * thereby acting as ahelper-function for the main fucntion of the module.
 * The aim is also to directly modify the given char * to push it to the start of the      successive matrix,thus allowing to run the function repeatedly
 * This function rests on the CRUCIAL assumption of a 2x2 matrix
*/
float ** parse_float_matrix(const int max_col, const int max_row, char ** token_start) {

    const char arr_open  ='[';
    const char arr_close = ']';
    float ** M;       // Array of row pointers
    float curr_float; 
    int curr_row = 0;
    int curr_col = 0;
    
    // Start by allocating an array of matrix row pointers
    if ((M = malloc(max_row * sizeof(float *))) == NULL) {
        DEBUG_ERROR("malloc failure for M");
        return NULL;
    }
    // Now we start iterating through the data, and each time we finish a row, 
    // we dynamically allocate space for the next one
    while (**token_start && **token_start != arr_open) { /** IMPORTANT: must always check we don't get the zero character string! */
        (*token_start)++;
    }  // advance until we encounter the first, and onece we break the loop we are beyond
    if (**token_start == '\0') {  // Check if the end of the string was reached
        DEBUG_ERROR("No opening bracket found");
        free(M);
        return NULL;
    }
    // At this point (*token_start) would be pointing to exactly the first [ bracket unless finished, thus needs a push forward: 
    while (**token_start && **token_start != arr_close && curr_row < max_row) {
        (*token_start)++;
    #ifdef DBG
        printf("reached row %d, and column %d, with token_start = %c\n", curr_row, curr_col, **token_start);
    #endif
        /* start of ROW */
        // Allocate space for row
        if ((M[curr_row] = malloc(max_col * sizeof(float))) == NULL) {
            DEBUG_ERROR("malloc failure for row of M");
            return NULL;
        }
        curr_col = 0; // reset column to zero
        // Now that (*token_start) points directly at first float digit, catch it with sscanf
        while (**token_start && (**token_start != arr_close) && curr_col < max_col &&
            (sscanf((*token_start), "%f,", &curr_float) == 1 ||
             sscanf((*token_start), "%f]", &curr_float) == 1)) {

    #ifdef DBG
            printf("sscanf: Float recorded: %f\n", curr_float);
    #endif
            // Store & increment column
            M[curr_row][curr_col++] = curr_float;
            // Move to the next float or end of the row
            while (**token_start && **token_start != ',' && **token_start != arr_close) {
                (*token_start)++;
            }
            while (**token_start == ',' || **token_start == arr_close) {
                (*token_start)++;  // Skip the comma & ]
            }
        }
        curr_row++;

#ifdef DBG
        printf("reached END row %d, and column %d, with token_start = %c\n", curr_row, curr_col, **token_start);
#endif
    }
    return M; 
}
/**
 *  * This function assumes an already malloc'd string, and tokenizes it to extract
 * .json parameters
 * This function returns a malloc'd set data structure containing all the parameters 
***/
set_t * parse_json_string(char * input_str) {

    set_t * params; // this will be the set containing all the parameters; analogously to the json object in Python
    const char pre_delim = '#'; // remove tabs as well
    const char * delims = "{}#"; // VERY precise "cooki-cutter" delimiters
    const char arr_open ='[';
    const char arr_close = ']';
    const char delim = ',';
    char * pre_it = input_str; 
    char * token;
    char * last_token;
    float * value_param;
    float * test_value;
    char key[BUFF_SIZE]; // Buffer holding the key that will be inserted in set

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

    /** At this point we tokenize using pre_delim, breaking perfectly in constants and array with remaining commas and newlines which remain unaffected **/
    token = strtok(input_str, delims); // Get first token

    // Walk through other tokens
    while (token != NULL) {

    // Now that we pulled out the (dirty) token, we strip it from all its spaces
    remove_spaces(token);

#ifdef DBG
    fprintf(dfp,"[START]%s[END]", token);
#endif

    /*** IMPORTANT: We have to malloc the item but NOT the key
    * The key string will be allocated & freed internally by the module,
    * but we will need to manually allocate the item here, and free
    * upon the set deletion
    ***/

    value_param = malloc(sizeof(int)); // Allocate space for each item

    if (sscanf(token, "\"%[^\"]\":%f", key, value_param) == 2) {
#ifdef DBG
        printf("sscanf: The key is: %s\n", key);
        printf("sscanf: The float extracted is: %f\n", *value_param);
#endif
    } else {
        DEBUG_ERROR("sscanf failed to extract key or value_param");
    }

    if (!set_insert(params, key, value_param)) {
        DEBUG_ERROR("Failed to insert key in the set");
    }
#ifdef DBG
    if ((test_value = set_find(params, key)) == NULL) {
        DEBUG_ERROR("Error insertng into set; key could not be found");
    }
    printf("Inserted key in set: %s; float: %f in set\n", key, *test_value);
#endif
    last_token = token; // keep track of last token
    token = strtok(NULL, delims);
    }
    // Given the current separator "{}#", we will have a final token containing a 
    // lump of all remaining arrays at the bottom, which can be handled in a separate routine down here
#ifdef DBG
    fprintf(dfp,"\n\n[START]%s[END]", last_token);
#endif

    /*** NOTE: We can use the just-parsed values of the grid to know how much to allocate for the matrices */
    // First we can extract our key for the current matrix as for the other variables 
    if (sscanf(last_token, "\"%[^\"]\"", key) == 1) {
#ifdef DBG
        printf("sscanf: The key is: %s\n", key);
#endif
    } else {
        DEBUG_ERROR("sscanf failed to extract key or value_param");
    }
    // Then call subroutine to parse & allocate matrix
    float ** M = parse_float_matrix(
        * (int*) set_find(params, "num_obstacle_parameters"), 
        * (int*) set_find(params, "num_obstacles"), 
        &last_token);
    // Add to the set the matrix 
    set_insert(params, key, M);


    //!! LATER method to efficiently store all such matrices
    //!! LATER Testing to ensure BOTH matrices written correctly//
    //!! LATER free'ing method and ensure no memory leaks
    return params;
}


int main(){

    // Extract string from file
    char * copy_str = store_file(input_json);
    
#ifdef DBG
    if ((dfp = fopen(output_test, "w")) == NULL) {
        fprintf(stderr, "Error: failed to open debugging file %s\n", output_test);
        return 1;
    }
#endif

    // Testing:
    parse_json_string(copy_str);

    /*** IMPORTANT: DO NOT forget to free the string ***/
    free(copy_str);

    /*** IMPORTANT: SET "params" must be freed, calling free on each of the malloc'd items ***/

#ifdef DBG
    fclose(dfp);
#endif

    return 0;
}
