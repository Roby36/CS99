
#pragma once

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "utils.h"
#include "cost_funcs.h" // for expanding paths 
#include "parser.h" // for writing paths to .json

typedef double complex Complex;

/***** forward declarations *******/
typedef struct hom_classes_list hom_classes_list_t;
typedef struct hom_class hom_class_t;
struct minheap_node; 
struct hom_vertex;

/****************************** hom_vertex *****************************/

typedef struct hom_vertex {

    /* Homotopy class independent properties of each vertex */
    int x_s, y_s;   // discretized s-steps; vertex would be located at matrix entry M[y_s][x_s]

    /* Linked list containing all homotopic classes with the vertex as endpoint */
    hom_classes_list_t * hom_classes;    

} hom_vertex_t;

/************* simple constructor for homotop vertex *************/
hom_vertex_t * hom_vertex_new(
    int x_s, int y_s, 
    int max_hom_classes,  
    bool (*less_than)(hom_class_t *, hom_class_t *)
);

/****************************** Backtrack_Path *****************************
 * 
 * This struct holds a 2-dimensional path,
 * and keeps both components of the path's cost
*/
typedef struct {
    int total_edges;

    float ** point_list;
    float absolute_length;
    float g_image_riemann_tot;

} Backtrack_Path;


/************************************** hom_class ***************************************
 * 
 * This struct holds the main data defining a homotopy class:
 * 
 *  - Lval is the descriptor for the homotopy class,
 *    defined as the integral of the obstacle marker function 
 *    between the start vertex and endpoint vertex of the homotopy class
 * 
 *  - the next field is a special element to conveniently place the homtopy class
 *    within a linked list data structure.
 * 
 *  - the homotopy class holds all the attributes required to 
 *    be enqueued in the priority queue for the A* algorithm,
 *    such as g_score, f_score, etc.
 * 
 *  - the backtrack path is generally uninitialized, and is initialized by 
 *    calling the method expand_backtrack in the astar.c module when 
 *    the homotopy class is worth expanding (saves space since we will be enqueuing hundreds 
 *    of thousands of these before getting to the goal) 
 */
struct hom_class {

    /* Homotopy class special properties */
    Complex Lval;
    struct hom_class * next;
    bool updated;   // keeps track of which homotopy classes were updated during a single A* iteration

    /* General properties of vertices, updated for each homotopy class */
    struct hom_class * parent; 
    float g_score; 
    float f_score;
    bool is_evaluated;  // track membership in the closed set

    /* Book-keeping properties */
    struct hom_vertex * endpoint_vertex;
    struct minheap_node * minheap_node;

    /* This can be expanded when we have optimized the homotopy class */
    Backtrack_Path * backtrack;

};

/********************* hom_class_new ******************
 * 
 * NOTE: This constructor initializes mainly the CONSTANT fields.
 *       Caller is responsible to keep g score, f score, and parent
 *       updated throughout the program
 */
hom_class_t * hom_class_new(hom_vertex_t * endpoint_vertex, Complex Lval);

/********************** hom_class_free *******************************
 * 
 * Destructor, to be invoked whenever we need to free a homotopy class
 * that we allocated by calling hom_class_new
*/
void hom_class_free(hom_class_t * hc);

/** g_score_compare
 * 
 * Simple function intended to be passed as the default less_than operator
 * field for hom_classes_list_t. Acts like a "<" operator between
 * the first and second homotopy classes passed, comparing them in f_score.
 * 
 * NOTE: insert_hom_sorted is the ONLY function of the module invoking
 *      this operator, and NULL-checks both arguments, 
 *      to prevent undefined behaviour
*/
bool g_score_compare(hom_class_t * hc_1, hom_class_t * hc_2);

/********************* expand_backtrack ****************
 * 
 * This function uses the information stored within the hom_class_t, set by the A* procedure
 * to carefully backtrack and allocate the path of the homotopy class. It is convenient because
 * it allows us to arbitrarily determine when a path is worth allocating and expanding at any time
 * we want to do so, by just using lightweight information stored in the homotopy class struct.
 * 
 * Inputs:
 *      Params struct for the given environment
 *      homotopy class struct pointer for which we want to expand and store the path
 * 
 * Outputs:
 *      The nested struct member hom_class->backtrack is updated with all the information on the path
 * 
 * NOTE: Caller must then call the corresponding clean-up function free_backtrack 
 *       which carefully free's all the memory malloc'd by this function
 * 
*/
void expand_backtrack(Params *params, hom_class_t * hom_class);

/************************ write_path ***************************
 * 
 * This function writes the points of the output path corresponding to some 
 * cost coefficients a,b and homotopy class to the .json output file,
 * so that it can be parsed and rendered by graphing software (i.e. matplotlib)
 * 
 * Inputs:
 *      homotopy class object containing the path
 *      config struct containing a,b coefficients for cost function on which path was evaluated
 * 
 * Outputs:
 *      Path points written out as a matrix to ./params.json file (name hardcoded)
 * 
 * NOTE: The caller must first invoke expand_backtrack(params, hom_class)
 *       to ensure that the backtrack path is both computed and stored in hom_class->backtrack,
 *       and, preferably, invoke write_path before free_backtrack which deletes the path
 * 
 * Usage example:
 * 
 *  expand_backtrack(params, hom_class);
 *  // do something with the path
 *  write_path(hom_class, config);
 *  // do something with the path
 *  free_backtrack(hom_class);
 * 
 */
void write_path(hom_class_t * hom_class, Config * config, char * filepath);

/********************* free_backtrack *********************
 * 
 * The corresponding clean-up routine to expand_backtrack.
 * 
 * Inputs:
 *      homotopy class struct whose backtrack data member we want to free
 * 
 * Outputs:
 *      The function carefully deallocates all the memory taken up by
 *      hom_class->backtrack 
*/
void free_backtrack(hom_class_t * hom_class);


/*********************************************** hom_classes_list ********************************************
 * 
 * This struct uses the struct hom_class * next field of homtopy classes
 * to join them together into a simple linked list.
 * 
 * Because these will rarely contain over 20 homotopy classes,
 * we can such simple data structure without efficiency concerns.
 * 
 * Additionally, we keep some maximum capacity of the homotopy class
 * list. The data structure will ensure that only the (capacity) best 
 * homotopy classes by some defined metric (probably f-score) will
 * be kept into the data structure, and the others ignored.
 */
struct hom_classes_list {
    
    hom_class_t * head;
    int capacity;   // constant maximum number of homotopy classes allowed into list at any time
    int size;       // curent number of homotopy classes in the list
    bool (*less_than)(hom_class_t *, hom_class_t *);

};

/************* Public functions for the hom_classes_list module: Documented in detail in hom_classes.c *****************/

/************************* hom_classes_list_new *************************** 
 * 
 * This function allocates a linked list data structure containing homotopy classes,
 * and returns its corresponding pointer.
 * 
 * Inupts:
 *      - capacity: maximum number of homotopym classes to be accepted into the the list
 *      - less_than: the operator < to determine whether the first hc is smaller than the second
 * 
 * NOTE: Caller responsible for calling free_hom_classes_list on the pointer returned 
 *       to free the memory malloc'd by this function 
 * 
 * IMPORTANT: This module NEVER malloc's any hom_class_t objects. Caller fully responsible to allocate 
 *            and manage those out of these functions' scope
 */
hom_classes_list_t *hom_classes_list_new(int capacity, bool (*less_than)(hom_class_t *, hom_class_t *));

/************************* free_hom_classes_list ******************************
 * 
 * This is the corresponding clean-up routine for hom_classes_list_new,
 * called when we finished using a hom_classes_list_t struct.
 * 
 * IMPORTANT: This is the ONLY method free'ing hom_class_t, ensuring that the caller does not 
 *            have the overhead to track which ones were free'd already. This has two main implications:
 *      1) Caller fully responsible to free the objects removed from the list before calling this function 
 *      2) Caller must prevent double free's, by not free'ing any hom_class_t after calling this function,
 *         unless they were previously removed from the list
 */ 
void free_hom_classes_list(hom_classes_list_t *hom_classes_list);

/****************************** hom_class_get ***************************
 * 
 * Inputs:
 *  - hom_classes_list_t struct pointer within which we want to search a homotopy class with some Lvalue descriptor
 *  - Complex-valued L-value we want to search for in the linked list of homotopy classes
 *  - double absolute tolerance value within which we are expecting to find the L-value
 * 
 * Output:
 *  - If the homotopy class with the desired L-value is present, return directly the pointer to it
 *  - If no homotopy class with the desired L-value is present, return NULL 
 * 
 * IMPORTANT: NO allocations or deallocations, and class remains in the list
 *      Caller should NEVER free the returned pointer, to avoid double free due to free_hom_classes_list
 */
hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);

/************************** insert_hom_class_sorted *****************************
 * 
 * The aim of this method is to maintain a pool of the currently best homotopy classes
 * into some vertex, which is updated as we try to insert new homotopy classes.
 * 
 * Inputs:
 *  - hom_classes_list_t struct pointer within which we desire to insert a new homotopy class
 *  - pointer to already-allocated homotopy class struct that we want to insert
 *  - double absolute tolerance value to determine if the homotopy class is already in the list
 *  - double pointer holding the truncated homotopy class
 * 
 * Outputs:
 *  - false if the list is full to capacity, and the homotopy class we are trying to insert 
 *    does not qualify (i.e. worse than all the homotopy classes already present in the list according to the less_than operator)
 *  - true if no homotopy class with the same L-value is present in the list,
 *    and the homotopy class is within the best (capacity) homotopy classes in the list according to less_than,
 *    and homotopy class is therefore successfully inserted into the linked list
 * -  the double-pointer to_remove is set to the just-truncated element which needs to be managed by the caller, 
 *    or NULL if no truncation occured
 *  
 * 
 * NOTE: This function requires an already-allocated homtopy class struct, and the caller is responsible for providing that.
 *       This function will NEITHER allocate NOR free any hom_class_t objects, and the caller has full responsibility
 *       for that both in the case that the insertion occurs (potentially freeing truncated) and in the case when the 
 *       insertion does not occur (managing the failed-to-insert element) 
 */
bool insert_hom_class_sorted(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol, hom_class_t ** truncated);

/******************* complex_list_to_string ***********************
 * 
 * This function stringifies a homtopy class list struct, 
 * allocating dynamically a buffer and returning a char * to it
 * 
 * NOTE: Caller responsible for freeing returned char *
 */
char* complex_list_to_string(hom_classes_list_t *list);

/******************** unflag_homotopy_classes ***********************
 * 
 * This function unflags all the homotopy classes in a given list
 * to inform A* that they have not yet been updated by the current iteration
*/
void unflag_homotopy_classes(hom_classes_list_t *list);

