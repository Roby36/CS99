
#pragma once

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "commonmacros.h"
#include "utils.h"

typedef double complex Complex;

/* forward declarations */
struct minheap_node; 
struct hom_vertex;

/******* Backtrack_Path ******
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

/****** hom_class *******
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
typedef struct hom_class {

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

} hom_class_t;

/** g_score_compare
 * 
 * Simple function intended to be passed as the default less_than operator
 * field for hom_classes_list_t. Acts like a "<" operator between
 * the first and second homotopy classes passed, comparing them in f_score.
 * 
 * NOTE: If any of the two homotopy classes is NULL, this function returns false,
 * and thus may potentially lead to undefined behavior. Thus, the caller
 * should check carefully that NULL arguments are never passed. 
*/
bool g_score_compare(hom_class_t * hc_1, hom_class_t * hc_2);

/****** hom_classes_list ******
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
 * 
 * The last_ptr points to the last element of the linked list to 
 * make insertion at the bottom easier and more efficient.
 */
typedef struct hom_classes_list {
    
    hom_class_t * head;
    int capacity;   // constant maximum number of homotopy classes allowed into list at any time
    int size;       // curent number of homotopy classes in the list
    bool (*less_than)(hom_class_t *, hom_class_t *);

    hom_class_t ** last_ptr;

} hom_classes_list_t;

/** Public functions for the hom_classes_list module 
 * Documented in detail in hom_classes.c
*/
/***** hom_classes_list_new ******* 
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
 */
hom_classes_list_t *hom_classes_list_new(int capacity, bool (*less_than)(hom_class_t *, hom_class_t *));

/****** free_hom_classes_list ******
 * 
 * This is the corresponding clean-up routine for hom_classes_list_new,
 * called when we finished using a hom_classes_list_t struct.
 * 
 * NOTE: This function free's also the internal hom_class objects, 
 *        which were NOT allocated by the module itself but rather outside of it
 */
void free_hom_classes_list(hom_classes_list_t *hom_classes_list);

/****** hom_class_get *******
 * 
 * Inputs:
 *  - hom_classes_list_t struct pointer within which we want to search a homotopy class with some Lvalue descriptor
 *  - Complex-valued L-value we want to search for in the linked list of homotopy classes
 *  - double absolute tolerance value within which we are expecting to find the L-value
 * 
 * Output:
 *  - If the homotopy class with the desired L-value is present, return directly the pointer to it
 *  - If no homotopy class with the desired L-value is present, return NULL 
 */
hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);

/******* insert_hom_class_sorted ********
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
 * -  the double-pointer to_remove is set to the just-truncated element, or NULL if no element was truncated
 *  
 * 
 * 
 * NOTE: This function requires an already-allocated homtopy class struct, and the caller is responsible for that.
 *       The linked list is merely keyed by the L-values of homtopy classes. It is thus a perfect system to store 
 *       a small representative sample of distinct homotopy classes, but not suited for any type of exhaustive list.
 *
 */
bool insert_hom_class_sorted(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol, hom_class_t ** truncated);


/********* insert_hom_class ********
 * 
 * This is the more simplistic version of the function above that only fills up the list
 * up to the capacity and then stops accepting any more homotopy classes.
 * 
 * It is efficient if we assume that the homotopy classes are always initialized with f_score = INFINITY,
 * and thus already come with some sort of sorting logic.
*/
bool insert_hom_class(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol);

/***** complex_list_to_string *******
 * 
 * This function essentially stringifies a homtopy class list struct, 
 * allocating dynamically a buffer and returning a char * to it
 * 
 * NOTE: Caller responsibke for freeing returned char *
 */
char* complex_list_to_string(hom_classes_list_t *list);

/************* unflag_homotopy_classes *************
 * 
 * This function unflags all the homotopy classes in a given list
 * to inform A* that they have not yet been updated by the current iteration
*/
void unflag_homotopy_classes(hom_classes_list_t *list);
