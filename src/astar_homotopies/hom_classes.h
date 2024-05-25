
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

/****** hom_classes_list ******
 * 
 * This struct uses the struct hom_class * next field of homtopy classes
 * to join them together into a simple linked list.
 * 
 * Because these will rarely contain over 20 homotopy classes,
 * we can such simple data structure without efficiency concerns.
 * 
 * The last_ptr points to the last element of the linked list to 
 * make insertion at the bottom easier and more efficient.
 */
typedef struct hom_classes_list {
    
    hom_class_t *  head;
    hom_class_t ** last_ptr;

} hom_classes_list_t;

/** Public functions for the hom_classes_list module 
 * Documented in detail in hom_classes.c
*/
hom_classes_list_t *hom_classes_list_new();
void free_hom_classes_list(hom_classes_list_t *hom_classes_list);
hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);
bool insert_hom_class(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol);
char* complex_list_to_string(hom_classes_list_t *list);
void unflag_homotopy_classes(hom_classes_list_t *list);
