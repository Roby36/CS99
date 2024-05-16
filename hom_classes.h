
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

// forward declarations (to avoid recursive inclusions)
struct minheap_node; 
struct vertex;

/* Details of path returned by A* */
typedef struct {
    int total_edges;

    float ** point_list;
    float absolute_length;
    float g_image_riemann_tot;

} Backtrack_Path;

/* These structs must be accessible to A* */
typedef struct hom_class {

    /* Homotopy class special properties */
    Complex Lval;
    struct hom_class * next;

    /* General properties of vertices, updated for each homotopy class */
    struct hom_class * parent; 
    float g_score; 
    float f_score;
    bool is_evaluated;  // track membership in the closed set

    /* Book-keeping properties */
    struct vertex * endpoint_vertex;
    struct minheap_node * minheap_node;

    /* This can be expanded when we have optimized the homotopy class */
    Backtrack_Path * backtrack;

} hom_class_t;

typedef struct hom_classes_list {
    
    hom_class_t *  head;
    hom_class_t ** last_ptr;

} hom_classes_list_t;


hom_classes_list_t *hom_classes_list_new();
void free_hom_classes_list(hom_classes_list_t *hom_classes_list);
hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);
bool insert_hom_class(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol);
hom_class_t * hom_class_get_min_f(hom_classes_list_t *hom_classes_list);
char* complex_list_to_string(hom_classes_list_t *list);

