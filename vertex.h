
#pragma once 

#include <stdbool.h>
#include "hom_classes.h"

struct minheap_node; // forward declaration of min_heap node

typedef struct vertex {
    /* Homotopy class independent properties of each vertex */
    int x_s, y_s;                       // discretized s-steps; vertex would be located at matrix entry M[y_s][x_s]
    struct minheap_node * minheap_node; // keep track of corresponding node in min_heap (zero if not in minheap)
    bool is_evaluated;                  // keeps track of whether vertex added to closed set or not

    /* Homotopy class relative properties, kept at the vertex level here to reflect the minimum across all homotopy classes */
    struct vertex * parent; 
    float g_score; 
    float f_score;

    /* Linked list containing all homotopic classes through which the vertex is reachable, with relative properties */
    hom_classes_list_t * hom_classes;    

} vertex_t;

