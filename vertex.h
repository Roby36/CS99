
#pragma once 

#include <stdbool.h>
#include "hom_classes.h"

struct minheap_node; // forward declaration of min_heap node

typedef struct vertex {

    /* Homotopy class independent properties of each vertex */
    int x_s, y_s;   // discretized s-steps; vertex would be located at matrix entry M[y_s][x_s]

    /* Linked list containing all homotopic classes with the vertex as endpoint */
    hom_classes_list_t * hom_classes;    

    /** NOTE:
     * In the case of a homotopic class based search, vertices are merely lists of homotopy classes
     * corresponding to each point of the grid, and hence these properties are removed from vertices 
     * as they are specific to homotopy class objects:
    *
        struct minheap_node * minheap_node; // keep track of corresponding node in min_heap (zero if not in minheap)
        bool is_evaluated;                  // keeps track of whether vertex added to closed set or not
        struct vertex * parent; 
        float g_score; 
        float f_score;
    */

} vertex_t;

