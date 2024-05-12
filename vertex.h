
#pragma once 

#include <stdbool.h>
#include "Lval_list.h"

struct minheap_node; // forward declaration of min_heap node

typedef struct vertex {
    int x_s, y_s; // discretized s-steps; vertex would be located at matrix entry M[y_s][x_s]
    struct vertex * parent; // defaults to NULL is no parent
    float g_score; 
    float f_score; // used in priority queue
    // struct vertex * next; // Pointer to the next vertex in linked list implementation of priority queue
    struct minheap_node * minheap_node; // keep track of corresponding node in min_heap (zero if not in minheap)
    bool is_evaluated; // keeps track of whether vertex added to closed set or not

    /* Homotopy addition */
    Lval_list_t * L_values;    // The list of Lvalues for the vertex

} vertex_t;

