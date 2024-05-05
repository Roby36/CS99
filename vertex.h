
#pragma once 

#include <stdbool.h>

typedef struct vertex {
    int x_s, y_s; // discretized s-steps; vertex would be located at matrix entry M[y_s][x_s]
    struct vertex * parent; // defaults to NULL is no parent
    float g_score; 
    float f_score; // used in priority queue
    struct vertex * next; // Pointer to the next vertex in priority queue
    bool is_evaluated; // keeps track of whether vertex added to closed set or not

} vertex_t;

