
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "vertex.h"

#define FH_DBG 1 // unit testing 

/* Interface for the open set / priority queue used by A* */

typedef struct open_set open_set_t;  // Forward declaration of the structure

/* Functions for open set / priority queue */
bool open_set_is_empty(open_set_t *set);
open_set_t *open_set_new();
void insert_sorted(open_set_t *set, vertex_t *new_vertex);
vertex_t *dequeue_min(open_set_t *set);

// void free_open_set(open_set_t *set); 




