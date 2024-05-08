
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "vertex.h"
#include "commonmacros.h"

/* Interface for the open set / priority queue used by A* */

typedef struct open_set open_set_t;  // Forward declaration of the structure
typedef struct minheap_node minheap_node;

/* Functions for open set / priority queue */
open_set_t *open_set_new();
bool open_set_is_empty(open_set_t *set);
void insert_sorted(open_set_t *set, vertex_t *new_vertex);
vertex_t *dequeue_min(open_set_t *set);
void decrease_key(open_set_t *set, minheap_node *node, float new_key);
void free_open_set(open_set_t *set); 



