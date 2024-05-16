
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "vertex.h"
#include "hom_classes.h"
#include "commonmacros.h"

/* Interface for the minimum heap open set / priority queue used by A* */

/** NOTE: 
 * Because we want to use this data structure for both the "standard" A* algorithm where we enqueue vertices,
 * and the extended version of the algorithm where we actually enqueue homotopy class objects, 
 * we are going to implement "type punning" to allow the data structure to hold a generic type.
 * This will also force further clarity and abstraction in explicitly defining which elements 
 * from vertices / homotopy classes the data structure actually needs to read and write to 
*/

typedef struct open_set open_set_t;  
typedef struct minheap_node minheap_node;

/* Prototypes for supported generic datatypes */
float vertex_get_f_score (void * vertex);
float hom_class_get_f_score (void * hom_class);
void vertex_set_minheap_node (void * vertex, minheap_node * minheap_node);
void hom_class_set_minheap_node (void * hom_class, minheap_node * minheap_node);

/* Functions for open set / priority queue */
open_set_t *open_set_new(
    float (* get_f_score) (void *),
    void  (* set_minheap_node) (void *, minheap_node *));
bool open_set_is_empty(open_set_t *set);
void insert_sorted(open_set_t *set, void * data);
void * dequeue_min(open_set_t *set);
void decrease_key(open_set_t *set, minheap_node *node, float new_key);
void free_open_set(open_set_t *set); 

/* Simple getters */
int open_set_size(open_set_t * set);



