
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "hom_vertex.h" // only for unit testing purposes
#include "hom_classes.h"
#include "commonmacros.h"

/* Opaque types */
typedef struct open_set open_set_t;  
typedef struct minheap_node minheap_node;

/* Prototypes for supported generic datatypes */
float vertex_get_f_score (void * vertex);
float hom_class_get_f_score (void * hom_class);
void vertex_set_minheap_node (void * vertex, minheap_node * minheap_node);
void hom_class_set_minheap_node (void * hom_class, minheap_node * minheap_node);

/* Public functions for the minimum heap implementation of the priority queue, documented in min_heap.c */
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
