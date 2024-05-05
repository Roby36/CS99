
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "vertex.h"

// #define LL_DBG 1 // unit testing 

/* Interface for the open set / priority queue used by A* */

typedef struct open_set open_set_t;  // Forward declaration of the structure

/* Functions for open set / priority queue */
bool open_set_is_empty(open_set_t *set);
open_set_t *open_set_new();
void insert_sorted(open_set_t *set, vertex_t *new_vertex);
vertex_t* extract_sorted(open_set_t *set, vertex_t *vertex);
vertex_t *dequeue_min(open_set_t *set);
void print_open_set(open_set_t *set);
bool is_in_open_set(open_set_t *set, vertex_t *vertex);
void free_open_set(open_set_t *set); 
void open_set_testing();


/* Include open test testing function here, so that we can execute same tests for different implementations */

#ifdef LL_DBG

void open_set_testing() {
    // Initialize the open set
    open_set_t *set = malloc(sizeof(open_set_t));
    set->head = NULL;

    // Create and insert some vertices
    vertex_t *vertex1 = malloc(sizeof(vertex_t));
    vertex1->x_s = 1; vertex1->y_s = 1; vertex1->f_score = 10.5;
    insert_sorted(set, vertex1);

    vertex_t *vertex2 = malloc(sizeof(vertex_t));
    vertex2->x_s = 2; vertex2->y_s = 2; vertex2->f_score = 5.5;
    insert_sorted(set, vertex2);

    vertex_t *vertex3 = malloc(sizeof(vertex_t));
    vertex3->x_s = 3; vertex3->y_s = 3; vertex3->f_score = 7.5;
    insert_sorted(set, vertex3);

    vertex_t *vertex4 = malloc(sizeof(vertex_t));
    vertex3->x_s = 3; vertex3->y_s = 3; vertex4->f_score = 40;
    insert_sorted(set, vertex4);

    vertex_t *vertex5 = malloc(sizeof(vertex_t));
    vertex3->x_s = 3; vertex3->y_s = 3; vertex5->f_score = 112;
    insert_sorted(set, vertex5);

    vertex_t *vertex6 = malloc(sizeof(vertex_t));
    vertex3->x_s = 3; vertex3->y_s = 3; vertex6->f_score = 0.5;
    insert_sorted(set, vertex6);

    vertex_t *vertex7 = malloc(sizeof(vertex_t));
    vertex3->x_s = 3; vertex3->y_s = 3; vertex7->f_score = 65;
    insert_sorted(set, vertex7);

    // Print the open set
    print_open_set(set);

    // Dequeue the minimum and print the updated open set
    vertex_t *min_vertex = dequeue_min(set);
    printf("Dequeued Vertex: (%d, %d) with f_score: %f\n", min_vertex->x_s, min_vertex->y_s, min_vertex->f_score);
    print_open_set(set);

    min_vertex = dequeue_min(set);
    printf("Dequeued Vertex: (%d, %d) with f_score: %f\n", min_vertex->x_s, min_vertex->y_s, min_vertex->f_score);
    print_open_set(set);

    // Free the minimum vertex and the open set
    free(min_vertex);
    free_open_set(set);
}

#endif 


