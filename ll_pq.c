
#include "open_set.h"

/* Internal structure of ll priority queue, abstracted from the user */
struct open_set {
    vertex_t *head;  // Pointer to the head of the priority queue
};

/** Open set / priority queue functions **/

bool open_set_is_empty(open_set_t *set) {
    return set->head == NULL;
}

open_set_t *open_set_new() {
    // Allocate memory for the open_set structure
    open_set_t *open_set = malloc(sizeof(open_set_t));
    if (open_set == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for open set.\n");
        return NULL; // Return NULL to indicate failure
    }

    // Initialize the head of the list to NULL
    open_set->head = NULL;

    return open_set; // Return the pointer to the newly created open set
}

void insert_sorted(open_set_t *set, vertex_t *new_vertex) {

    // Debug before insertion
    #ifdef ASTDBG
    printf("Inserting vertex (%d, %d) with f_score: %f\n", new_vertex->x_s, new_vertex->y_s, new_vertex->f_score);
    #endif

    // Insertion logic 
    vertex_t **tracer = &(set->head);
    while (*tracer != NULL && (*tracer)->f_score < new_vertex->f_score) {
        tracer = &(*tracer)->next;
    }

    new_vertex->next = *tracer;
    *tracer = new_vertex;
}

vertex_t* extract_sorted(open_set_t *set, vertex_t *vertex) {

    // Start at the head of the list
    vertex_t **tracer = &(set->head);
    // Traverse the list to find the target vertex
    while (*tracer != NULL && *tracer != vertex) {
        tracer = &(*tracer)->next;
    }

    // Check if the vertex was found
    if (*tracer == NULL) {
        // Vertex not found
        return NULL;
    }

    // Remove the vertex from the list
    vertex_t *extracted_vertex = *tracer; // This is the vertex to be extracted
    *tracer = extracted_vertex->next; // Bypass the extracted vertex in the list
    extracted_vertex->next = NULL; // Clear the next pointer of the extracted vertex
    return extracted_vertex; // Return the pointer to the extracted vertex
}


vertex_t *dequeue_min(open_set_t *set) {

    // Debug before dequeueing
    #ifdef ASTDBG
    if (set->head) {
        printf("Dequeueing vertex (%d, %d) with f_score: %f\n", set->head->x_s, set->head->y_s, set->head->f_score);
    }
    #endif

    // Dequeue logic 
    if (set->head == NULL) return NULL;
    vertex_t *min_vertex = set->head;
    set->head = set->head->next;
    return min_vertex;
}

bool is_in_open_set(open_set_t *set, vertex_t *vertex) {

    vertex_t *current = set->head;
    while (current != NULL) {
        if (current == vertex) {
            return true;
        }
        current = current->next;
    }
    return false;
}

// Function to free the open set: NOT USED since verteces systematically deallocated from grid already
void free_open_set(open_set_t *set) {
    vertex_t *current = set->head;
    while (current != NULL) {
        vertex_t *next = current->next;
        free(current);
        current = next;
    }
    free(set);
}


/* Testing funtions & helpers */
// Function to print the vertices in the open set
void print_open_set(open_set_t *set) {
    vertex_t *current = set->head;
    while (current != NULL) {
        printf("Vertex at (%d, %d) with f_score: %f\n", current->x_s, current->y_s, current->f_score);
        current = current->next;
    }
}


#ifdef LL_DBG

int main() {
    open_set_testing();
}

#endif
