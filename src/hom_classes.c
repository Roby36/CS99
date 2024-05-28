
#include "hom_classes.h"

// here to prevent duplicate symbol
const char * output_path_key_format = "a=%.2f, b=%.2f, L=(%.2f + %.2fi)";

/************************************** hom_vertex ***************************************/

hom_vertex_t * hom_vertex_new(
    int x_s, int y_s, 
    int max_hom_classes,  
    bool (*less_than)(hom_class_t *, hom_class_t *))
{
    hom_vertex_t * hv = malloc(sizeof(hom_vertex_t));
    if (hv == NULL) {
        DEBUG_ERROR("Malloc error allocating new homotopy vertex struct");
        return NULL;
    }
    // constant fields
    hv->x_s = x_s;
    hv->y_s = y_s;
    // list to be accessed and updated as required by the caller
    hv->hom_classes = hom_classes_list_new(max_hom_classes, less_than);

    return hv;
}


/************************************** hom_class ***************************************/

hom_class_t * hom_class_new(hom_vertex_t * endpoint_vertex, Complex Lval) {

    hom_class_t * hc = (hom_class_t *) malloc(sizeof(hom_class_t));
    if (hc == NULL) {
        DEBUG_ERROR("Malloc error allocating new homotopy class struct");
        return NULL;
    }
    // constant fields
    hc->Lval = Lval;
    hc->endpoint_vertex = endpoint_vertex;
    // to be updated by the caller
    hc->next = NULL; 
    hc->updated = true; // constructed in this run of A*
    hc->parent = NULL;
    hc->is_evaluated = false;
    hc->minheap_node = NULL;
    hc->backtrack = NULL; 
    hc->g_score = INFINITY;
    hc->f_score = INFINITY;

    return hc;
}

void hom_class_free(hom_class_t * hc) {
    if (hc == NULL) return;
    // backtrack is the only field potentially pointing to other memory to deallocate
    if (hc->backtrack) {
        free_backtrack(hc);
    }
    free(hc);
}

bool g_score_compare(hom_class_t * hc_1, hom_class_t * hc_2) {
    return (
        hc_1 != NULL &&
        hc_2 != NULL &&
        hc_1->g_score < hc_2->g_score
    );
}

void expand_backtrack(Params *params, hom_class_t * hom_class) {
    
    int edge_number     = 0;       // number of discrete edges
    float abs_len_cumul = 0.0f;    // cumulative length of path, scaled up
    float riem_cumul    = 0.0f;    // cumulative rieamnn sum for g_image buildup, scaled up
    int x_curr, y_curr;            // backtracking coordinates
    int x_next, y_next;         

    // First, find the length of the path by traversing from goal to start
    hom_class_t * curr_hom_class = hom_class;
    while (curr_hom_class != NULL) {
        edge_number++;
        curr_hom_class = curr_hom_class->parent;
    }

    // Check if backtrack was already expanded 
    if (hom_class->backtrack == NULL) {
        hom_class->backtrack = (Backtrack_Path *) malloc (sizeof(Backtrack_Path));
    } else {
        DEBUG_ERROR(
            "homotopy class has non-NULL backtrack, meaning it is already expanded: "
            "please free by calling free_backtrack before re-expanding to prevent memory leaks "
        );
        return;
    }

    Backtrack_Path * bt = hom_class->backtrack;

    // Set length for path struct
    bt->total_edges = edge_number;

    // Allocate memory for the points list
    bt->point_list = malloc(sizeof(float *) * edge_number);
    for (int i = 0; i < edge_number; i++) {
        bt->point_list[i] = malloc(sizeof(float) * 2); // Each point has two coordinates (x, y)
        if (!bt->point_list[i]) {
            fprintf(stderr, "Memory allocation failed for point_list[%d].\n", i);
            return;
        }
    }

    // Backtrack from goal to start and store the path in reverse
    curr_hom_class = hom_class;
    for (int i = edge_number - 1; i >= 0; i--) {

        // Extract coordinates from current vertex and parent if there is one
        x_curr = x_next = curr_hom_class->endpoint_vertex->x_s;
        y_curr = y_next = curr_hom_class->endpoint_vertex->y_s;
        if (curr_hom_class->parent != NULL) {
            x_next = curr_hom_class->parent->endpoint_vertex->x_s;
            y_next = curr_hom_class->parent->endpoint_vertex->y_s;
        }

        // Re-compute length and Riemann sum as they were originally computed, this time without the constants a,b
        abs_len_cumul  +=                 len(x_next, y_next, x_curr, y_curr, params);
        riem_cumul     += g_image_riemann_sum(x_next, y_next, x_curr, y_curr, params);

        // Record points in the array containing the path
        bt->point_list[i][0] = params->x_min + (x_curr * params->s); // Convert x_s back to x
        bt->point_list[i][1] = params->y_min + (y_curr * params->s); // Convert y_s back to y

        #ifdef ASTDBG
        printf("Backtracking path point[%d]: (%.2f, %.2f)\n", i, bt->point_list[i][0], bt->point_list[i][1]);
        #endif

        curr_hom_class = curr_hom_class->parent;    // iterate backwards through vertices on optimal path
    }

    // Set elements for the path
    bt->g_image_riemann_tot = riem_cumul;
    bt->absolute_length     = abs_len_cumul;

}

void write_path(hom_class_t * hom_class, Config * config, char * filepath){

    Backtrack_Path * bt = hom_class->backtrack;
    if (bt == NULL) return;
    const int str_size = 256;
    char * Lval_string = (char*) malloc(str_size);

    // Format the entry for the path in the output .json file
    snprintf(
        Lval_string, str_size, 
        output_path_key_format, 
        config->a, config->b, creal(hom_class->Lval), cimag(hom_class->Lval)
    );
    #ifdef AST_HC_DBG
    printf("Writing path to filepath %s with key: %s\n", filepath, Lval_string);
    #endif
    write_json(filepath, Lval_string, bt->point_list, bt->total_edges - 1, 2);
    free(Lval_string);
}

void free_backtrack(hom_class_t * hom_class) {

    Backtrack_Path * bt = hom_class->backtrack;

    // Verify that both path struct and internal point list are accessible 
    if (bt == NULL || bt->point_list == NULL || bt->total_edges == 0) {
        DEBUG_ERROR("free_point_array: opt_path = NULL OR opt_path->point_list = NULL OR opt_path->l = 0");
        return;
    }

    // Edge iteration through array
    for (int i = 0; i < bt->total_edges; i++) {
        // Verify that we are actually free'ing
        if (bt->point_list[i] == NULL) {
            DEBUG_ERROR("free_point_array: unexpected NULL value for opt_path->point_list[i]");
        }
        // Free the point
        free(bt->point_list[i]);
    }
    free(bt->point_list);

    // Finally, free the backtrack structure pointer itself
    free(bt); 
}


/************************************** hom_classes_list **************************************/

hom_classes_list_t *hom_classes_list_new(int capacity, bool (*less_than)(hom_class_t *, hom_class_t *)) {

    hom_classes_list_t *hom_classes_list = malloc(sizeof(hom_classes_list_t));
    if (hom_classes_list == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for hom_classes_list.\n");
        return NULL; // allocation failure
    }

    hom_classes_list->head = NULL;
    hom_classes_list->capacity = capacity;
    hom_classes_list->size = 0;
    hom_classes_list->less_than = less_than;

    return hom_classes_list;
}

void free_hom_classes_list(hom_classes_list_t *hom_classes_list) {
    if (hom_classes_list == NULL) return;
    hom_class_t *current = hom_classes_list->head;
    while (current != NULL) {
        hom_class_t *next = current->next;
        hom_class_free(current);
        current = next;
    }
    free(hom_classes_list);
}

hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol) {

    if (!hom_classes_list) return NULL;

    // List iteration logic
    hom_class_t * it = hom_classes_list->head;
    while (it != NULL) {
        // Check if iterator's Lval is within range of desired Lval
        if (cmplx_compare(Lval, it->Lval, abs_tol)) {
            return it;
        }
        it = it->next;
    }
    return NULL;  // If we have gone through the entire successfully, then it means it is not contained
}

bool insert_hom_class_sorted(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol, hom_class_t ** truncated) {

    /*** IMPORTANT:
     * This method does not handle the update of an already existing homotopy class,
     * but merely adds a new one corresponding to a new Lvalue if it qualifies.
     * To update an already-existing class in the list extract it with 
     * hom_class_get(hom_classes_list, hom_class->Lval, abs_tol)
    */

    // NULL checks
    if (!hom_classes_list || !hom_class) return false;

    // Redundant check
    if (hom_class_get(hom_classes_list, hom_class->Lval, abs_tol) != NULL) {
        return false;
    }

    // Ensure to_remove is set to default of NULL
    *(truncated) = NULL;
    
    hom_class_t **it = &(hom_classes_list->head);
    // note that we have NULL-checked both args of less_than
    while (*it != NULL && hom_classes_list->less_than(*it, hom_class)) {
        it = &(*it)->next; // Navigate to the correct insertion point.
    }
    // If we are already at capacity, and have reached null as candidate insertion, then didn't make it into the list 
    bool reached_capacity = (hom_classes_list->size == hom_classes_list->capacity);
    if (reached_capacity && *it == NULL) {
        return false;
    }
    // If we make it to here then we can insert
    hom_class->next = *it;
    *it = hom_class;
    hom_classes_list->size++;
    // If we had initially reached capacity, we need to delete the final element
    if (reached_capacity) {
        hom_class_t * prev = hom_class; // the pass of the previous false condition guarantees that curr != NULL
        hom_class_t * curr = prev->next;
        while (curr->next != NULL) {
            prev = curr;    // keep previous value
            curr = curr->next; // curr_next serves as iterator
        }
        // prev->next now points to element to truncate
        *(truncated) = prev->next;  // save the just-truncated element 
        prev->next = NULL;          // mark prev as closing element
        hom_classes_list->size--;   // decrease size after removing last element  
    }
    return true;
}

char* complex_list_to_string(hom_classes_list_t *list) {
    hom_class_t *current = list->head;

    size_t bufferSize = 256;  // Initial buffer size
    char *result = malloc(bufferSize);
    if (!result) return NULL; // Memory allocation failed

    char buffer[50];
    size_t currentPosition = 0;

    while (current != NULL) {
        // First write new "piece" corresponding to one complex number to the buffer
        int written = snprintf(buffer, sizeof(buffer), "\t(%.2f + %.2fi),", 
                               creal(current->Lval), cimag(current->Lval));
        // Check if the allocated buffersize would suffice, and reallocate twice the size of necessary
        if (currentPosition + written >= bufferSize) {
            bufferSize *= 2; // Increase buffer size
            result = realloc(result, bufferSize);
            if (!result) return NULL; // Memory allocation failed
        }
        strcpy(result + currentPosition, buffer);   // copy buffer into the result
        currentPosition += written; // advance the pointer to the current position

        current = current->next;
    }
    return result;
}

void unflag_homotopy_classes(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    while (current != NULL) {
        current->updated = false;
        current = current->next;
    }
}



/****************************** Unit testing area *******************************/

#ifdef HCSL_UT

#define CAPACITY 10

void print_complex_list(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    while (current != NULL) {
        printf("\t(%.2f + %.2fi),", creal(current->Lval), cimag(current->Lval));
        current = current->next;
    }
    printf("\n");
}

// Function to print the list
void print_list(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    printf("List (size %d): ", list->size);
    while (current != NULL) {
        printf("%g -> ", current->g_score);
        current = current->next;
    }
    printf("NULL\n\n");
}

void example_usage() {

    // hard-coded constants
    const double abs_tol = 0.1;
    const int num_homotopy_classes = 24;

    // Create the linked list
    hom_classes_list_t * list = hom_classes_list_new(CAPACITY, g_score_compare);

    /** NOTE: caller responsible for malloc'ing homotopy classes to then insert */
    hom_class_t * classes[num_homotopy_classes];
    for (int i = 0; i < num_homotopy_classes; i++) {
        hom_class_t *hc = malloc(sizeof(hom_class_t));
        if (hc == NULL) {
            fprintf(stderr, "Memory allocation failed for homotopy class.\n");
            return;
        }
        hc->Lval    = CMPLX(i, i);
        hc->g_score = (float)rand() / RAND_MAX * 10.0;
        hc->next    = NULL;
        classes[i] = hc;
    }

    // insertion procedure
    for (int i = 0; i < num_homotopy_classes; i++) {
        hom_class_t * truncated;
        bool inserted = insert_hom_class_sorted(list, classes[i], abs_tol, &truncated);
        if (inserted) {
            printf("Inserted class with g-score: %f\n", classes[i]->g_score);
            /** NOTE: caller responsible for managing deallocation of truncated element upon insertion */
            if (truncated) {
                printf("Truncated and free'd class with g-score: %f\n", truncated->g_score);
                free(truncated);
            }
        } else {
            printf("Failed to insert class with g-score: %f\n", classes[i]->g_score);
            /** NOTE: caller responsible for managing deallocation of failed-to-insert element */
            free(classes[i]);
        }
        print_list(list);
        // Allocate and free the string representing the list
        char * list_str = complex_list_to_string(list);
        printf("complex_list_to_string output: %s\n", list_str);
        free(list_str);
    }

    // now the list handles the free'ing of all remaining elements
    free_hom_classes_list(list);
}

int main() {
    example_usage();
    return 0;
}

#endif



