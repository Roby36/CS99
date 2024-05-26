
#include "hom_classes.h"

/** This linked list-like data structure handles 
 * the operations necessary for processing L-values associated
 * with each vertex  
*/

bool g_score_compare(hom_class_t * hc_1, hom_class_t * hc_2) {
    return (
        hc_1 != NULL &&
        hc_2 != NULL &&
        hc_1->g_score < hc_2->g_score
    );
}

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
    hom_classes_list->last_ptr = &(hom_classes_list->head);

    return hom_classes_list;
}

void free_hom_classes_list(hom_classes_list_t *hom_classes_list) {
    hom_class_t *current = hom_classes_list->head;
    while (current != NULL) {
        hom_class_t *next = current->next;
        free(current);
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
    // Proceed to potential insertion for a new homotopy class, with different L-val
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

bool insert_hom_class(hom_classes_list_t *hom_classes_list, hom_class_t *hom_class, double abs_tol) {

    if (!hom_classes_list || !hom_class) return false; // Check for null pointers.

    if (hom_class_get(hom_classes_list, hom_class->Lval, abs_tol) != NULL) {
        return false; // Prevent insertion if duplicate is found.
    }

    hom_class_t **it = &(hom_classes_list->head);
    while (*it != NULL && hom_classes_list->less_than(*it, hom_class)) {
        it = &(*it)->next; // Navigate to the correct insertion point.
    }

    // Perform the insertion only if the list is not at capacity, regardless of f_scores
    if (hom_classes_list->size < hom_classes_list->capacity) {
        hom_class->next = *it;
        *it = hom_class;
        hom_classes_list->size++;
        return true;
    }

    return false; // Return false if the list is at capacity and no insertion was made.
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

/******* Unit testing area ********/

#ifdef LVAL_UT

#define CAPACITY 10

void print_complex_list(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    while (current != NULL) {
        printf("\t(%.2f + %.2fi),", creal(current->Lval), cimag(current->Lval));
        current = current->next;
    }
    printf("\n");
}
// Function to create a new homotopy class
hom_class_t *new_hom_class(double complex Lval, float g_score) {
    hom_class_t *hc = malloc(sizeof(hom_class_t));
    if (hc == NULL) {
        fprintf(stderr, "Memory allocation failed for homotopy class.\n");
        return NULL;
    }
    hc->Lval = Lval;
    hc->g_score = g_score;
    hc->next = NULL;
    return hc;
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

// Test insertion function
void test_insertion() {
    const double abs_tol = 0.1;
    const int num_homotopy_classes = 24;
    // Create the linked list
    hom_classes_list_t list = {NULL, CAPACITY, 0, g_score_compare};

    // Create homotopy classes with varying f-scores and complex L-values
    hom_class_t *classes[num_homotopy_classes];
    for (int i = 0; i < num_homotopy_classes; i++) {
        classes[i] = new_hom_class(CMPLX(i, i), (float)rand() / RAND_MAX * 10.0);
    }

    // Insert classes into the list and print the list after each insertion
    for (int i = 0; i < num_homotopy_classes; i++) {
        if (insert_hom_class_sorted(&list, classes[i], abs_tol)) {
            printf("Inserted class with g-score: %f\n", classes[i]->g_score);
        } else {
            printf("Failed to insert class with g-score: %f\n", classes[i]->g_score);
        }
        print_list(&list);
    }
}

int main() {
    test_insertion();
    return 0;
}

#endif



