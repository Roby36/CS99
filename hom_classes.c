
#include "hom_classes.h"

/* This linked list-like data structure handles all the operations necessary for processing vertex L-values */

/*
hom_class_t * hom_class_new(Complex Lval) {

    hom_class_t * hom_class_new = (hom_class_t *) malloc(sizeof(hom_class_t));
    if (hom_class_new == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for hom_class\n");
        return NULL; // Return NULL to indicate failure
    }

    hom_class_new->Lval = Lval;
    hom_class_new->next = NULL;
    // Initialize graph properties as they would be initialized for new vertex
    hom_class_new->parent = NULL;
    // hom_class_new->parent_hom_class = NULL;
    hom_class_new->g_score = INFINITY;
    hom_class_new->f_score = INFINITY;

    return hom_class_new;
}
*/

hom_classes_list_t *hom_classes_list_new() {

    // Allocate memory for the open_set structure
    hom_classes_list_t *hom_classes_list = malloc(sizeof(hom_classes_list_t));
    if (hom_classes_list == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for hom_classes_list.\n");
        return NULL; // Return NULL to indicate failure
    }

    // Initialize the head of the list to NULL
    hom_classes_list->head = NULL;
    // Hold the address to the head of the list for the first insertion
    hom_classes_list->last_ptr = &(hom_classes_list->head);

    return hom_classes_list; // Return the pointer to the newly created open set
}

// Clean-up function 
/** CAREFUL: This function free's also the internal hom_class objects, which were not allocated by the module itself */
void free_hom_classes_list(hom_classes_list_t *hom_classes_list) {
    hom_class_t *current = hom_classes_list->head;
    while (current != NULL) {
        hom_class_t *next = current->next;
        free(current);
        current = next;
    }
    free(hom_classes_list);
}

/* Extract from the list the details for a homotopic class corresponding to some Lval */ 
hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol) {

    // NULL if we pass a NULL list
    if (!hom_classes_list) return NULL;

    // Extract real and imaginary parts
    double Lval_re = creal(Lval);
    double Lval_im = cimag(Lval);

    // List iteration logic
    hom_class_t * it = hom_classes_list->head;
    while (it != NULL) {
        double tracer_Lval_re = creal((it)->Lval);
        double tracer_Lval_im = cimag((it)->Lval);
        // If we are within abs_tol for real AND imaginary part, then we consider it and return it
        if (fabs(tracer_Lval_re - Lval_re) < abs_tol &&
            fabs(tracer_Lval_im - Lval_im) < abs_tol) {
                return (it);
        }
        // Slide tracer forward until we encounter NULL, updating the last pointer each time
        it = it->next;
    }
    return NULL;   // If we have gone through the entire successfully, then it means it is not contained
}

/* Insert an L_value only if distinct, and return boolean  */
bool insert_hom_class(hom_classes_list_t *hom_classes_list, hom_class_t * hom_class, double abs_tol) {

    // Check NULL inputs
    if (!hom_classes_list || !hom_class) return false;

    // First check if already in the list
    if (hom_class_get(hom_classes_list, hom_class->Lval, abs_tol) != NULL) {
        return false;
    }

    // hom_class_t * hom_class = hom_class_new(Lval);
    *(hom_classes_list->last_ptr) = hom_class;
    // Set pointer to the next value to fill
    hom_classes_list->last_ptr = &(hom_class->next);

    return true;
}

/* Return hom_class struct with minimum f_score */
/*
hom_class_t * hom_class_get_min_f(hom_classes_list_t *hom_classes_list) {
    // Note that this returns NULL if we have an empty list
    float min_f = INFINITY;
    hom_class_t * min_class = NULL;

    // List iteration logic
    hom_class_t * it = hom_classes_list->head;    // as usual, we assume no NULL pointer is passed
    while (it != NULL) {
        float curr_f = it->f_score;
        if (curr_f < min_f) {
            min_f     = curr_f;
            min_class = it;
        }
        it = it->next;
    }
    return min_class;
}
*/





/* Unit testing area */

void print_complex_list(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    while (current != NULL) {
        printf("\t(%.2f + %.2fi),", creal(current->Lval), cimag(current->Lval));
        current = current->next;
    }
    printf("\n");
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

#ifdef LVAL_UT

void test_insertion() {
    hom_classes_list_t *list = hom_classes_list_new();
    if (list == NULL) {
        printf("Failed to create list.\n");
        return;
    }

    // Insert first complex number
    Complex z1 = CMPLX(1.0, 1.0);
    insert_Lval(list, z1, 0.1);
    printf("After inserting z1 (1.0 + 1.0i):\n");
    print_complex_list(list);

    // Try inserting 0 + 0i (edge case) several times
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    insert_Lval(list, CMPLX(0.0, 0.0), 0.1); 
    printf("After inserting (0.0 + 0.0i) repeatedly:\n");
    print_complex_list(list);

    // Insert a close value to test the tolerance
    Complex z2 = CMPLX(1.05, 1.0);  // Within tolerance of z1
    insert_Lval(list, z2, 0.1);
    printf("After inserting z2 (1.05 + 1.0i), within tolerance:\n");
    print_complex_list(list);

    // Insert clearly distinct values
    Complex z3 = CMPLX(2.0, 2.0);
    insert_Lval(list, z3, 0.1);
    printf("After inserting z3 (2.0 + 2.0i):\n");
    print_complex_list(list);

    // Insert another value far from existing ones
    Complex z4 = CMPLX(5.0, 5.0);
    insert_Lval(list, z4, 0.1);
    printf("After inserting z4 (5.0 + 5.0i):\n");
    print_complex_list(list);

    // Finally try another value within tolerance
    Complex z5 = CMPLX(1.95, 2.05);
    insert_Lval(list, z5, 0.1);
    printf("After inserting z5 (1.95 + 2.05i):\n");
    print_complex_list(list);

    // Creating second list, adding some values
    hom_classes_list_t *list2 = hom_classes_list_new();
    insert_Lval(list2, CMPLX(0.0, 0.0), 0.1);
    printf("Inserting z6 (0.0 + 0.0i) into list2\n");
    insert_Lval(list2, CMPLX(2.0, 3.0), 0.1);
    printf("Inserting z6 (2.0 + 3.0i) into list2\n");
    insert_Lval(list2, CMPLX(4.0, 4.0), 0.1);
    printf("Inserting z6 (4.0 + 4.0i) into list2\n");
    insert_Lval(list2, CMPLX(3.0, 4.0), 0.1);
    printf("Inserting z6 (3.0 + 4.0i) into list2\n");
    print_complex_list(list2);

    // Now attempt to merge lists with an offset
    /*
    printf("Merging L-values with offset of (1.0 + 1.0i)\n");
    merge_Lvalues(list, list2, CMPLX(1.0, 1.0), 0.1);
    print_complex_list(list);
    */

    // Clean up
    printf("End of tests, cleaning up\n");
    free_hom_classes_list(list);
    free_hom_classes_list(list2);
}

int main() {
    test_insertion();
    return 0;
}

#endif



