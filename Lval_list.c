
#include "Lval_list.h"

/* This linked list-like data structure handles all the operations necessary for processing vertex L-values */

struct Lval_node {
    Complex Lval;
    struct Lval_node * next;
};

struct Lval_list {
    
    Lval_node * head;
    Lval_node ** last;

};

Lval_list_t *Lval_list_new() {

    // Allocate memory for the open_set structure
    Lval_list_t *Lval_list = malloc(sizeof(Lval_list_t));
    if (Lval_list == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for Lval_list.\n");
        return NULL; // Return NULL to indicate failure
    }

    // Initialize the head of the list to NULL
    Lval_list->head = NULL;
    Lval_list->last = NULL;

    return Lval_list; // Return the pointer to the newly created open set
}

// Clean-up function
void free_Lval_list(Lval_list_t *Lval_list) {
    Lval_node *current = Lval_list->head;
    while (current != NULL) {
        Lval_node *next = current->next;
        free(current);
        current = next;
    }
    free(Lval_list);
}

/* Check whether an arbitrary Lval is already in the list up to some tolerance */ 
bool Lval_contains(Lval_list_t *Lval_list, Complex Lval, double abs_tol) {

    // Extract real and imaginary parts
    double Lval_re = creal(Lval);
    double Lval_im = cimag(Lval);

    // List iteration logic
    Lval_node **tracer = &(Lval_list->head);
    while (*tracer != NULL) {
        double tracer_Lval_re = creal((*tracer)->Lval);
        double tracer_Lval_im = cimag((*tracer)->Lval);
        // If we are within abs_tol for real AND imaginary part, then we consider it contained
        if (fabs(tracer_Lval_re - Lval_re) < abs_tol &&
            fabs(tracer_Lval_im - Lval_im) < abs_tol) {
                return true;
        }
        // Slide tracer forward until we encounter NULL, updating the last pointer each time
        tracer = &(*tracer)->next;
    }
    // Update last value of list with tracer pointer to value to be initialized
    Lval_list->last = (tracer);

    // If we have gone through the entire successfully, then it means it is not contained 
    return false;
}

/* Aim: return true iff all the L-values in other_list are contained in Lval_list */
bool Lval_list_contains(Lval_list_t * Lval_list, Lval_list_t * other_list, double abs_tol) {

    Lval_node * iterator = other_list->head;
    while (iterator != NULL) {
        // As soon as we can find some element in the secoond list contained in the first, false
        if (!Lval_contains(Lval_list, iterator->Lval, abs_tol))  {
            return false;
        }
        iterator = iterator->next;
    }
    return true;
}

/* Insert an L_value only if distinct */
void insert_Lval(Lval_list_t *Lval_list, Complex Lval, double abs_tol) {

    // If this call returns false, then Lval_list->last is updated with the pointer of the node to initialize
    if (Lval_contains(Lval_list, Lval, abs_tol)) {
        return;
    }

    // Work on initializing the last node, which is pointed toward by Lval_list->last
    (*Lval_list->last) = (Lval_node *) malloc(sizeof(Lval_node));
    (*Lval_list->last)->Lval = Lval;
    (*Lval_list->last)->next = NULL;
}

/* This function handles the situation where given an other_list of Lvalues for another state, and L_offset between the states,
 * we update the list of our current state to incorporate those new Lvalues */
void merge_Lvalues(Lval_list_t *Lval_list, Lval_list_t * other_list, Complex L_offset, double abs_tol) {

    Lval_node * iterator = other_list->head;
    while (iterator != NULL) {
        // Attempt insertion into our list for each value in other_list
        insert_Lval(Lval_list, (iterator->Lval + L_offset), abs_tol);
        iterator = iterator->next;
    }
}

/*
params->x_min + (x_curr * params->s); // Convert x_s back to x
params->y_min + (y_curr * params->s); // Convert y_s back to y
*/


/* Unit testing area */

void print_complex_list(Lval_list_t *list) {
    Lval_node *current = list->head;
    while (current != NULL) {
        printf("\t(%.2f + %.2fi),", creal(current->Lval), cimag(current->Lval));
        current = current->next;
    }
}

char* complex_list_to_string(Lval_list_t *list) {
    Lval_node *current = list->head;

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


void test_insertion() {
    Lval_list_t *list = Lval_list_new();
    if (list == NULL) {
        printf("Failed to create list.\n");
        return;
    }

    // Insert first complex number
    Complex z1 = CMPLX(1.0, 1.0);
    insert_Lval(list, z1, 0.1);
    printf("After inserting z1 (1.0 + 1.0i):\n");
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
    Lval_list_t *list2 = Lval_list_new();
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
    printf("Merging L-values with offset of (1.0 + 1.0i)\n");
    merge_Lvalues(list, list2, CMPLX(1.0, 1.0), 0.1);
    print_complex_list(list);

    // Clean up
    printf("End of tests, cleaning up\n");
    free_Lval_list(list);
    free_Lval_list(list2);
}

#ifdef LVAL_UT

int main() {
    test_insertion();
    return 0;
}

#endif
