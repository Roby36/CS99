
#include "hom_classes.h"

/** This linked list-like data structure handles 
 * the operations necessary for processing L-values associated
 * with each vertex  
*/

/***** hom_classes_list_new ******* 
 * 
 * This function allocates a linked list data structure containing homotopy classes,
 * and returns its corresponding pointer.
 * 
 * NOTE: Caller responsible for calling free_hom_classes_list on the pointer returned 
 *       to free the memory malloc'd by this function 
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

/****** free_hom_classes_list ******
 * 
 * This is the corresponding clean-up routine for hom_classes_list_new,
 * called when we finished using a hom_classes_list_t struct.
 * 
 * NOTE: This function free's also the internal hom_class objects, 
 *        which were NOT allocated by the module itself but rather outside of it
 */
void free_hom_classes_list(hom_classes_list_t *hom_classes_list) {
    hom_class_t *current = hom_classes_list->head;
    while (current != NULL) {
        hom_class_t *next = current->next;
        free(current);
        current = next;
    }
    free(hom_classes_list);
}

/****** hom_class_get *******
 * 
 * Inputs:
 *  - hom_classes_list_t struct pointer within which we want to search a homotopy class with some Lvalue descriptor
 *  - Complex valued L-value we want to search for in the linked list of homotopy classes
 *  - double absolute tolerance value within which we are expecting to find the L-value
 * 
 * Output:
 *  - If the homotopy class with the desired L-value is present, return directly the pointer to it
 *  - If no homotopy class with the desired L-value is present, return NULL 
 */
hom_class_t * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol) {

    // NULL if we pass a NULL list
    if (!hom_classes_list) return NULL;

    // List iteration logic
    hom_class_t * it = hom_classes_list->head;
    while (it != NULL) {
        // Check if iterator's Lval is within range of desired Lval
        if (cmplx_compare(Lval, it->Lval, abs_tol)) {
            return it;
        }
        // Slide tracer forward until we encounter NULL, updating the last pointer each time
        it = it->next;
    }
    return NULL;   // If we have gone through the entire successfully, then it means it is not contained
}

/******* insert_hom_class ********
 * 
 * Inputs:
 *  - hom_classes_list_t struct pointer within which we desire to insert a new homotopy class
 *  - pointer to already-allocated homotopy class struct that we want to insert
 *  - double absolute tolerance value to determine if the homotopy class is already in the list
 * 
 * Outputs:
 *  - false if some homotopy class with the same L-value is already present in the list,
 *    and homotopy class therefore is not inserted, even if it may possibly have different feaures, 
 *    such as f_score, g_score, backtrack etc.
 *  - true if no homotopy class with the same L-value is present in the list,
 *    and homotopy class is therefore successfully appended to the linked list
 * 
 * NOTE: This function requires an already-allocated homtopy class struct, and the caller is responsible for that.
 *       The linked list is merely keyed by the L-values of homtopy classes. It is thus a perfect system to store 
 *       a small representative sample of distinct homotopy classes, but not suited for any type of exhaustive list.
 *       
 *      It therefore should not be confused with the open set, which is the data structure actually storing all the possible 
 *      homotopy classes structs in A*, many of which with the same L-values.   
 */
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


/***** complex_list_to_string *******
 * 
 * This function essentially stringifies a homtopy class list struct, 
 * allocating dynamically a buffer and returning a char * to it
 * 
 * NOTE: Caller responsibke for freeing returned char *
 */
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

/************* unflag_homotopy_classes *************
 * 
 * This function unflags all the homotopy classes in a given list
 * to inform A* that they have not yet been updated by the current iteration
*/
void unflag_homotopy_classes(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    while (current != NULL) {
        current->updated = false;
        current = current->next;
    }
}

/******* Unit testing area ********/

#ifdef LVAL_UT

void print_complex_list(hom_classes_list_t *list) {
    hom_class_t *current = list->head;
    while (current != NULL) {
        printf("\t(%.2f + %.2fi),", creal(current->Lval), cimag(current->Lval));
        current = current->next;
    }
    printf("\n");
}

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



