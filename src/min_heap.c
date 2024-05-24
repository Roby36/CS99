
#include "open_set.h"

/** NOTE: 
 * Because we want to reuse this data structure for both the "standard" A* algorithm where we enqueue vertices,
 * and the extended version of the algorithm where we actually enqueue homotopy class objects, 
 * we are going to implement "type punning" to allow the data structure to hold a generic type.
 * This will also force further clarity and abstraction in explicitly defining which elements 
 * from vertices / homotopy classes the data structure actually needs to read and write to 
*/

/******* attributes_accessor *******
 * 
 * This struct holds the pointers to the functions required to both set and get
 * the required elements from the "generic" data type. In this way, these pointers are set
 * at construction time and then determine on what data type the data structure ends up operating,
 * defining a sort of common interface.
*/
typedef struct {
    float (* get_f_score) (void *);
    void (* set_minheap_node) (void *, minheap_node *);

} attributes_accessor;

/* Now define main min-heap specific structures */
typedef struct minheap_node {
    void * generic;     // Pointer to the corresponding generic
    float key;          // heap node key, based on vertex->f_score 
    int index;          // index within the array

} minheap_node;

typedef struct open_set {
    minheap_node **nodes;  // Array of pointers to heap nodes
    int size;              // Current size of the heap
    int capacity;          // Maximum capacity of the heap

    attributes_accessor accessor;   // pointers to encapsulated getters and setters
} open_set_t;

/***** static helper functions *****/

void swap(minheap_node **a, minheap_node **b) {
    minheap_node *temp = *a;    // copy memory address of node a into pointer temp
    *a = *b;                    // Set array entry where memory address for a is stored, to memory address of b
    *b = temp;                  // Set array entry where memory address of b is stored, to memory address of a

    // Update indices in the nodes after swapping pointers.
    int tempIndex = (*a)->index;
    (*a)->index = (*b)->index;
    (*b)->index = tempIndex;
}

void bubble_up(open_set_t *set, int index) {
    // While the iterator node, initialized at index, is smaller than its parent, swap them
    while (index > 1 && set->nodes[index]->key < set->nodes[index / 2]->key) {
        swap(&set->nodes[index], &set->nodes[index / 2]);
        index /= 2;
    }
}

void min_heapify(open_set_t *set, int i) {
    // Extract array indeces corresponding to left and right elements
    int smallest = i;
    int left = 2 * i;
    int right = 2 * i + 1;

    // Check that there are indeed left or right children before comparing keys 
    if (left <= set->size && set->nodes[left]->key < set->nodes[smallest]->key) {
        smallest = left;
    }
    if (right <= set->size && set->nodes[right]->key < set->nodes[smallest]->key) {
        smallest = right;
    }
    // If left or right children are smaller, swap them with parent and restore heap property downwards 
    if (smallest != i) {
        swap(&set->nodes[i], &set->nodes[smallest]);
        min_heapify(set, smallest);
    }
}

/********** Public functions **************/

/****** open_set_new ********
 * 
 * Inputs: 
 *  These are the data-type-specific accessor functions,
 *  determining at construction the data type on which the minimum heap will operate:
 *   - get_f_score: returns the f_score of a given vertex / hom_class
 *   - set_minheap_node: sets the minheap_node pointer for a given vertex / hom_class
 * 
 * Output:
 *  Pointer to min_heap data structure if allocation successful,
 *  NULL if initialization failed
 * 
 * NOTE: Caller responsible for calling corresponding clean-up routine free_open_set
 */
open_set_t *open_set_new(
    float (* get_f_score) (void *),
    void  (* set_minheap_node) (void *, minheap_node *)) 
{
    open_set_t *set = malloc(sizeof(open_set_t));
    if (!set) return NULL;
    set->capacity = 1024;  // Initial capacity, will be increased if needed
    set->size = 0;
    // Allocate space for node pointers, only up to initial capacity
    set->nodes = malloc(sizeof(minheap_node *) * set->capacity);
    if (!set->nodes) {
        free(set);
        return NULL;
    }
    // Set accessor function pointers controlling behaviour on all generic objects across this data structure 
    set->accessor.get_f_score      = get_f_score;
    set->accessor.set_minheap_node = set_minheap_node;
    return set;
}

/******* free_open_set ********
 * 
 * Corresponding clean-up routine for open_set_new,
 * free'ing all the memory allocated for the opaque data types,
 * including minimum heap nodes and the minimum heap struct itself.
 * 
 * NOTE: As intended, this function does not affect the void * data elements
 *       entered in the heap, and thus the caller is fully responsible for those
 */
void free_open_set(open_set_t *set) {
    // free each node in the array
    for (int i = 1; i <= set->size; i++) {
        free(set->nodes[i]);
    }
    // free pointer to array of node pointers
    free(set->nodes);
    // free minimum heap itself
    free(set);
}

/********* insert_sorted ********
 * 
 * This is the function to insert an element of any type in the heap
 * in O(logn), whilst crucially preserving the min-heap property
 * 
 * Inputs:
 *  - pointer to minheap where we want to insert the element
 *  - void * to the generic data-type we want to insert (i.e. vertex_t, hom_class_t, ...)
 * 
 * Output:
 *  The function does not return anything, but arranges the element within the 
 *  minimum heap, handling resizes and restoring the min heap property via 
 *  the bubble_up static subroutine 
 */
void insert_sorted(open_set_t *set, void * data) {
    // First check if we need a dynamic resize
    if (set->size >= set->capacity - 1) {
        // Resize array if capacity is exceeded, through dynamic reallocation
        set->capacity *= 2;
        set->nodes = realloc(set->nodes, sizeof(minheap_node *) * set->capacity);
    }
    minheap_node * node = malloc(sizeof(minheap_node));
    // Allocate the wrapper for the arbitrary void*
    node->generic = data;  
    /* Now use the special accessor functions to handle the generic data type */
    node->key = set->accessor.get_f_score(node->generic); // set the key to the f_score of the input vertex / hom_class
    set->accessor.set_minheap_node(node->generic, node);  // flag the input vertex / hom_class as part of the open set data structure
    node->index = set->size + 1;        // Assign index as it is placed in the array
    set->nodes[++set->size] = node;     // store newly generated node in array and increase size 
    bubble_up(set, set->size);          // restore the heap property
}

/******** dequeue_min ********
 * 
 * This function dequeues the min from the root of the minimum heap, 
 * and restores the min heap property, all in O(logn) time
 * 
 * Inputs:
 *  - pointer to minheap from which we want to dequeue the minimum element
 * 
 * Outputs:
 *  - The original generic void * data type, as was originally inserted
 * 
 * NOTE: The function already flags the vertex/homotopy_class internal 
 *       minheap_node element to NULL via accessor.set_minheap_node, 
 *       thereby flagging that the element is not part of the open set anymore
 *       The function does not affect the void * entered in insert_sorted in any way
 */
void * dequeue_min(open_set_t *set) {
    if (open_set_is_empty(set)) return NULL;

    minheap_node *min_node = set->nodes[1];     // extract the minimum 
    void * min_data = min_node->generic;        // extract the arbitrary pointer 
    
    set->nodes[1] = set->nodes[set->size];  // Move the last node to the root position.
    set->nodes[1]->index = 1;               // Update the index of the new root node.
    set->size--;                            // Decrease the size of the heap.

    min_heapify(set, 1);                    // restore heap property from the root
    // mark the vertex / hom_class in min_data as outside of the heap
    set->accessor.set_minheap_node(min_data, NULL); 
    free(min_node);             // free the memory of the node truncated out of the list

    return min_data;            // return void* as was originally inserted
}

/****** decrease_key **********
 * 
 * This is a crucial function for the module, especially in the context of the A* algorithm,
 * where a node somewhere in the heap experiences an update in key which may potentially 
 * affect the structure of the minimum heap, and thus the nodes that will be successively dequeued.
 * 
 * This function ensures that whenever the key of an arbitrary node is updated to some new_key value,
 * the heap is updated in O(logn) time to account for this change, via the bubble_up subroutine.
 * This would not normally be possible, since from the outer scope we would not have access to the 
 * internal index of the node, which is required to rearrange the heap. 
 * 
 * However, we work around this by storing and maintaining up to date, for each minheap_node in the heap,
 * the index of this node within the array confguration of the heap. This allows to then access
 * from the vertex/hom_class object the corresponding minheap_node, and from that the corresponding index,
 * allowing the reconfiguration to smoothly take place through the bubble_up subroutine in O(logn).
 */
void decrease_key(open_set_t *set, minheap_node *node, float new_key) {
    if (new_key > node->key) {
        printf("New key is greater than current key.\n");
        return;
    }
    node->key = new_key;    // update node key
    bubble_up(set, node->index);  // restore heap property from below
}


/* Simple getters */

bool open_set_is_empty(open_set_t *set) {
    return set->size == 0;
}

int open_set_size(open_set_t * set) {
    return (set->size);
}


/* Define down here the underlying getters and setters for the supported types (could also be defined outside of the file) */

/** NOTE: 
 * These methods are disabled because we undefined these elements for vertex structs 
 * since they are currently held by the homotopy classes strcuts 
float vertex_get_f_score (void * vertex) {
    return ((vertex_t *) vertex)->f_score;
}
void vertex_set_minheap_node (void * vertex, minheap_node * minheap_node) {
    ((vertex_t *) vertex)->minheap_node = minheap_node;
}
*/

float hom_class_get_f_score (void * hom_class) {
    return ((hom_class_t *) hom_class)->f_score;
}
void hom_class_set_minheap_node (void * hom_class, minheap_node * minheap_node) {
    ((hom_class_t *) hom_class)->minheap_node = minheap_node;
}


/**************** Unit testing area ***************/

#ifdef MH_DBG

void print_open_set(open_set_t *set) {
    printf("Open Set contains %d elements:\n", set->size);
    for (int i = 1; i <= set->size; i++) {
        vertex_t *v = set->nodes[i]->generic;
        printf("Index %d: Vertex(%d, %d) [g_score: %.2f, f_score: %.2f, key: %.2f]\n", 
            i, v->x_s, v->y_s, v->g_score, v->f_score, set->nodes[i]->key);
    }
}

void open_set_testing() {
    // Creating test vertices
    int num_vertices = 15;

    vertex_t vertices[num_vertices];
    for (int i = 0; i < num_vertices; i++) {
        vertices[i].x_s = i;
        vertices[i].y_s = i;
        vertices[i].g_score = i * 2.0;
        vertices[i].f_score = i * 2.5;
        vertices[i].is_evaluated = false;
        vertices[i].parent = NULL;
        vertices[i].minheap_node = NULL;
    }

    // Create the open set
    open_set_t *set = open_set_new(
        vertex_get_f_score,
        vertex_set_minheap_node
    );

    // Insert vertices into the open set and print the state
    printf("Inserting elements into the open set:\n");
    for (int i = num_vertices; i > 0; i--) {
        insert_sorted(set, &vertices[i-1]);
        print_open_set(set);
    }

    // Testing decrease key
    int vertex_number = num_vertices - 3;
    printf("\nDecreasing key for vertex %d with %f f_score:\n", vertex_number, vertices[vertex_number].f_score);
    vertices[vertex_number].f_score = 0.1; // Update the f_score
    decrease_key(set, vertices[vertex_number].minheap_node, vertices[vertex_number].f_score);
    print_open_set(set);

    // Dequeue elements and print
    printf("\nDequeuing elements:\n");
    while (!open_set_is_empty(set)) {
        vertex_t *min_vertex = dequeue_min(set);
        printf("Dequeued Vertex(%d, %d) with final f_score: %.2f\n", min_vertex->x_s, min_vertex->y_s, min_vertex->f_score);
        print_open_set(set);
    }

    // Free the open set
    free_open_set(set);
}

int main() {
    open_set_testing();
    return 0;
}

#endif



