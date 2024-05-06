
#include "open_set.h"

typedef struct minheap_node {
    vertex_t *vertex;   // Pointer to the corresponding vertex
    float key;          // heap node key, based on vertex->f_score 
    int index;          // index within the array

} minheap_node;

typedef struct open_set {
    minheap_node **nodes;  // Array of pointers to heap nodes
    int size;              // Current size of the heap
    int capacity;          // Maximum capacity of the heap

} open_set_t;

open_set_t *open_set_new() {
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
    return set;
}

bool open_set_is_empty(open_set_t *set) {
    return set->size == 0;
}

/* Swap two nodes in the node array */
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

void insert_sorted(open_set_t *set, vertex_t *new_vertex) {
    if (set->size >= set->capacity - 1) {
        // Resize array if capacity is exceeded, through dynamic reallocation
        set->capacity *= 2;
        set->nodes = realloc(set->nodes, sizeof(minheap_node *) * set->capacity);
    }

    minheap_node *node = malloc(sizeof(minheap_node));
    node->vertex = new_vertex;
    node->key = new_vertex->f_score; 
    new_vertex->minheap_node = node;    // attach to the vertex the corresponding min_heap node
    node->index = set->size + 1;        // Assign index as it is placed in the array

    set->nodes[++set->size] = node;     // store newly generated node in array and increase size 
    bubble_up(set, set->size);          // restore the heap property
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

vertex_t *dequeue_min(open_set_t *set) {
    if (open_set_is_empty(set)) return NULL;

    minheap_node *min_node = set->nodes[1];     // extract the minimum 
    vertex_t *min_vertex = min_node->vertex;
    
    set->nodes[1] = set->nodes[set->size];  // Move the last node to the root position.
    set->nodes[1]->index = 1;               // Update the index of the new root node.
    set->size--;                            // Decrease the size of the heap.

    min_heapify(set, 1);                    // restore heap property from the root
    min_vertex->minheap_node = NULL;        // mark the vertex as outside of the heap
    free(min_node);                         // free the memory of the node truncated out of the list

    return min_vertex;
}

void decrease_key(open_set_t *set, minheap_node *node, float new_key) {
    if (new_key > node->key) {
        printf("New key is greater than current key.\n");
        return;
    }
    node->key = new_key;    // update node key
    bubble_up(set, node->index);  // restore heap property from below
}

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


/* TESTING section */
#ifdef MH_DBG

void print_open_set(open_set_t *set) {
    printf("Open Set contains %d elements:\n", set->size);
    for (int i = 1; i <= set->size; i++) {
        vertex_t *v = set->nodes[i]->vertex;
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
    open_set_t *set = open_set_new();

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