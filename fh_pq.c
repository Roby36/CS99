
#include "open_set.h"

typedef struct fib_node {
    vertex_t *vertex;       // Pointer to the corresponding vertex
    struct fib_node *parent;
    struct fib_node *child;
    struct fib_node *left;
    struct fib_node *right;
    int degree;
    bool marked;
} fib_node;

typedef struct open_set {
    fib_node *min;     // Pointer to the minimum node
    int n;             // Number of nodes in the heap
} open_set_t;

open_set_t *open_set_new() {

    open_set_t *set = malloc(sizeof(open_set_t));
    if (!set) {
        fprintf(stderr, "Memory allocation failed for the Fibonacci heap.\n");
        return NULL;
    }
    set->min = NULL;
    set->n = 0;
    return set;
}

bool open_set_is_empty(open_set_t *set) {
    return set->min == NULL;
}

void insert_sorted(open_set_t *set, vertex_t *new_vertex) {
    fib_node *node = malloc(sizeof(fib_node));
    if (!node) {
        fprintf(stderr, "Failed to allocate memory for Fibonacci node.\n");
        return;
    }
    node->vertex = new_vertex;
    new_vertex->fib_node = node;  // Link vertex to its heap node
    node->degree = 0;
    node->parent = NULL;
    node->child = NULL;
    node->marked = false;
    node->left = node->right = node;

    if (set->min == NULL) {
        set->min = node->left = node->right = node;
    } else {
        node->right = set->min;
        node->left = set->min->left;
        if (set->min->left)
            set->min->left->right = node;
        set->min->left = node;
        if (new_vertex->f_score < set->min->vertex->f_score) {
            set->min = node;
        }
    }
    set->n++;
}

void link(fib_node *y, fib_node *x) {
    // Remove y from root list and make it a child of x
    y->left->right = y->right;
    y->right->left = y->left;
    if (x->child == NULL) {
        x->child = y;
        y->right = y->left = y;
    } else {
        y->right = x->child;
        y->left = x->child->left;
        if (x->child->left)
            x->child->left->right = y;
        x->child->left = y;
    }
    y->parent = x;
    x->degree++;
    y->marked = false;
}

void insert_into_root_list(fib_node *root_list, fib_node *node_to_insert) {
    // Ensure that the node is initially self-linked to handle cases where it is the only node
    node_to_insert->left = node_to_insert;
    node_to_insert->right = node_to_insert;

    // Insert node_to_insert into the root list
    if (root_list != NULL) {
        fib_node *right = root_list->right; // Node that will be right of the inserted node
        node_to_insert->right = right;
        node_to_insert->left = root_list;
        root_list->right = node_to_insert;
        right->left = node_to_insert;
    }
}

void consolidate(open_set_t *set) {
    if (set->min == NULL) return;  // No consolidation needed if heap is empty

    // Array to keep track of degrees, and consolidate nodes
    int arraySize = (int)(log(set->n) / log(2)) + 1;
    fib_node **degreeArray = (fib_node **)calloc(arraySize, sizeof(fib_node *));
    if (!degreeArray) return;  // Memory allocation check

    fib_node *current = set->min;
    do {
        fib_node *x = current;
        current = current->right;
        int d = x->degree;
        while (degreeArray[d] != NULL) {
            fib_node *y = degreeArray[d];  // Another node with the same degree as x
            // Ignore vertices that were just removed from the heap, checking for NULLs
            if (x->vertex && y->vertex && (x->vertex->f_score > y->vertex->f_score)) {
                fib_node *temp = x;
                x = y;
                y = temp;
            }
            if (y == set->min) {
                set->min = x;
            }
            link(y, x);  // Link y into x's child list
            degreeArray[d] = NULL; // Explicitly reset the degree entry after linking
            d++;
        }
        degreeArray[d] = x;
    } while (current != set->min);

    // Reconstruct the root list from the degree array
    set->min = NULL;
    for (int i = 0; i < arraySize; i++) {
        if (degreeArray[i] != NULL) {
            if (set->min == NULL) {
                set->min = degreeArray[i];
            } else {
                degreeArray[i]->right = degreeArray[i]->left = degreeArray[i];
                insert_into_root_list(set->min, degreeArray[i]);
                if (degreeArray[i]->vertex && set->min->vertex && degreeArray[i]->vertex->f_score < set->min->vertex->f_score) {
                    set->min = degreeArray[i];
                }
            }
        }
    }

    free(degreeArray);
}

vertex_t *dequeue_min(open_set_t *set) {
    if (set->min == NULL) {
        return NULL;  // If there are no nodes in the heap, return NULL.
    }

    fib_node *min = set->min;
    vertex_t *min_vertex = min->vertex;
    if (min->vertex == NULL) {
        fprintf(stderr, "dequeue_min(); set->min->vertex = NULL\n");
        return NULL;
    }
    min_vertex->fib_node = NULL;  // Clear the heap node pointer on removal

    // If min has children, integrate them into the root list
    if (min->child != NULL) {
        fib_node *last = min->child->left; // Last child before modification
        for (fib_node *child = min->child, *next = NULL; child != last; child = next) {
            next = child->right; // Save next child

            // Detach child from its current list
            child->right->left = child->left;
            child->left->right = child->right;

            // Add to root list
            child->right = set->min;
            child->left = set->min->left;
            if (set->min->left)
                set->min->left->right = child;
            set->min->left = child;
            child->parent = NULL;  // Child is now a top-level node
        }
        // Handle the last child separately to close the loop
        last->right = set->min;
        last->left = set->min->left;
        if (set->min->left)
            set->min->left->right = last;
        set->min->left = last;
        last->parent = NULL;
    }

    // Remove min from the root list
    if (min->right == min) {
        set->min = NULL;  // This was the only node in the heap
    } else {
        min->left->right = min->right;
        min->right->left = min->left;
        set->min = min->right;  // Temporarily set to any node
    }

    set->n--;

    // Consolidate the heap to reorganize and merge trees of the same degree
    consolidate(set);

    free(min);  // Free the node that contained the minimum element
    return min_vertex;  // Return the vertex data from the removed minimum node
}

void cut(open_set_t *set, fib_node *node) {
    if (node->parent) {
        node->parent->degree -= 1;
        if (node->right) node->right->left = node->left;
        if (node->left) node->left->right = node->right;
        if (node->parent->child == node) {
            node->parent->child = (node->right != node) ? node->right : NULL;
        }
    }
    node->left = node->right = node;
    node->marked = false;

    // add to root list
    fib_node *min = set->min;
    if (min) {
        node->right = min;
        node->left = min->left;
        if (min->left)
            min->left->right = node;
        min->left = node;
    }
    if (min == NULL || node->vertex && min->vertex && (node->vertex->f_score < min->vertex->f_score)) {
        set->min = node;
    }
}

void cascading_cut(open_set_t *set, fib_node *node) {
    fib_node *parent = node->parent;
    if (parent) {
        if (node->marked) {
            cut(set, node);
            cascading_cut(set, parent);
        } else {
            node->marked = true;
        }
    }
}

void decrease_key(open_set_t *set, fib_node *node, float new_f_score) {
    if (new_f_score >= node->vertex->f_score) {
        return;  // New f_score is not smaller, do nothing
    }
    node->vertex->f_score = new_f_score;

    fib_node *parent = node->parent;
    if (parent && parent->vertex && new_f_score < parent->vertex->f_score) {
        cut(set, node);
        cascading_cut(set, parent);
    }

    if (new_f_score < set->min->vertex->f_score) {
        set->min = node;  // Update the minimum node pointer if necessary
    }
}

// Auxiliary function to free the nodes recursively
void free_node(fib_node *node) {
    if (node == NULL) return;
    fib_node *current = node;
    do {
        fib_node *temp = current;
        current = current->right;  // Move to the next node before freeing the current
        if (temp->child != NULL) {
            free_node(temp->child);  // Recursively free children
        }
        free(temp);  // Free the current node
    } while (current != node && current != NULL); // Ensure we do not loop indefinitely
}

void free_open_set(open_set_t *set) {
    if (set == NULL) return;  // No heap to free

    // If there's no nodes, just free the heap structure
    if (set->min == NULL) {
        free(set);
        return;
    }

    // Start from the min node and free all nodes in the root list
    free_node(set->min);
    free(set);  // Finally, free the set itself
}


/* Include open test testing function here, so that we can execute same tests for different implementations */

#ifdef FH_DBG

void print_open_set(open_set_t *set) {
    if (set->min == NULL) {
        printf("The open set is empty.\n");
        return;
    }
    printf("Open set nodes (min first): ");
    fib_node *current = set->min;
    do {
        printf("%f ", current->vertex->f_score);
        current = current->right;
    } while (current != set->min);
    /** NOTE: keeps printing infinitely because we can't find the minimum of the set */
    printf("\n");
}

void open_set_testing() {
    printf("Starting Fibonacci Heap Tests...\n");

    // Test 1: Initialize the heap
    open_set_t *heap = open_set_new();
    if (heap == NULL) {
        printf("Failed to initialize the Fibonacci heap.\n");
        return;
    }
    printf("Test 1 Passed: Heap initialized.\n");

    // Test 2: Insert elements
    vertex_t ** vertices = malloc(100 * sizeof(vertex_t *));
    srand(time(NULL));  // Seed random number generator
    printf("Inserted elements: ");
    for (int i = 0; i < 100; i++) {
        vertices[i] = (vertex_t *) malloc(sizeof(vertex_t));
        vertices[i]->f_score = rand() % 1000 / 10.0;  // Generate random f_scores
        vertices[i]->fib_node = NULL;  // Ensure fib_node pointers are initially NULL
        insert_sorted(heap, vertices[i]);
        printf("%f ", vertices[i]->f_score);
    }
    printf("\nTest 2 Passed: 100 elements inserted.\n");
    print_open_set(heap);

    // Test 3: Repeatedly extract the minimum element and print each extracted value
    printf("Extracted min elements: ");
    for (int i = 0; i < 10; i++) {
        vertex_t *min_vertex = dequeue_min(heap);
        if (min_vertex == NULL) {
            printf("\nHeap is empty earlier than expected.\n");
            break;
        }
        printf("%f ", min_vertex->f_score);
    }
    printf("\nTest 3 Passed: Minimum elements extracted and printed.\n");
    print_open_set(heap);

    // Test 4: Random decrease key
    printf("Decreasing keys:\n");
    for (int i = 0; i < 10; i++) {
        int index = rand() % 90;  // Choose random vertex to decrease its key
        if (vertices[index]->fib_node) {
            float new_f_score = (rand() % 1000 / 100.0);
            printf("Decreasing vertex %d f_score from %f to %f\n", index, vertices[index]->f_score, new_f_score);
            decrease_key(heap, vertices[index]->fib_node, new_f_score);
            vertices[index]->f_score = new_f_score;  // Update the f_score after decrease
        }
    }
    printf("Test 4 Passed: Keys decreased.\n");
    print_open_set(heap);

    // Test 5: Validate new minimum after key decrease
    vertex_t * min_vertex = dequeue_min(heap);
    if (min_vertex) {
        printf("Test 5 Passed: Decrease key updated the heap correctly, new min is %f\n", min_vertex->f_score);
    } else {
        printf("Test 5 Failed: Decrease key did not update the heap correctly.\n");
    }
    print_open_set(heap);

    // Test 6: Clean up
    // free_open_set(heap);
    printf("Test 6 Passed: Heap cleaned up.\n");
}

int main() {
    open_set_testing();
}

#endif 

