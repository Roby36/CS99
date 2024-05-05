
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
    node->degree = 0;
    node->parent = NULL;
    node->child = NULL;
    node->marked = false;
    node->left = node->right = node;

    if (set->min == NULL) {
        set->min = node;
    } else {
        node->right = set->min;
        node->left = set->min->left;
        set->min->left->right = node;
        set->min->left = node;
        if (new_vertex->f_score < set->min->vertex->f_score) {
            set->min = node;
        }
    }
    set->n++;
}

vertex_t *dequeue_min(open_set_t *set) {
    if (set->min == NULL) {
        return NULL;  // If there are no nodes in the heap, return NULL.
    }

    fib_node *min = set->min;
    vertex_t *min_vertex = min->vertex;

    // If min has children, integrate them into the root list
    if (min->child != NULL) {
        fib_node *child = min->child;
        do {
            fib_node *next = child->right;
            // Detach child from its sibling list
            if (child->right != child) {  // More than one child
                child->right->left = child->left;
                child->left->right = child->right;
            }
            // Now add the child to the root list of the heap
            if (set->min->right == set->min) {  // Only one node in root list
                set->min->right = set->min->left = child;
                child->right = child->left = set->min;
            } else {  // Insert into the existing root list
                child->right = set->min;
                child->left = set->min->left;
                set->min->left->right = child;
                set->min->left = child;
            }
            child->parent = NULL;  // Child is now a top-level node
            child = next;
        } while (child != min->child);
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
            if (x->vertex->f_score > y->vertex->f_score) {
                fib_node *temp = x;
                x = y;
                y = temp;
            }
            if (y == set->min) {
                set->min = x;
            }
            link(y, x);  // Link y into x's child list
            degreeArray[d] = NULL;
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
                if (degreeArray[i]->vertex->f_score < set->min->vertex->f_score) {
                    set->min = degreeArray[i];
                }
            }
        }
    }

    free(degreeArray);
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
        x->child->left->right = y;
        x->child->left = y;
    }
    y->parent = x;
    x->degree++;
    y->marked = false;
}


void decrease_key(open_set_t *set, fib_node *node, float new_f_score) {
    if (new_f_score >= node->vertex->f_score) {
        return;  // New f_score is not smaller, do nothing
    }
    node->vertex->f_score = new_f_score;

    fib_node *parent = node->parent;
    if (parent && new_f_score < parent->vertex->f_score) {
        cut(set, node);
        cascading_cut(set, parent);
    }

    if (new_f_score < set->min->vertex->f_score) {
        set->min = node;  // Update the minimum node pointer if necessary
    }
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
        min->left->right = node;
        min->left = node;
    }
    if (min == NULL || node->vertex->f_score < min->vertex->f_score) {
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




